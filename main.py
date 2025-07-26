from flask import Flask, jsonify
import os
import subprocess
import threading

app = Flask(__name__)

BASE_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.join(BASE_DIR, "scripts")

script_mapping = {
    "star_car": [
        "pros_app/rosbridge_server.sh",
    ],
    "slam_ydlidar": [
        "pros_app/slam_ydlidar.sh",
    ],
    "slam_oradarlidar": [
        "pros_app/slam_oradarlidar.sh",
    ],
    "store_map": [
        "pros_app/store_map.sh",
    ],
    "localization_ydlidar": [
        "pros_app/localization_ydlidar.sh",
    ],
    "localization_oradarlidar": [
        "pros_app/localization_oradarlidar.sh",
    ],
}

container_mapping = {
    "star_car": ["rosbridge_server"],
    "slam_ydlidar": ["ydlidar", "slam"],
    "localization_ydlidar": ["navigation", "localization", "ydlidar"],
    "slam_oradarlidar": ["oradarlidar", "lidar_filter", "slam"],
    "localization_oradarlidar": ["localization", "lidar_filter", "oradarlidar"],
}

active_signals: set[str] = set()
active_containers: dict[str, list[str]] = {}


def run_shell_script(rel_path_or_paths: str | list[str]):
    if isinstance(rel_path_or_paths, str):
        script_paths_to_execute = [rel_path_or_paths]
        is_single_script = True
    elif isinstance(rel_path_or_paths, list):
        script_paths_to_execute = rel_path_or_paths
        is_single_script = False
        if not script_paths_to_execute:
            return {"status": "error", "message": "Empty script list provided"}, 400
    else:
        return {"status": "error", "message": "Invalid script mapping format"}, 500

    all_individual_outputs = []

    for rel_path_item in script_paths_to_execute:
        if not isinstance(rel_path_item, str):
            return {"status": "error", "message": f"Invalid script path item: {rel_path_item}"}, 500

        path = os.path.join(SCRIPTS_DIR, rel_path_item)
        if not os.path.isfile(path):
            return {
                "status": "error",
                "message": f"Script '{rel_path_item}' not found",
            }, 404

        try:
            res = subprocess.run(
                ["bash", path], check=True, capture_output=True, text=True
            )
            all_individual_outputs.append(
                {"script": rel_path_item, "output": res.stdout.strip()}
            )
        except subprocess.CalledProcessError as e:
            return {
                "status": "error",
                "message": f"Error in script '{rel_path_item}': {e}",
                "stderr": e.stderr.strip(),
                "failed_script": rel_path_item,
            }, 500

    if is_single_script and all_individual_outputs:
        return {"status": "success", "output": all_individual_outputs[0]["output"]}, 200
    else:
        return {"status": "success", "outputs": all_individual_outputs}, 200


def stop_containers(names: list[str]) -> list[str]:
    errors = []
    for name in names:
        try:
            subprocess.run(
                ["docker", "kill", name], check=True, capture_output=True, text=True
            )
            subprocess.run(
                ["docker", "rm", name], check=True, capture_output=True, text=True
            )
        except subprocess.CalledProcessError as e:
            errors.append(f"{name}: {e.stderr or e}")
    return errors


def are_containers_running(names: list[str]) -> bool:
    """
    回傳 True 如果 names 裡有至少一個 container 正在運行中。
    """
    try:
        res = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}"],
            check=True,
            capture_output=True,
            text=True,
        )
        running = set(res.stdout.splitlines())
        return any(name in running for name in names)
    except subprocess.CalledProcessError:
        return False


@app.route("/wheel/<speed_str>", methods=["GET"])
def receive_wheel_speed(speed_str: str):
    try:
        # 將速度字串切割並轉成整數，例如 "1_1_1_1" -> [1, 1, 1, 1]
        wheel_speeds = list(map(int, speed_str.strip().split("_")))

        # 檢查格式是否合理（假設你預期四輪）
        if len(wheel_speeds) != 4:
            return (
                jsonify(
                    {
                        "status": "error",
                        "message": f"Expected 4 wheel speeds, got {len(wheel_speeds)}",
                    }
                ),
                400,
            )

        # ✅ 顯示接收到的訊號（可以改成傳給硬體或儲存）
        print(f"[INFO] Received wheel speed command: {wheel_speeds}")

        return jsonify({"status": "success", "wheel_speeds": wheel_speeds}), 200

    except ValueError:
        return (
            jsonify(
                {
                    "status": "error",
                    "message": "Invalid format. Use integers separated by underscores (e.g., 1_1_1_1)",
                }
            ),
            400,
        )


@app.route("/robot_arm/<joint_str>", methods=["GET"])
def receive_robot_arm_command(joint_str: str):
    try:
        # 將 joint 角度字串轉為 list，例如 "90_45_30_0_180_90" -> [90, 45, 30, 0, 180, 90]
        joint_angles = list(map(int, joint_str.strip().split("_")))

        # 假設機械手臂是 6 軸的話檢查長度
        if len(joint_angles) != 6:
            return (
                jsonify(
                    {
                        "status": "error",
                        "message": f"Expected 6 joint values, got {len(joint_angles)}",
                    }
                ),
                400,
            )

        # ✅ 顯示訊號（或你也可以在這裡 publish 到 ROS 或執行控制）
        print(f"[INFO] Received robot_arm command: {joint_angles}")

        return jsonify({"status": "success", "robot_arm_joints": joint_angles}), 200

    except ValueError:
        return (
            jsonify(
                {
                    "status": "error",
                    "message": "Invalid format. Use integers separated by underscores (e.g., 90_45_30_0_180_90)",
                }
            ),
            400,
        )


@app.route("/run-script/<signal_name>", methods=["GET"])
def run_signal(signal_name: str):
    # 處理 stop 指令
    if signal_name.endswith("_stop"):
        base = signal_name[:-5]
        was_active = base in active_signals
        containers = container_mapping.get(base, [])
        if containers:
            errs = stop_containers(containers)
            active_signals.discard(base)
            active_containers.pop(base, None)
            if errs:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Failed to stop some containers",
                            "errors": errs,
                        }
                    ),
                    500,
                )
            return (
                jsonify(
                    {
                        "status": "success",
                        "action": "stopped_containers",
                        "signal": base,
                        "was_active": was_active,
                        "containers": containers,
                    }
                ),
                200,
            )
        return (
            jsonify(
                {
                    "status": "error",
                    "message": "Failed to stop some containers",
                    "errors": errs
                }), 500
            return jsonify({
                "status":     "success",
                "action":     "stopped_containers",
                "signal":     base,
                "was_active": was_active,
                "containers": containers
            }), 200
        return jsonify({
            "status":  "error",
            "message": f"No containers mapped to stop for '{base}'"
        }), 404

    # 處理 store_map 腳本的情況
    if signal_name == "store_map":
        # 無視目前狀態，直接啟動 store_map 腳本
        threading.Thread(target=run_shell_script, args=(script_mapping["store_map"],)).start()
        return jsonify({"status": "Store map execution started."}), 202

    # 確認有對應的 script
    if signal_name not in script_mapping:
        return jsonify({
            "status":  "error",
            "message": f"No script for '{signal_name}'"
        }), 404

    containers = container_mapping.get(signal_name, [])

    # 檢查是否已經有 container 在跑
    if containers and are_containers_running(containers):
        return jsonify({
            "status":     "error",
            "message":    f"Containers for '{signal_name}' already running",
            "containers": containers
        }), 409

    # 檢查 signal 是否已 activate
    if signal_name in active_signals:
        return jsonify({
            "status":  "error",
            "message": f"'{signal_name}' already active"
        }), 409

    # 啟動腳本
    threading.Thread(
        target=run_shell_script,
        args=(script_mapping[signal_name],)
    ).start()

    active_signals.add(signal_name)
    active_containers[signal_name] = containers

    return jsonify({"status": "Script execution started"}), 202


@app.route("/active-signals", methods=["GET"])
def list_active():
    return (
        jsonify(
            {"active_signals": sorted(active_signals), "containers": active_containers}
        ),
        200,
    )


@app.route("/", methods=["GET"])
def hello():
    return "Ready to receive signals!"


if __name__ == "__main__":
    # Flask 內建 server for debugging; 生產環境用 Gunicorn 啟動
    app.run(host="0.0.0.0", port=5000, debug=True, timeout=120)
