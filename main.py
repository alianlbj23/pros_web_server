from flask import Flask, jsonify
import os
import subprocess

app = Flask(__name__)

BASE_DIR    = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.join(BASE_DIR, "scripts")

# signal -> 啟動腳本
script_mapping = {
    "star_car":     "zlink-bus-servo-driver/car_activate.sh",
    "reset_system": "system/reset.sh",
    "init_camera":  "camera/init.sh",
}

# 在 star_car 被触发（启动）后，要关掉哪些容器
# 这里直接写死两个你在 docker ps 看到的容器名
container_mapping = {
    "star_car": [
        "arm_controller_node",
        "microros_agent_usb_rear_wheel"
    ],
    # 其他 signal 对应容器也可以按需添加
}

active_signals: set[str]      = set()
active_containers: dict[str, list[str]] = {}

def run_shell_script(rel_path: str):
    path = os.path.join(SCRIPTS_DIR, rel_path)
    if not os.path.isfile(path):
        return {"status":"error","message":f"{rel_path} not found"}, 404
    try:
        res = subprocess.run(
            ["bash", path],
            check=True, capture_output=True, text=True
        )
        return {"status":"success","output":res.stdout.strip()}, 200
    except subprocess.CalledProcessError as e:
        return {"status":"error","message":str(e),"stderr":e.stderr}, 500

def stop_containers(names: list[str]):
    errors = []
    for name in names:
        try:
            subprocess.run(["docker", "stop", name], check=True,
                           capture_output=True, text=True)
            subprocess.run(["docker", "rm",   name], check=True,
                           capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            errors.append(f"{name}: {e.stderr or e}")
    return errors

@app.route("/run-script/<signal_name>", methods=["GET"])
def run_signal(signal_name: str):
    # --- 停止流程 ---
    if signal_name.endswith("_stop"):
        base = signal_name[:-5]
        was_active = base in active_signals

        # 如果有静态映射的容器列表，就停它们
        containers = container_mapping.get(base, [])
        if containers:
            errs = stop_containers(containers)
            # 不论成功还是部分失败，都清理 state
            active_signals.discard(base)
            active_containers.pop(base, None)
            if errs:
                return jsonify({
                    "status":"error",
                    "message":"Failed to stop some containers",
                    "errors": errs
                }), 500
            return jsonify({
                "status":      "success",
                "action":      "stopped_containers",
                "signal":      base,
                "was_active":  was_active,
                "containers":  containers
            }), 200

        # fallback：没容器映射就返回错误
        return jsonify({
            "status":"error",
            "message":f"No containers mapped to stop for '{base}'"
        }), 404

    # --- 启动流程 ---
    if signal_name not in script_mapping:
        return jsonify({"status":"error","message":f"No script for '{signal_name}'"}), 404
    if signal_name in active_signals:
        return jsonify({"status":"error","message":f"'{signal_name}' already active"}), 409

    # 执行启动脚本
    resp, code = run_shell_script(script_mapping[signal_name])
    if code == 200:
        active_signals.add(signal_name)
        # 记录要停掉的容器列表
        active_containers[signal_name] = container_mapping.get(signal_name, [])
    return jsonify(resp), code

@app.route("/active-signals", methods=["GET"])
def list_active():
    return jsonify({
        "active_signals":  sorted(active_signals),
        "containers":      active_containers
    }), 200

@app.route("/")
def hello():
    return "Ready to receive signals!"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
