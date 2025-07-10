from flask import Flask, jsonify
import os
import subprocess
import threading

app = Flask(__name__)

BASE_DIR    = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.join(BASE_DIR, "scripts")

script_mapping = {
    "star_car": [
        "pros_app/rosbridge_server.sh",
    ],
    "slam_ydlidar": [
        "pros_app/slam_ydlidar.sh",
    ],
    "store_map": [
        "pros_app/store_map.sh",
    ],
    "localization_ydlidar": [
        "pros_app/localization_ydlidar.sh",
    ],
}


container_mapping = {
    "star_car": [
        "rosbridge_server"
    ],
    "slam_ydlidar": [
        "ydlidar",
        "slam"
    ],
    "localization_ydlidar": [
        "navigation",
        "localization",
        "ydlidar"
    ],
}

active_signals: set[str]      = set()
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
            return {"status": "error", "message": f"Invalid script path item in list: {rel_path_item}"}, 500

        path = os.path.join(SCRIPTS_DIR, rel_path_item)
        if not os.path.isfile(path):
            return {"status": "error", "message": f"Script '{rel_path_item}' not found"}, 404

        try:
            res = subprocess.run(
                ["bash", path],
                check=True, capture_output=True, text=True
            )
            all_individual_outputs.append(
                {"script": rel_path_item, "output": res.stdout.strip()}
            )
        except subprocess.CalledProcessError as e:
            return {
                "status": "error",
                "message": f"Error in script '{rel_path_item}': {str(e)}",
                "stderr": e.stderr.strip(),
                "failed_script": rel_path_item
            }, 500

    if is_single_script and all_individual_outputs:
        return {"status": "success", "output": all_individual_outputs[0]["output"]}, 200
    else:
        return {"status": "success", "outputs": all_individual_outputs}, 200

def stop_containers(names: list[str]):
    errors = []
    for name in names:
        try:
            subprocess.run(["docker", "kill", name], check=True,
                           capture_output=True, text=True)
            subprocess.run(["docker", "rm",   name], check=True,
                           capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            errors.append(f"{name}: {e.stderr or e}")
    return errors

@app.route("/run-script/<signal_name>", methods=["GET"])
def run_signal(signal_name: str):
    if signal_name.endswith("_stop"):
        base = signal_name[:-5]
        was_active = base in active_signals

        containers = container_mapping.get(base, [])
        if containers:
            errs = stop_containers(containers)
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

        return jsonify({
            "status":"error",
            "message":f"No containers mapped to stop for '{base}'"
        }), 404

    if signal_name not in script_mapping:
        return jsonify({"status":"error","message":f"No script for '{signal_name}'"}), 404
    if signal_name in active_signals:
        return jsonify({"status":"error","message":f"'{signal_name}' already active"}), 409

    threading.Thread(target=run_shell_script, args=(script_mapping[signal_name],)).start()
    active_signals.add(signal_name)
    active_containers[signal_name] = container_mapping.get(signal_name, [])
    return jsonify({"status": "Script execution started"}), 202

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
    app.run(host="0.0.0.0", port=5000, debug=True, timeout=120)
