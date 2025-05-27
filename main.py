from flask import Flask, jsonify
import os
import subprocess

app = Flask(__name__)

BASE_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.join(BASE_DIR, "scripts")

# ✅ 這裡定義 signal_name 與實際腳本的對應關係（含子資料夾）
script_mapping = {
    "star_car": "zlink-bus-servo-driver/car_activate.sh",
    "stop_motor": "motors/stop.sh",
    "reset_system": "system/reset.sh",
    "init_camera": "camera/init.sh",
    # 可繼續新增...
}


def run_shell_script(script_rel_path: str):
    script_path = os.path.join(SCRIPTS_DIR, script_rel_path)

    if not os.path.isfile(script_path):
        return {
            "status": "error",
            "message": f"Script '{script_rel_path}' not found"
        }, 404

    try:
        result = subprocess.run(["bash", script_path], check=True, capture_output=True, text=True)
        return {
            "status": "success",
            "script": script_rel_path,
            "output": result.stdout.strip()
        }, 200
    except subprocess.CalledProcessError as e:
        return {
            "status": "error",
            "message": str(e),
            "stderr": e.stderr
        }, 500


@app.route("/run-script/<signal_name>", methods=["GET"])
def run_script(signal_name):
    if signal_name not in script_mapping:
        return jsonify({
            "status": "error",
            "message": f"No script mapped for signal '{signal_name}'"
        }), 404

    script_rel_path = script_mapping[signal_name]
    response, status = run_shell_script(script_rel_path)
    return jsonify(response), status


@app.route("/")
def hello():
    return "Ready to receive signals!"
