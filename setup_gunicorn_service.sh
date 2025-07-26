#!/bin/bash

# === 基本參數 ===
USER_NAME="$(whoami)"
WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="gunicorn_web_server"
VENV_DIR="$WORK_DIR/venv"
PYTHON_BIN="$VENV_DIR/bin/python3"
GUNICORN_BIN="$VENV_DIR/bin/gunicorn"
REQUIREMENTS_FILE="$WORK_DIR/requirements.txt"

echo "🔧 [1/5] 建立虛擬環境..."
python3 -m venv "$VENV_DIR"

echo "📦 [2/5] 安裝 requirements.txt..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install -r "$REQUIREMENTS_FILE"
deactivate

echo "🛠️ [3/5] 建立 systemd service 檔案..."

SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Gunicorn Web Server
After=network.target

[Service]
User=$USER_NAME
WorkingDirectory=$WORK_DIR
ExecStart=$GUNICORN_BIN --workers 4 --bind 0.0.0.0:6000 main:app
Restart=always
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 [4/5] 重新載入 systemd 並啟用開機啟動..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo "🚀 [5/5] 啟動服務..."
sudo systemctl start "$SERVICE_NAME"

echo "✅ 完成！你可以使用以下指令查看狀態："
echo "  sudo systemctl status $SERVICE_NAME"
