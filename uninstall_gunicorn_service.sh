#!/bin/bash

SERVICE_NAME="gunicorn_web_server"
WORK_DIR="/home/nckucsie/workspace/pros_web_server"
VENV_DIR="$WORK_DIR/venv"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

echo "🛑 停止並停用 systemd 服務..."
sudo systemctl stop "$SERVICE_NAME"
sudo systemctl disable "$SERVICE_NAME"

echo "🗑️ 移除 systemd service 檔案..."
if [ -f "$SERVICE_FILE" ]; then
    sudo rm "$SERVICE_FILE"
    sudo systemctl daemon-reload
    sudo systemctl reset-failed
else
    echo "⚠️ 找不到 $SERVICE_FILE，可能已手動刪除。"
fi

echo "❌ 刪除虛擬環境: $VENV_DIR ..."
if [ -d "$VENV_DIR" ]; then
    rm -rf "$VENV_DIR"
else
    echo "⚠️ 找不到虛擬環境目錄。"
fi

# ✅ 若你希望也刪除整個專案資料夾，可以取消下面這行的註解：
# echo "💣 刪除整個專案資料夾 $WORK_DIR ..."
# rm -rf "$WORK_DIR"

echo "✅ 清除完成。"
