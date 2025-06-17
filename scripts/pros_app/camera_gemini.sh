#!/usr/bin/env bash
set -euo pipefail

# 1) 定位到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 2) 引入 utils.sh（必须存在）
UTILS_FILE="$SCRIPT_DIR/utils.sh"
if [[ ! -f "$UTILS_FILE" ]]; then
  echo "Error: utils.sh not found in $SCRIPT_DIR" >&2
  exit 1
fi
source "$UTILS_FILE"

# 3) 调用 main，并使用绝对路径指向你的 docker-compose 文件
COMPOSE_FILE="$SCRIPT_DIR/docker/compose/docker-compose_camera_gemini.yml"
if [[ ! -f "$COMPOSE_FILE" ]]; then
  echo "Error: Compose file not found at $COMPOSE_FILE" >&2
  exit 1
fi

# 执行
main "$COMPOSE_FILE"
