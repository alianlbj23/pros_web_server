#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/utils.sh"
main "$SCRIPT_DIR/docker/compose/docker-compose_camera_gemini.yml"