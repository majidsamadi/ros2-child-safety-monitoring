#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
PORT="${PORT:-8090}"

echo "Starting laptop webcam streamer..."
echo "Camera index: ${CAMERA_INDEX}"
echo "Port: ${PORT}"
echo ""
echo "Browser test:"
echo "  http://127.0.0.1:${PORT}/video"
echo ""
echo "Docker stream URL:"
echo "  http://host.docker.internal:${PORT}/video"
echo ""

python3 -m pip install --user "opencv-python==4.10.0.84" "numpy==1.26.4"
python3 "${REPO_DIR}/scripts/host_webcam_streamer.py" --camera "${CAMERA_INDEX}" --port "${PORT}"
