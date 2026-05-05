#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker rm -f ros2-child-safety-dev 2>/dev/null || true

docker run -it --rm \
  --name ros2-child-safety-dev \
  -p 8080:8080 \
  -v "${REPO_DIR}:/root/ros2_ws/src/ros2-child-safety-monitoring" \
  osrf/ros:humble-desktop \
  bash
