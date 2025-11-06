#!/usr/bin/env bash
set -euo pipefail

# Configuration
SESSION_NAME="precision_land"
PROJECT_DIR="/home/jecon2/Precision-Land"
PYTHON_CMD="python3"
MAIN_PY="${PROJECT_DIR}/src/main.py"

# Ensure DISPLAY is set for GUI environments (if needed by OpenCV, etc.)
export DISPLAY=":0"
export XAUTHORITY="/home/jecon2/.Xauthority"

# Create tmux session if it doesn't exist
if ! tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  tmux new-session -d -s "${SESSION_NAME}" -c "${PROJECT_DIR}"
  tmux send-keys -t "${SESSION_NAME}" "${PYTHON_CMD} ${MAIN_PY}" C-m
fi

# Keep the script running as long as the tmux session exists
while tmux has-session -t "${SESSION_NAME}" 2>/dev/null; do
  sleep 5
done

exit 0


