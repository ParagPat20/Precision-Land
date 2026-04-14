#!/usr/bin/env bash
set -euo pipefail

# Repo root = parent of this script's directory (works on any user/path)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

SESSION_NAME="precision_land"
PYTHON_CMD="python3"
MAIN_PY="${PROJECT_DIR}/src/main.py"

# DISPLAY for OpenCV window; XAUTHORITY follows login user (systemd sets HOME for User=)
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}"

# Create tmux session if it doesn't exist
if ! tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  tmux new-session -d -s "${SESSION_NAME}" -c "${PROJECT_DIR}"
  tmux set-option -t "${SESSION_NAME}" mouse on
  tmux send-keys -t "${SESSION_NAME}" "${PYTHON_CMD} ${MAIN_PY}" C-m
fi

# Keep the script running as long as the tmux session exists
while tmux has-session -t "${SESSION_NAME}" 2>/dev/null; do
  sleep 5
done

exit 0


