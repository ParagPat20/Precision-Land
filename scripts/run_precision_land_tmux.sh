#!/usr/bin/env bash
set -euo pipefail

# Ensure standard binary paths are available under systemd
export PATH="/usr/local/bin:/usr/bin:/bin:${PATH:-}"

# Repo root = parent of this script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
USER_HOME="${HOME:-/home/jech}"

SESSION_PL="precision_land"
SESSION_VIDEO="video_feed"
SESSION_MAV="mavlink"
SESSION_WEB="fpv_web"

PYTHON_CMD="python3"
MAIN_PY="${PROJECT_DIR}/src/main.py"
MEDIAMTX_CFG="${USER_HOME}/mediamtx.yml"
MAVPROXY_SH="${USER_HOME}/start_mavproxy.sh"
WEB_DIR="${USER_HOME}"

# Fallback config locations inside project if missing in home
if [[ ! -f "${MEDIAMTX_CFG}" && -f "${PROJECT_DIR}/mediamtx.yml" ]]; then
  MEDIAMTX_CFG="${PROJECT_DIR}/mediamtx.yml"
fi
if [[ ! -f "${MAVPROXY_SH}" && -f "${PROJECT_DIR}/start_mavproxy.sh" ]]; then
  MAVPROXY_SH="${PROJECT_DIR}/start_mavproxy.sh"
fi

# DISPLAY for OpenCV window if desktop is active
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-${USER_HOME}/.Xauthority}"

# Clean shutdown handler for all managed tmux sessions
cleanup() {
  echo "[PRECISION-LAND SERVICE] Stopping all managed tmux sessions..."
  tmux kill-session -t "${SESSION_PL}" 2>/dev/null || true
  tmux kill-session -t "${SESSION_MAV}" 2>/dev/null || true
  tmux kill-session -t "${SESSION_VIDEO}" 2>/dev/null || true
  tmux kill-session -t "${SESSION_WEB}" 2>/dev/null || true
  exit 0
}
trap cleanup SIGINT SIGTERM SIGHUP

echo "[PRECISION-LAND SERVICE] Starting unified autonomous drone stack..."

# 1. Video Relay (MediaMTX)
start_video() {
  if ! tmux has-session -t "${SESSION_VIDEO}" 2>/dev/null; then
    echo "[SERVICE] Launching MediaMTX video relay..."
    tmux new-session -d -s "${SESSION_VIDEO}" "mediamtx ${MEDIAMTX_CFG}"
    sleep 1
  fi
}

# 2. Telemetry Multiplexer (MAVProxy / MAVLink Router)
start_mavlink() {
  if ! tmux has-session -t "${SESSION_MAV}" 2>/dev/null; then
    echo "[SERVICE] Launching MAVLink telemetry multiplexer..."
    tmux new-session -d -s "${SESSION_MAV}" "${MAVPROXY_SH}"
    sleep 2
  fi
}

# 3. FPV Web Viewer Server (Port 8080)
start_web() {
  if ! tmux has-session -t "${SESSION_WEB}" 2>/dev/null; then
    echo "[SERVICE] Launching FPV Web Viewer server on port 8080..."
    tmux new-session -d -s "${SESSION_WEB}" -c "${WEB_DIR}" "python3 -m http.server 8080"
    sleep 1
  fi
}

# 4. Precision Landing Engine (Vision + ArUco + Camera + HUD + Firebase + Servos)
start_precision_land() {
  if ! tmux has-session -t "${SESSION_PL}" 2>/dev/null; then
    echo "[SERVICE] Launching Precision-Land autonomy engine..."
    tmux new-session -d -s "${SESSION_PL}" -c "${PROJECT_DIR}"
    # Pre-lock VCM autofocus motor on Arducam 64MP to hyperfocal distance (sharp from 0.8m to infinity)
    for subdev in /dev/v4l-subdev3 /dev/v4l-subdev1 /dev/v4l-subdev2; do
      if [[ -e "${subdev}" ]]; then
        v4l2-ctl -d "${subdev}" --set-ctrl=focus_absolute=160 2>/dev/null || true
        break
      fi
    done
    # Runs headless with autofocus locked and vertical flip enabled
    tmux send-keys -t "${SESSION_PL}" "${PYTHON_CMD} ${MAIN_PY} --no-video --no-servo --focus auto --vflip" C-m
  fi
}

# Start all subsystems
start_video
start_mavlink
start_web
start_precision_land

echo "[PRECISION-LAND SERVICE] All subsystems launched. Entering supervisor loop..."

# Continuous supervisor watchdog loop
while true; do
  start_video
  start_mavlink
  start_web
  start_precision_land
  sleep 5
done
