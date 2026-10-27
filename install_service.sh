#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="precision-land.service"
TARGET="/etc/systemd/system/${SERVICE_NAME}"

echo "=================================================="
echo "Installing Precision-Land Autonomous Drone Service"
echo "=================================================="

# 1. Make runner scripts executable
chmod +x "${SCRIPT_DIR}/scripts/run_precision_land_tmux.sh" 2>/dev/null || true

# 3. Install systemd service
echo "[INSTALL] Copying service unit to ${TARGET}..."
sudo cp "${SCRIPT_DIR}/scripts/${SERVICE_NAME}" "${TARGET}"
sudo chmod 644 "${TARGET}"

# 4. Reload and enable
echo "[INSTALL] Reloading systemd daemon and enabling service..."
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "=================================================="
echo "Precision-Land Service Installed & Started!"
echo "Status check:"
echo "=================================================="
sudo systemctl status "${SERVICE_NAME}" --no-pager -l || true
