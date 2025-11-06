#!/usr/bin/env python3
import os
import stat
import subprocess
import sys
from pathlib import Path


SERVICE_NAME = "precision-land.service"
TMUX_SCRIPT_REL = "scripts/run_precision_land_tmux.sh"


def ensure_executable(path: Path) -> None:
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def write_service_file(service_path: Path, project_dir: Path, user: str) -> None:
    tmux_script = project_dir / TMUX_SCRIPT_REL
    unit = f"""
[Unit]
Description=Precision-Land tmux runner
Wants=network-online.target display-manager.service
After=network-online.target display-manager.service

[Service]
Type=simple
User={user}
WorkingDirectory={project_dir}
Environment=PYTHONUNBUFFERED=1
Environment=DISPLAY=:0
Environment=XAUTHORITY=/home/{user}/.Xauthority
ExecStart={tmux_script}
Restart=always
RestartSec=5

[Install]
WantedBy=graphical.target
""".lstrip()
    service_path.write_text(unit)


def run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True)


def main() -> int:
    user = "jecon2"
    project_dir = Path(f"/home/{user}/Precision-Land").resolve()
    tmux_script = project_dir / TMUX_SCRIPT_REL
    if not tmux_script.exists():
        print(f"ERROR: tmux runner not found at {tmux_script}", file=sys.stderr)
        return 1

    # Ensure tmux script is executable
    ensure_executable(tmux_script)

    service_path = Path("/etc/systemd/system") / SERVICE_NAME

    if os.geteuid() != 0:
        print("This script must run as root (use sudo).", file=sys.stderr)
        return 1

    write_service_file(service_path, project_dir, user)

    # Reload systemd and enable service
    run(["systemctl", "daemon-reload"])
    run(["systemctl", "enable", SERVICE_NAME])

    # Optionally start immediately
    if "--start" in sys.argv:
        run(["systemctl", "restart", SERVICE_NAME])

    print(f"Installed service at {service_path}")
    print("Enable/Start status:")
    subprocess.run(["systemctl", "status", SERVICE_NAME, "--no-pager", "-l"])  # no check
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


