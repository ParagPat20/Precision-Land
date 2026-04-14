#!/usr/bin/env python3
import os
import pwd
import stat
import subprocess
import sys
from pathlib import Path


SERVICE_NAME = "precision-land.service"
TMUX_SCRIPT_REL = "scripts/run_precision_land_tmux.sh"


def detect_project_dir() -> Path:
    """Repo root: parent of the directory containing this installer."""
    return Path(__file__).resolve().parent.parent


def detect_service_user() -> str:
    """User for systemd User= (not root when using sudo)."""
    if os.environ.get("SUDO_USER"):
        return os.environ["SUDO_USER"]
    u = pwd.getpwuid(os.getuid()).pw_name
    if u == "root":
        print(
            "ERROR: Cannot infer desktop user. Run: sudo -E python3 ... "
            "from your login account, or: sudo -u jech python3 ...",
            file=sys.stderr,
        )
        raise SystemExit(1)
    return u


def user_home(user: str) -> Path:
    return Path(pwd.getpwnam(user).pw_dir)


def ensure_executable(path: Path) -> None:
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def write_service_file(service_path: Path, project_dir: Path, user: str) -> None:
    tmux_script = project_dir / TMUX_SCRIPT_REL
    xauth = user_home(user) / ".Xauthority"
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
Environment=XAUTHORITY={xauth}
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
    project_dir = detect_project_dir()
    user = detect_service_user()
    tmux_script = project_dir / TMUX_SCRIPT_REL
    if not tmux_script.exists():
        print(f"ERROR: tmux runner not found at {tmux_script}", file=sys.stderr)
        return 1

    print(f"precision-land: user={user} project_dir={project_dir}")

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


