#!/usr/bin/env python3
"""
Precision-Land service/session controller.

Why this exists:
- One short CLI to install/manage the systemd service (start/stop/status/logs/etc.)
- Also manage the tmux session created by `scripts/run_precision_land_tmux.sh`

Typical usage on Raspberry Pi (Debian):
- Install/enable the service:
    sudo -E python3 scripts/pl_service.py install --start
- Check service status:
    systemctl status precision-land.service --no-pager -l
    python3 scripts/pl_service.py status
- Tail logs:
    python3 scripts/pl_service.py logs -n 200 -f
- Manage tmux session:
    python3 scripts/pl_service.py session status
    python3 scripts/pl_service.py session logs -n 200
    python3 scripts/pl_service.py session kill
"""

from __future__ import annotations

import argparse
import os
import stat
import subprocess
import sys
import getpass
from pathlib import Path
from typing import Iterable

try:
    # Linux/Unix only. Keep optional so `--help` works on Windows.
    import pwd  # type: ignore
except Exception:  # pragma: no cover - platform dependent
    pwd = None  # type: ignore


# Service and runner defaults (keep in sync with scripts/run_precision_land_tmux.sh)
SERVICE_NAME = "precision-land.service"
TMUX_SESSION_NAME = "precision_land"
TMUX_RUNNER_REL = Path("scripts/run_precision_land_tmux.sh")


def detect_project_dir() -> Path:
    """Repo root: parent of the directory containing this script."""
    return Path(__file__).resolve().parent.parent


def detect_service_user(explicit_user: str | None) -> str:
    """
    User for systemd `User=` when installing the service.

    If invoked via sudo, prefer SUDO_USER (the logged-in desktop user).
    """
    if explicit_user:
        return explicit_user
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        return sudo_user
    # On Windows, fall back to the current login name.
    if pwd is None or not hasattr(os, "getuid"):
        return getpass.getuser()

    u = pwd.getpwuid(os.getuid()).pw_name
    if u == "root":
        print("ERROR: running as root without SUDO_USER; pass --user <name>.", file=sys.stderr)
        raise SystemExit(2)
    return u


def user_home(user: str) -> Path:
    if pwd is None:
        raise RuntimeError("user_home() requires a POSIX host (pwd module unavailable).")
    return Path(pwd.getpwnam(user).pw_dir)


def ensure_executable(path: Path) -> None:
    """Make sure the runner script is executable (systemd ExecStart requires it)."""
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def require_root(action: str) -> None:
    """Installing/writing to /etc/systemd/system requires root."""
    if os.name != "posix":
        print(f"ERROR: '{action}' is intended for Linux systemd hosts.", file=sys.stderr)
        raise SystemExit(2)
    if os.geteuid() != 0:
        print(f"ERROR: '{action}' requires root. Re-run with sudo.", file=sys.stderr)
        raise SystemExit(2)


def run(cmd: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run a command, printing it for transparency."""
    print("+", " ".join(cmd))
    return subprocess.run(cmd, check=check, text=True)


def run_capture(cmd: list[str], *, check: bool = True) -> str:
    """Run a command and return stdout (for status/log outputs)."""
    print("+", " ".join(cmd))
    cp = subprocess.run(cmd, check=check, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return cp.stdout or ""


def write_service_file(service_path: Path, project_dir: Path, user: str) -> None:
    """
    Create/update the systemd unit that runs the tmux runner.

    Notes:
    - DISPLAY/XAUTHORITY are included so OpenCV windows can open on the desktop session.
    - XAUTHORITY is bound to the chosen user's home.
    """
    tmux_script = project_dir / TMUX_RUNNER_REL
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


def systemctl(args: Iterable[str], *, check: bool = True) -> int:
    cmd = ["systemctl", *args]
    return run(cmd, check=check).returncode


def cmd_install(args: argparse.Namespace) -> int:
    require_root("install")

    project_dir = Path(args.project_dir).resolve() if args.project_dir else detect_project_dir()
    user = detect_service_user(args.user)

    tmux_script = project_dir / TMUX_RUNNER_REL
    if not tmux_script.exists():
        print(f"ERROR: tmux runner not found at {tmux_script}", file=sys.stderr)
        return 1

    print(f"precision-land: user={user} project_dir={project_dir}")
    ensure_executable(tmux_script)

    service_path = Path("/etc/systemd/system") / SERVICE_NAME
    write_service_file(service_path, project_dir, user)

    # Reload and enable (safe to re-run).
    systemctl(["daemon-reload"])
    systemctl(["enable", SERVICE_NAME])

    if args.start:
        systemctl(["restart", SERVICE_NAME])

    print(f"Installed/updated service at {service_path}")
    return 0


def cmd_status(_: argparse.Namespace) -> int:
    systemctl(["status", SERVICE_NAME, "--no-pager", "-l"], check=False)
    return 0


def cmd_start(_: argparse.Namespace) -> int:
    systemctl(["start", SERVICE_NAME])
    return 0


def cmd_stop(_: argparse.Namespace) -> int:
    systemctl(["stop", SERVICE_NAME])
    return 0


def cmd_restart(_: argparse.Namespace) -> int:
    systemctl(["restart", SERVICE_NAME])
    return 0


def cmd_enable(_: argparse.Namespace) -> int:
    systemctl(["enable", SERVICE_NAME])
    return 0


def cmd_disable(_: argparse.Namespace) -> int:
    systemctl(["disable", SERVICE_NAME])
    return 0


def cmd_logs(args: argparse.Namespace) -> int:
    cmd = ["journalctl", "-u", SERVICE_NAME, "--no-pager"]
    if args.lines is not None:
        cmd += ["-n", str(args.lines)]
    if args.follow:
        cmd.append("-f")
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=False)
    return 0


def tmux(args: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return run(["tmux", *args], check=check)


def tmux_capture(args: list[str], *, check: bool = True) -> str:
    return run_capture(["tmux", *args], check=check)


def tmux_has_session() -> bool:
    cp = subprocess.run(
        ["tmux", "has-session", "-t", TMUX_SESSION_NAME],
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return cp.returncode == 0


def cmd_session_status(_: argparse.Namespace) -> int:
    if tmux_has_session():
        print(f"tmux session '{TMUX_SESSION_NAME}': running")
        # Show a quick summary (windows/panes) without failing the CLI if tmux is odd.
        tmux(["list-windows", "-t", TMUX_SESSION_NAME], check=False)
        return 0
    print(f"tmux session '{TMUX_SESSION_NAME}': not running")
    return 1


def cmd_session_kill(_: argparse.Namespace) -> int:
    if not tmux_has_session():
        print(f"tmux session '{TMUX_SESSION_NAME}' is not running.")
        return 0
    tmux(["kill-session", "-t", TMUX_SESSION_NAME])
    print(f"Killed tmux session '{TMUX_SESSION_NAME}'.")
    return 0


def cmd_session_logs(args: argparse.Namespace) -> int:
    if not tmux_has_session():
        print(f"tmux session '{TMUX_SESSION_NAME}' is not running.", file=sys.stderr)
        return 1

    # Capture recent output from the active pane.
    # -S supports negative offsets in newer tmux; if unsupported, we fall back gracefully.
    lines = int(args.lines) if args.lines is not None else 200
    capture_cmd = ["capture-pane", "-p", "-t", TMUX_SESSION_NAME]
    try:
        # Try to capture only the last N lines.
        out = tmux_capture([*capture_cmd, "-S", f"-{lines}"], check=True)
    except subprocess.CalledProcessError:
        out = tmux_capture(capture_cmd, check=False)
        out_lines = out.splitlines()
        out = "\n".join(out_lines[-lines:])

    print(out.rstrip("\n"))
    return 0


def cmd_session_attach(_: argparse.Namespace) -> int:
    """
    Attaching is inherently interactive; we still provide a helper that runs tmux attach.
    """
    if not tmux_has_session():
        print(f"tmux session '{TMUX_SESSION_NAME}' is not running.", file=sys.stderr)
        return 1
    # Let tmux take over the terminal.
    os.execvp("tmux", ["tmux", "attach-session", "-t", TMUX_SESSION_NAME])
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pl_service.py", description="Precision-Land service/session control")
    sub = p.add_subparsers(dest="cmd", required=True)

    # Service install/update
    sp = sub.add_parser("install", help="install/update systemd unit (requires sudo)")
    sp.add_argument("--user", help="service User= (defaults to SUDO_USER)")
    sp.add_argument("--project-dir", help="repo root (auto-detected if omitted)")
    sp.add_argument("--start", action="store_true", help="restart service after install")
    sp.set_defaults(func=cmd_install)

    # Service control
    for name, fn, help_text in [
        ("status", cmd_status, "show service status"),
        ("start", cmd_start, "start service"),
        ("stop", cmd_stop, "stop service"),
        ("restart", cmd_restart, "restart service"),
        ("enable", cmd_enable, "enable service at boot"),
        ("disable", cmd_disable, "disable service at boot"),
    ]:
        sp2 = sub.add_parser(name, help=help_text)
        sp2.set_defaults(func=fn)

    sp = sub.add_parser("logs", help="show service logs (journalctl)")
    sp.add_argument("-n", "--lines", type=int, default=200, help="number of lines (default: 200)")
    sp.add_argument("-f", "--follow", action="store_true", help="follow logs")
    sp.set_defaults(func=cmd_logs)

    # Tmux session controls
    sp = sub.add_parser("session", help="manage the tmux session")
    session_sub = sp.add_subparsers(dest="session_cmd", required=True)

    sp2 = session_sub.add_parser("status", help="check whether the tmux session exists")
    sp2.set_defaults(func=cmd_session_status)

    sp2 = session_sub.add_parser("kill", help="kill the tmux session")
    sp2.set_defaults(func=cmd_session_kill)

    sp2 = session_sub.add_parser("logs", help="print recent tmux pane output")
    sp2.add_argument("-n", "--lines", type=int, default=200, help="number of lines (default: 200)")
    sp2.set_defaults(func=cmd_session_logs)

    sp2 = session_sub.add_parser("attach", help="attach interactively to the tmux session")
    sp2.set_defaults(func=cmd_session_attach)

    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())

