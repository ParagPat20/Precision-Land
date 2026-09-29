#!/usr/bin/env python3
"""
Precision-Land Complete Systemd Service Installer.
Installs and activates all drone avionics subsystems on boot:
  1. Precision-Land (Vision tracking, ArUco detection, Arducam 64MP OwlSight, HUD, Firebase dispatch, Servos)
  2. Video Feed (MediaMTX low-latency RTSP & WebRTC streaming relay)
  3. Telemetry (MAVLink router & multiplexer over Ethernet to CUAV V6X and GCS)
  4. FPV Web Viewer (HTTP server on port 8080)
"""

from __future__ import annotations

import os
import sys
import stat
import subprocess
import getpass
from pathlib import Path

SERVICE_NAME = "precision-land.service"
TMUX_RUNNER_REL = Path("scripts/run_precision_land_tmux.sh")

try:
    import pwd
except ImportError:
    pwd = None


def detect_project_dir() -> Path:
    return Path(__file__).resolve().parent.parent


def detect_user() -> str:
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        return sudo_user
    if pwd is not None and hasattr(os, "getuid"):
        try:
            u = pwd.getpwuid(os.getuid()).pw_name
            if u != "root":
                return u
        except Exception:
            pass
    return "jech"


def user_home(user: str) -> Path:
    if pwd is not None:
        try:
            return Path(pwd.getpwnam(user).pw_dir)
        except Exception:
            pass
    return Path(f"/home/{user}")


def fix_line_endings_and_chmod(path: Path) -> None:
    if not path.exists():
        return
    # Convert CRLF to LF
    content = path.read_bytes()
    if b"\r\n" in content:
        content = content.replace(b"\r\n", b"\n")
        path.write_bytes(content)
        print(f"[OK] Normalized line endings (CRLF -> LF) on {path.name}")
    # Chmod +x
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    print(f"[OK] Executable permissions set on {path.name}")


def ensure_config_files(home_dir: Path, project_dir: Path) -> None:
    # 1. mediamtx.yml
    target_mtx = home_dir / "mediamtx.yml"
    if not target_mtx.exists():
        mtx_content = """logLevel: info
logDestinations: [stdout]

rtsp: yes
rtspTransports: [tcp, udp]
rtspAddress: :8554

webrtc: yes
webrtcAddress: :8889
webrtcAdditionalHosts: [192.168.50.1, 10.87.142.237, 192.168.144.15]

hls: yes
hlsAddress: :8888

paths:
  cam: {}
"""
        target_mtx.write_text(mtx_content)
        print(f"[OK] Generated MediaMTX config at {target_mtx}")

    # 2. start_mavproxy.sh
    target_mav = home_dir / "start_mavproxy.sh"
    if not target_mav.exists():
        mav_content = """#!/usr/bin/env bash
exec python3 -m MAVProxy.mavproxy \\
  --master=udpout:192.168.144.14:14551 \\
  --out=127.0.0.1:14555 \\
  --out=192.168.50.255:14550 \\
  --out=10.87.142.255:14550 \\
  --out=udpin:0.0.0.0:14550 \\
  --daemon
"""
        target_mav.write_text(mav_content)
        fix_line_endings_and_chmod(target_mav)
        print(f"[OK] Generated MAVProxy launcher at {target_mav}")
    else:
        # Ensure the --out port is correct (not udpin)
        content = target_mav.read_text()
        if "--out=udpin:127.0.0.1:14555" in content:
            content = content.replace("--out=udpin:127.0.0.1:14555", "--out=127.0.0.1:14555")
            target_mav.write_text(content)
        fix_line_endings_and_chmod(target_mav)


def write_systemd_service(service_path: Path, project_dir: Path, user: str, home_dir: Path) -> None:
    runner_script = project_dir / TMUX_RUNNER_REL
    xauth = home_dir / ".Xauthority"
    unit_content = f"""[Unit]
Description=Precision-Land Autonomous Drone Stack (Vision, Video, MAVLink, Firebase)
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User={user}
WorkingDirectory={project_dir}
Environment=PYTHONUNBUFFERED=1
Environment=HOME={home_dir}
Environment=DISPLAY=:0
Environment=XAUTHORITY={xauth}
Environment=PATH=/usr/local/bin:/usr/bin:/bin
ExecStart={runner_script}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
"""
    service_path.write_text(unit_content)
    print(f"[OK] Installed systemd unit at {service_path}")


def run_cmd(cmd: list[str], check: bool = True) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(cmd))
    return subprocess.run(cmd, check=check, text=True)


def install_service() -> int:
    if os.name == "posix" and os.geteuid() != 0:
        print("[!] Root privileges required. Re-running with sudo...")
        return subprocess.run(["sudo", sys.executable, *sys.argv]).returncode

    project_dir = detect_project_dir()
    user = detect_user()
    home = user_home(user)

    print(f"=== Installing Precision-Land Avionics Service ===")
    print(f"Project Directory : {project_dir}")
    print(f"Target User       : {user}")
    print(f"User Home         : {home}")

    runner_script = project_dir / TMUX_RUNNER_REL
    if not runner_script.exists():
        print(f"[ERROR] Runner script not found: {runner_script}", file=sys.stderr)
        return 1

    fix_line_endings_and_chmod(runner_script)
    ensure_config_files(home, project_dir)

    service_path = Path("/etc/systemd/system") / SERVICE_NAME
    write_systemd_service(service_path, project_dir, user, home)

    print("[*] Reloading systemd daemon...")
    run_cmd(["systemctl", "daemon-reload"])

    print(f"[*] Enabling {SERVICE_NAME} to launch automatically on reboot...")
    run_cmd(["systemctl", "enable", SERVICE_NAME])

    print(f"[*] Starting {SERVICE_NAME}...")
    run_cmd(["systemctl", "restart", SERVICE_NAME])

    print("\n=== Installation Complete! ===")
    print("Services running in background:")
    print("  * Video Relay (MediaMTX)       -> tmux attach -t video_feed")
    print("  * Telemetry (MAVLink Router)   -> tmux attach -t mavlink")
    print("  * Web FPV Viewer (:8080)       -> tmux attach -t fpv_web")
    print("  * Autonomy & Vision Engine     -> tmux attach -t precision_land")
    print("\nCheck service status anytime with:")
    print(f"  systemctl status {SERVICE_NAME}")
    return 0


def main() -> int:
    # If explicit CLI commands are given (e.g. status, logs, stop), delegate to pl_service.py
    if len(sys.argv) > 1 and sys.argv[1] not in ["--help", "-h", "install"]:
        pl_service = Path(__file__).resolve().parent / "pl_service.py"
        import runpy
        return runpy.run_path(str(pl_service), run_name="__main__")

    return install_service()


if __name__ == "__main__":
    sys.exit(main())
