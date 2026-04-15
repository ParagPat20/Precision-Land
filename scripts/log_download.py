from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
import time

# ================= CONFIG =================
CONNECTION = '/dev/ttyUSB0'
BAUD = 115200
REMOTE_DIR = '/APM/LOGS'   # ArduPilot log directory
# ==========================================


def connect():
    print("Connecting...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    master.wait_heartbeat()
    print("Connected")
    return master


def list_logs(ftp):
    print("\nFetching log list...")

    # cmd_list returns a MAVFTPReturn (not an iterable). Directory entries are in ret.directory_listing.
    ftp.cmd_ftp(["list", REMOTE_DIR])
    ret = ftp.process_ftp_reply("ListDirectory", timeout=20)
    if ret is None or getattr(ret, "directory_listing", None) is None:
        print("Failed to list directory")
        return []

    logs = []
    for entry in ret.directory_listing:
        name = getattr(entry, "name", "")
        if isinstance(name, str) and name.lower().endswith(".bin"):
            logs.append(name)

    logs.sort()

    for i, log in enumerate(logs):
        print(f"{i+1}. {log}")

    return logs

def download_file(ftp, remote_file, local_file):
    print(f"\nDownloading {remote_file} → {local_file}")

    start_time = time.time()
    last_time = start_time
    last_bytes = 0

    def progress_callback(transferred, total):
        nonlocal last_time, last_bytes

        now = time.time()
        if now - last_time >= 1:
            speed = (transferred - last_bytes) / (now - last_time) / 1024
            percent = (transferred / total) * 100

            print(
                f"\r{percent:5.1f}% | {speed:6.1f} KB/s | {transferred}/{total} bytes",
                end=""
            )

            last_time = now
            last_bytes = transferred

    # Download uses cmd_get + process_ftp_reply loop inside pymavlink.mavftp.
    # progress_callback signature in this wrapper is (fraction_done: float).
    total_bytes_hint = None

    def progress_fraction(fraction):
        nonlocal total_bytes_hint
        # total size is discovered on OpenFileRO and stored on the MAVFTP instance.
        total = int(getattr(ftp, "remote_file_size", 0) or 0)
        if total > 0:
            total_bytes_hint = total
        transferred = int((fraction or 0) * (total_bytes_hint or 0)) if total_bytes_hint else 0
        if total_bytes_hint:
            progress_callback(transferred, total_bytes_hint)

    ftp.cmd_get([remote_file, local_file], progress_callback=progress_fraction)
    ret = ftp.process_ftp_reply("get", timeout=500)
    if ret is not None and getattr(ret, "error_code", 0):
        ret.display_message()

    print("\nDownload complete")


def main():
    master = connect()

    ftp = MAVFTP(master, target_system=master.target_system, target_component=master.target_component)

    logs = list_logs(ftp)

    if not logs:
        print("No logs found")
        return

    latest_log = logs[-1]

    remote_path = f"{REMOTE_DIR}/{latest_log}"
    local_path = latest_log

    download_file(ftp, remote_path, local_path)


if __name__ == "__main__":
    main()