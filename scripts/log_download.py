from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
import time

# ================= CONFIG =================
CONNECTION = '/dev/ttyUSB0'
BAUD = 115200
REMOTE_DIR = '@MAV_LOG'   # ArduPilot log directory
# ==========================================


def connect():
    print("Connecting...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    master.wait_heartbeat()
    print("Connected")
    return master


def list_logs(ftp):
    print("\nFetching log list...")
    files = ftp.listdir(REMOTE_DIR)

    logs = [f for f in files if f.endswith('.BIN') or f.endswith('.bin')]
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

    ftp.get(
        remote_file,
        local_file,
        callback=progress_callback
    )

    print("\nDownload complete")


def main():
    master = connect()

    ftp = MAVFTP(master)

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