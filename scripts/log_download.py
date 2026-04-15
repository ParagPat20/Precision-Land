from pymavlink import mavutil
import time
import os

DEVICE = "/dev/ttyUSB0"
BAUD = 921600

CHUNK_SIZE = 90

master = mavutil.mavlink_connection(DEVICE, baud=BAUD)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

target_system = master.target_system
target_component = master.target_component

entries = {}
download_set = set()

def cancel_any_active_log_transfer():
    """Ensure the FC is not stuck streaming LOG_DATA from a previous run."""
    try:
        master.mav.log_request_end_send(target_system, target_component)
        # Give the FC a brief moment to stop the stream.
        time.sleep(0.2)
    except Exception:
        pass


# ----------------------------
# STEP 1: Get log list
# ----------------------------
def get_log_list():
    # If a previous download was interrupted, stop it before requesting the list.
    cancel_any_active_log_transfer()
    master.mav.log_request_list_send(
        target_system,
        target_component,
        0,
        0xFFFF
    )

    print("Requesting log list...")

    start = time.time()
    while time.time() - start < 5:
        msg = master.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
        if msg:
            entries[msg.id] = msg
            print(f"Log {msg.id} size={msg.size}")

    return entries


# ----------------------------
# STEP 2: Download log
# ----------------------------
def download_log(log_id, filename):
    print(f"Downloading log {log_id} → {filename}")

    f = open(filename, "wb")

    # Total size comes from LOG_ENTRY (if present). Used for % and ETA-ish feedback.
    expected_size = int(entries.get(log_id).size) if log_id in entries else 0

    retries = 0
    last_recv = time.time()
    start = time.time()
    last_report = start
    last_report_bytes = 0

    master.mav.log_request_data_send(
        target_system,
        target_component,
        log_id,
        0,
        0xFFFFFFFF
    )

    while True:
        msg = master.recv_match(type='LOG_DATA', blocking=True, timeout=1)

        if msg is None:
            # retry missing
            if time.time() - last_recv > 1:
                print("Retrying missing chunks...")
                retry_missing(log_id)
                retries += 1
                last_recv = time.time()
            # Progress report (even if packets are missing).
            now = time.time()
            if now - last_report >= 2.0:
                size = os.path.getsize(filename) if os.path.exists(filename) else 0
                inst_rate_kbs = ((size - last_report_bytes) / 1024.0) / max(now - last_report, 0.001)
                avg_rate_kbs = (size / 1024.0) / max(now - start, 0.001)
                if expected_size > 0:
                    pct = min(100.0, (100.0 * size) / expected_size)
                    print(f"Progress: {size}/{expected_size} bytes ({pct:.1f}%)  speed={inst_rate_kbs:.1f} kB/s avg={avg_rate_kbs:.1f} kB/s retries={retries}")
                else:
                    print(f"Progress: {size} bytes  speed={inst_rate_kbs:.1f} kB/s avg={avg_rate_kbs:.1f} kB/s retries={retries}")
                last_report = now
                last_report_bytes = size
            continue

        last_recv = time.time()

        if msg.count > 0:
            f.seek(msg.ofs)
            # LOG_DATA.data is a sequence of ints in some pymavlink builds; convert to bytes for file I/O.
            f.write(bytes(msg.data[:msg.count]))
            download_set.add(msg.ofs // CHUNK_SIZE)

        # Progress report (every ~2 seconds) while data is flowing.
        now = time.time()
        if now - last_report >= 2.0:
            f.flush()
            size = os.path.getsize(filename)
            inst_rate_kbs = ((size - last_report_bytes) / 1024.0) / max(now - last_report, 0.001)
            avg_rate_kbs = (size / 1024.0) / max(now - start, 0.001)
            if expected_size > 0:
                pct = min(100.0, (100.0 * size) / expected_size)
                print(f"Progress: {size}/{expected_size} bytes ({pct:.1f}%)  speed={inst_rate_kbs:.1f} kB/s avg={avg_rate_kbs:.1f} kB/s retries={retries}")
            else:
                print(f"Progress: {size} bytes  speed={inst_rate_kbs:.1f} kB/s avg={avg_rate_kbs:.1f} kB/s retries={retries}")
            last_report = now
            last_report_bytes = size

        # finish condition
        if msg.count == 0:
            break

    f.close()

    size = os.path.getsize(filename)
    dt = max(time.time() - start, 0.001)
    speed = (size / 1024.0) / dt
    if expected_size > 0:
        pct = min(100.0, (100.0 * size) / expected_size)
        print(f"Done: {filename} ({size}/{expected_size} bytes, {pct:.1f}%, {dt:.1f}s, {speed:.1f} kB/s, retries={retries})")
    else:
        print(f"Done: {filename} ({size} bytes, {dt:.1f}s, {speed:.1f} kB/s, retries={retries})")


# ----------------------------
# STEP 3: Retry missing chunks
# ----------------------------
def retry_missing(log_id):
    if len(download_set) == 0:
        return

    highest = max(download_set)
    missing = set(range(highest)) - download_set

    for chunk in list(missing)[:20]:
        master.mav.log_request_data_send(
            target_system,
            target_component,
            log_id,
            chunk * CHUNK_SIZE,
            CHUNK_SIZE
        )


# ----------------------------
# MAIN FLOW
# ----------------------------
try:
    logs = get_log_list()

    if not logs:
        print("No logs found")
        exit()

    latest_log = max(logs.keys())
    download_log(latest_log, f"log{latest_log}.bin")
finally:
    # Always end the log transfer session (even on Ctrl+C) so the next run can list logs.
    cancel_any_active_log_transfer()