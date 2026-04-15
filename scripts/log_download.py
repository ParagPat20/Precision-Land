from pymavlink import mavutil
import time
import os

DEVICE = "/dev/ttyUSB0"
BAUD = 115200

CHUNK_SIZE = 90

master = mavutil.mavlink_connection(DEVICE, baud=BAUD)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

target_system = master.target_system
target_component = master.target_component

entries = {}
download_set = set()


# ----------------------------
# STEP 1: Get log list
# ----------------------------
def get_log_list():
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

    offset = 0
    retries = 0
    last_recv = time.time()

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
            continue

        last_recv = time.time()

        if msg.count > 0:
            f.seek(msg.ofs)
            # LOG_DATA.data is a sequence of ints in some pymavlink builds; convert to bytes for file I/O.
            f.write(bytes(msg.data[:msg.count]))
            download_set.add(msg.ofs // CHUNK_SIZE)

        # finish condition
        if msg.count == 0:
            break

    f.close()

    size = os.path.getsize(filename)
    print(f"Done: {filename} ({size} bytes, retries={retries})")


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
logs = get_log_list()

if not logs:
    print("No logs found")
    exit()

latest_log = max(logs.keys())
download_log(latest_log, f"log{latest_log}.bin")