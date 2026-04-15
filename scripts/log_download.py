from pymavlink import mavutil
import time
import os
import pickle

# ================= CONFIG =================
CONNECTION = '/dev/ttyUSB0'
BAUD = 115200

CHUNK_SIZE = 900
WINDOW_SIZE = 10
RETRY_TIMEOUT = 1.0

# ==========================================

def connect():
    print("Connecting...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    master.wait_heartbeat()
    print("Connected")
    return master


def get_log_list(master):
    print("Requesting log list...")
    master.mav.log_request_list_send(
        master.target_system,
        master.target_component,
        0,
        0xFFFF
    )

    logs = []
    while True:
        msg = master.recv_match(type='LOG_ENTRY', blocking=True)
        if not msg:
            continue

        logs.append(msg)
        print(f"Log {msg.id} size={msg.size}")

        if msg.id == msg.last_log_num:
            break

    return logs


# ---------- Resume ----------
def load_map(filename):
    if os.path.exists(filename + ".map"):
        with open(filename + ".map", "rb") as f:
            return pickle.load(f)
    return set()

def save_map(filename, offsets):
    with open(filename + ".map", "wb") as f:
        pickle.dump(offsets, f)


# ---------- Downloader ----------
def download_log(master, log_id, size, filename):
    print(f"\nDownloading log {log_id} → {filename} (size={size} bytes)")

    received = load_map(filename)
    requested = {}
    retries = 0

    # Preallocate file
    with open(filename, "ab") as f:
        f.truncate(size)

    f = open(filename, "r+b")

    total_bytes = sum(CHUNK_SIZE for _ in received)
    last_bytes = total_bytes
    last_time = time.time()

    # ---------- Send initial window ----------
    next_offset = 0
    for i in range(WINDOW_SIZE):
        offset = i * CHUNK_SIZE
        if offset not in received:
            master.mav.log_request_data_send(
                master.target_system,
                master.target_component,
                log_id,
                offset,
                CHUNK_SIZE
            )
            requested[offset] = time.time()
        next_offset = offset + CHUNK_SIZE

    # ---------- Main loop ----------
    while True:
        msg = master.recv_match(type='LOG_DATA', blocking=False)

        if msg:
            offset = msg.ofs

            if offset not in received:
                data = bytes(msg.data[:msg.count])
                f.seek(offset)
                f.write(data)

                received.add(offset)
                total_bytes += len(data)

            # mark as received
            if offset in requested:
                del requested[offset]

        # ---------- Send new requests ----------
        while len(requested) < WINDOW_SIZE and next_offset < size:
            if next_offset not in received:
                master.mav.log_request_data_send(
                    master.target_system,
                    master.target_component,
                    log_id,
                    next_offset,
                    CHUNK_SIZE
                )
                requested[next_offset] = time.time()

            next_offset += CHUNK_SIZE

        # ---------- Retry missing ----------
        now = time.time()
        for offset in list(requested.keys()):
            if now - requested[offset] > RETRY_TIMEOUT:
                master.mav.log_request_data_send(
                    master.target_system,
                    master.target_component,
                    log_id,
                    offset,
                    CHUNK_SIZE
                )
                requested[offset] = now
                retries += 1

        # ---------- Progress ----------
        if now - last_time >= 1:
            speed = (total_bytes - last_bytes) / (now - last_time) / 1024
            percent = (total_bytes / size) * 100

            print(
                f"\r{percent:5.1f}% | {speed:6.1f} KB/s | {total_bytes}/{size} bytes | retries={retries}",
                end=""
            )

            last_time = now
            last_bytes = total_bytes

            save_map(filename, received)

        # ---------- Exit ----------
        if total_bytes >= size:
            break

    f.close()
    print("\nDownload complete")


# ---------- MAIN ----------
def main():
    master = connect()
    logs = get_log_list(master)

    if not logs:
        print("No logs found")
        return

    latest = logs[-1]
    download_log(master, latest.id, latest.size, f"log{latest.id}.bin")


if __name__ == "__main__":
    main()