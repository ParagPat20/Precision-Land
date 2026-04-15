from pymavlink import mavutil
import time
import os

# ================= CONFIG =================
# **Flight-controller serial link**
CONNECTION = "/dev/ttyUSB0"   # change if needed
BAUD = 115200

# **DataFlash LOG_DATA protocol facts**
# LOG_DATA payload is up to ~90 bytes per MAVLink message.
LOG_DATA_PAYLOAD = 90

# **Request tuning**
# Each request asks for COUNT bytes starting at OFFSET.
# Bigger COUNT reduces request overhead but increases time-to-retry on loss.
CHUNK_SIZE = LOG_DATA_PAYLOAD * 10   # 900 bytes per request
WINDOW_SIZE = 10                     # how many chunks we keep "in flight"

# **Retry tuning**
IDLE_RESEND_SEC = 0.8                # if no packets arrive, resend missing
RESEND_MAX_PER_CYCLE = 20            # cap resends per idle cycle

# **Progress printing**
PRINT_EVERY_SEC = 2.0
# ==========================================


def connect():
    print("Connecting...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    master.wait_heartbeat(timeout=5)
    print(f"Connected (sys={master.target_system} comp={master.target_component})")
    return master


def get_log_list(master):
    # Request the log directory listing (LOG_ENTRY stream)
    print("Requesting log list...")
    master.mav.log_request_list_send(
        master.target_system,
        master.target_component,
        0,
        0xFFFF,
    )

    logs = {}
    start = time.time()

    # Collect until we see last_log_num (or timeout)
    last_log_num = None
    while time.time() - start < 8:
        msg = master.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
        if not msg:
            continue
        logs[int(msg.id)] = msg
        last_log_num = int(msg.last_log_num)
        print(f"Log {int(msg.id)} size={int(msg.size)}")
        if last_log_num is not None and len(logs) >= int(msg.num_logs):
            break

    return [logs[k] for k in sorted(logs.keys())]


def _send_chunk_request(master, log_id, offset, count):
    master.mav.log_request_data_send(
        master.target_system,
        master.target_component,
        int(log_id),
        int(offset),
        int(count),
    )


def download_log(master, log_id, size, filename):
    print(f"\nDownloading log {log_id} → {filename}  (size={size} bytes)")

    # Track which chunk-start offsets we have requested / received
    requested = set()
    received = {}  # offset -> bytes (may be < CHUNK_SIZE at end)

    # Track write progress using actual file size on disk
    start_time = time.time()
    last_print = start_time
    last_recv = start_time
    retries = 0

    # Offsets are byte offsets into the log file
    next_request_offset = 0

    # Create/overwrite output file
    with open(filename, "wb") as f:
        f.truncate(0)

        # --- Initial pipeline fill (WINDOW_SIZE requests) ---
        for _ in range(WINDOW_SIZE):
            if next_request_offset >= size:
                break
            _send_chunk_request(master, log_id, next_request_offset, CHUNK_SIZE)
            requested.add(next_request_offset)
            next_request_offset += CHUNK_SIZE

        while True:
            # --- Non-blocking receive: drain whatever is available quickly ---
            msg = master.recv_match(type="LOG_DATA", blocking=False)

            if msg:
                last_recv = time.time()
                ofs = int(msg.ofs)
                cnt = int(msg.count)

                # cnt==0 can be used by FC as end-of-stream marker; we still rely on size
                if cnt > 0 and ofs not in received:
                    received[ofs] = bytes(msg.data[:cnt])

                # Keep the pipeline full by issuing next request(s)
                while len(requested) < (WINDOW_SIZE + len(received)) and next_request_offset < size:
                    _send_chunk_request(master, log_id, next_request_offset, CHUNK_SIZE)
                    requested.add(next_request_offset)
                    next_request_offset += CHUNK_SIZE

            # --- Write any received chunks (order doesn’t matter because we seek) ---
            if received:
                for ofs, data in list(received.items()):
                    f.seek(ofs)
                    f.write(data)
                    del received[ofs]
                f.flush()

            # --- If link is idle, resend missing chunks (like MAVProxy log.py) ---
            now = time.time()
            if now - last_recv > IDLE_RESEND_SEC:
                # Determine missing chunk offsets up to the highest requested chunk
                if requested:
                    highest = max(requested)
                    expected_offsets = set(range(0, min(highest + CHUNK_SIZE, size), CHUNK_SIZE))
                    missing = sorted(expected_offsets - requested)  # usually empty
                    # More importantly: chunks requested but not yet written are “missing”
                    # We consider a chunk “done” if the file already has data at that offset.
                    # (Cheap heuristic: if we ever received it, it would have been written.)
                    # So we resend the earliest requested offsets again.
                    resend_candidates = sorted(list(requested))[:RESEND_MAX_PER_CYCLE]

                    for ofs in resend_candidates:
                        _send_chunk_request(master, log_id, ofs, CHUNK_SIZE)

                    retries += 1
                last_recv = now

            # --- Progress print (based on actual bytes written) ---
            if now - last_print >= PRINT_EVERY_SEC:
                done = os.path.getsize(filename) if os.path.exists(filename) else 0
                done = min(done, size) if size > 0 else done
                dt = max(now - start_time, 0.001)
                speed_kbs = (done / 1024.0) / dt
                pct = (done / size * 100.0) if size > 0 else 0.0
                print(f"{pct:5.1f}% | {speed_kbs:6.1f} KB/s | {done}/{size} bytes | retries={retries}")
                last_print = now

            # --- Exit condition: written size reached expected size ---
            if size > 0:
                done = os.path.getsize(filename) if os.path.exists(filename) else 0
                if done >= size:
                    break

    dt = max(time.time() - start_time, 0.001)
    final_size = os.path.getsize(filename)
    final_speed_kbs = (final_size / 1024.0) / dt
    print(f"Download complete: {filename} ({final_size} bytes in {dt:.1f}s, {final_speed_kbs:.1f} KB/s, retries={retries})")

    # Always stop the FC’s log stream cleanly
    master.mav.log_request_end_send(master.target_system, master.target_component)


def main():
    master = connect()

    logs = get_log_list(master)
    if not logs:
        print("No logs found")
        return

    latest = logs[-1]
    download_log(master, int(latest.id), int(latest.size), f"log{int(latest.id)}.bin")


if __name__ == "__main__":
    main()