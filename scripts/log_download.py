from dataclasses import dataclass
import os
import time

from pymavlink import mavutil

# ================= CONFIG =================
CONNECTION = "/dev/ttyUSB0"
BAUD = 115200
LOG_ID = "latest"  # Use "latest" or an integer log id from the printed list.
OUTPUT_FILE = None  # Use None for QGC-style log_<id>_<time>.bin naming.
PAUSE_TELEMETRY_DURING_DOWNLOAD = True
# ==========================================


LOG_DATA_LEN = 90
QGC_CHUNK_BINS = 512
QGC_CHUNK_SIZE = LOG_DATA_LEN * QGC_CHUNK_BINS
QGC_DATA_TIMEOUT_SEC = 0.5
QGC_LIST_TIMEOUT_SEC = 5.0
QGC_LIST_RETRIES = 2


@dataclass
class LogEntry:
    qgc_id: int
    mav_id: int
    size: int
    time_utc: int
    num_logs: int
    last_log_num: int


def connect():
    print("Connecting...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    master.wait_heartbeat()
    print(
        f"Connected: system={master.target_system} component={master.target_component}"
    )
    return master


def is_ardupilot(master):
    heartbeat = master.messages.get("HEARTBEAT")
    return (
        getattr(heartbeat, "autopilot", None)
        == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
    )


def first_missing_log_range(entries, expected):
    if expected <= 0:
        return None
    start = -1
    end = -1
    for idx in range(expected):
        if idx not in entries:
            if start < 0:
                start = idx
            else:
                end = idx
        elif start >= 0:
            break
    if start < 0:
        return None
    if end < 0:
        end = start
    return start, end


def get_log_list(master):
    """
    QGC-style LOG_REQUEST_LIST:
    request all entries, then retry the first missing contiguous range.
    """
    apm_offset = 1 if is_ardupilot(master) else 0
    entries = {}
    expected = 0
    retries = 0
    start = 0
    end = 0xFFFF

    print("Requesting log list...")
    while retries <= QGC_LIST_RETRIES:
        master.mav.log_request_list_send(
            master.target_system,
            master.target_component,
            start,
            end,
        )
        deadline = time.time() + QGC_LIST_TIMEOUT_SEC

        while time.time() < deadline:
            msg = master.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.1)
            if msg is None:
                continue
            if int(msg.num_logs) <= 0:
                return []

            expected = max(expected, int(msg.num_logs))
            mav_id = int(msg.id)
            size = int(msg.size)
            qgc_id = mav_id - apm_offset

            # QGC ignores ArduPilot's first count-only entry with zero size.
            if qgc_id < 0 or (apm_offset and size <= 0):
                continue

            entries[qgc_id] = LogEntry(
                qgc_id=qgc_id,
                mav_id=mav_id,
                size=size,
                time_utc=int(msg.time_utc),
                num_logs=int(msg.num_logs),
                last_log_num=int(msg.last_log_num),
            )

            if expected > 0 and len(entries) >= expected:
                return [entries[idx] for idx in sorted(entries)]

        missing = first_missing_log_range(entries, expected)
        if missing is None:
            break
        start, end = missing
        start += apm_offset
        end += apm_offset
        retries += 1

    return [entries[idx] for idx in sorted(entries)]


def print_log_list(logs):
    for entry in logs:
        if entry.time_utc > 0:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(entry.time_utc))
        else:
            timestamp = "UnknownDate"
        print(
            f"Log {entry.qgc_id} mav_id={entry.mav_id} "
            f"size={entry.size} time={timestamp}"
        )


class QGCLogDownload:
    def __init__(self, master, entry, filename):
        self.master = master
        self.entry = entry
        self.filename = filename
        self.current_chunk = 0
        self.chunk_table = [False] * self.chunk_bins(0)
        self.written_offsets = set()
        self.bytes_written = 0
        self.rate_bytes = 0
        self.rate_avg = 0.0
        self.last_rate_time = time.time()
        self.rate_packets = 0
        self.retries = 0

    def set_telemetry_streams(self, rate_hz):
        for stream_id in (
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ):
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                rate_hz,
                1 if rate_hz > 0 else 0,
            )

    def num_chunks(self):
        return max(1, (self.entry.size + QGC_CHUNK_SIZE - 1) // QGC_CHUNK_SIZE)

    def chunk_bins(self, chunk):
        remaining = max(0, self.entry.size - chunk * QGC_CHUNK_SIZE)
        bins = (remaining + LOG_DATA_LEN - 1) // LOG_DATA_LEN
        return max(1, min(QGC_CHUNK_BINS, bins))

    def chunk_complete(self):
        return bool(self.chunk_table) and all(self.chunk_table)

    def log_complete(self):
        return self.chunk_complete() and self.current_chunk + 1 == self.num_chunks()

    def advance_chunk(self):
        self.current_chunk += 1
        self.chunk_table = [False] * self.chunk_bins(self.current_chunk)

    def request_missing_range(self):
        if self.chunk_complete():
            return

        start = 0
        while start < len(self.chunk_table) and self.chunk_table[start]:
            start += 1

        end = start
        while end < len(self.chunk_table) and not self.chunk_table[end]:
            end += 1

        offset = self.current_chunk * QGC_CHUNK_SIZE + start * LOG_DATA_LEN
        count = (end - start) * LOG_DATA_LEN
        self.master.mav.log_request_data_send(
            self.master.target_system,
            self.master.target_component,
            self.entry.mav_id,
            offset,
            count,
        )
        self.retries += 1

    def handle_log_data(self, msg, file_obj):
        if int(msg.id) != self.entry.mav_id:
            return

        ofs = int(msg.ofs)
        count = int(msg.count)
        if ofs % LOG_DATA_LEN != 0:
            print(f"\nIgnored misaligned LOG_DATA offset {ofs}")
            return
        if ofs > self.entry.size:
            print(f"\nIgnored LOG_DATA offset past EOF {ofs}")
            return

        chunk = ofs // QGC_CHUNK_SIZE
        if chunk != self.current_chunk:
            return

        bin_index = (ofs - chunk * QGC_CHUNK_SIZE) // LOG_DATA_LEN
        if bin_index >= len(self.chunk_table):
            print(f"\nIgnored out-of-range LOG_DATA bin {bin_index}")
            return

        payload = bytes(msg.data[:count])
        file_obj.seek(ofs)
        file_obj.write(payload)

        if ofs not in self.written_offsets:
            self.written_offsets.add(ofs)
            self.bytes_written += count
            self.rate_bytes += count
            self.rate_packets += 1

        self.chunk_table[bin_index] = True

        if self.log_complete():
            return
        if self.chunk_complete():
            self.advance_chunk()
            self.request_missing_range()
        elif bin_index < len(self.chunk_table) - 1 and self.chunk_table[bin_index + 1]:
            self.request_missing_range()

    def print_progress(self, started):
        now = time.time()
        if now - self.last_rate_time < 1.0:
            return

        dt = max(now - self.last_rate_time, 0.000001)
        rate = self.rate_bytes / dt
        packet_rate = self.rate_packets / dt
        self.rate_avg = self.rate_avg * 0.95 + rate * 0.05
        total_dt = max(now - started, 0.000001)
        total_rate = self.bytes_written / total_dt
        percent = 100.0 * min(self.bytes_written, self.entry.size) / self.entry.size
        missing = self.chunk_table.count(False)

        print(
            f"\r{percent:5.1f}% | "
            f"now {rate / 1024:6.1f} KB/s | "
            f"avg {total_rate / 1024:6.1f} KB/s | "
            f"{packet_rate:5.1f} pkt/s | "
            f"{min(self.bytes_written, self.entry.size)}/{self.entry.size} bytes | "
            f"missing={missing} retries={self.retries}",
            end="",
        )
        self.rate_bytes = 0
        self.rate_packets = 0
        self.last_rate_time = now

    def download(self):
        print(
            f"\nDownloading log {self.entry.qgc_id} "
            f"(mav_id={self.entry.mav_id}) -> {self.filename}"
        )
        started = time.time()
        try:
            if PAUSE_TELEMETRY_DURING_DOWNLOAD:
                self.set_telemetry_streams(0)
            with open(self.filename, "wb") as file_obj:
                file_obj.truncate(self.entry.size)
                self.request_missing_range()
                next_retry = time.time() + QGC_DATA_TIMEOUT_SEC

                while not self.log_complete():
                    msg = self.master.recv_match(
                        type="LOG_DATA", blocking=True, timeout=0.05
                    )
                    if msg is not None:
                        self.handle_log_data(msg, file_obj)
                        next_retry = time.time() + QGC_DATA_TIMEOUT_SEC
                    elif time.time() >= next_retry:
                        self.request_missing_range()
                        next_retry = time.time() + QGC_DATA_TIMEOUT_SEC

                    self.print_progress(started)
        except Exception:
            self.master.mav.log_request_end_send(
                self.master.target_system,
                self.master.target_component,
            )
            if PAUSE_TELEMETRY_DURING_DOWNLOAD:
                self.set_telemetry_streams(4)
            if os.path.exists(self.filename):
                os.remove(self.filename)
            raise

        self.master.mav.log_request_end_send(
            self.master.target_system,
            self.master.target_component,
        )
        if PAUSE_TELEMETRY_DURING_DOWNLOAD:
            self.set_telemetry_streams(4)
        elapsed = max(time.time() - started, 0.000001)
        print(
            f"\nDownload complete: {self.entry.size} bytes in {elapsed:.1f}s "
            f"({self.entry.size / elapsed / 1024:.1f} KB/s)"
        )


def default_filename(entry):
    if entry.time_utc > 0:
        timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(entry.time_utc))
    else:
        timestamp = "UnknownDate"
    return f"log_{entry.qgc_id}_{timestamp}.bin"


def select_log(logs):
    if str(LOG_ID).lower() == "latest":
        return max(logs, key=lambda item: item.qgc_id)
    wanted = int(LOG_ID)
    for entry in logs:
        if entry.qgc_id == wanted or entry.mav_id == wanted:
            return entry
    raise RuntimeError(f"Log {wanted} was not reported by the flight controller")


def main():
    master = connect()
    logs = get_log_list(master)
    if not logs:
        print("No logs found")
        return

    print_log_list(logs)
    entry = select_log(logs)
    filename = OUTPUT_FILE or default_filename(entry)
    QGCLogDownload(master, entry, filename).download()


if __name__ == "__main__":
    main()
