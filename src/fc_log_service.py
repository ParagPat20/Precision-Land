import json
import os
import queue
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse
import mimetypes
import re


class FlightControllerLogService:
    CHUNK_SIZE = 90
    AUTOPILOT_COMPONENT_ID = 1
    FTP_MAX_DATA = 239
    FTP_HEADER_FORMAT = "<HBBBBBBI"
    FTP_HEADER_SIZE = struct.calcsize(FTP_HEADER_FORMAT)
    MAVFTP_LOG_DIRS = ("@SYS/logs", "/APM/LOGS", "/APM/LOGS/")
    FTP_OPCODE_NONE = 0
    FTP_OPCODE_TERMINATE_SESSION = 1
    FTP_OPCODE_RESET_SESSIONS = 2
    FTP_OPCODE_LIST_DIRECTORY = 3
    FTP_OPCODE_OPEN_FILE_RO = 4
    FTP_OPCODE_READ_FILE = 5
    FTP_OPCODE_BURST_READ_FILE = 15
    FTP_OPCODE_ACK = 128
    FTP_OPCODE_NAK = 129
    FTP_ERROR_NONE = 0
    FTP_ERROR_FAIL = 1
    FTP_ERROR_EOF = 6
    FTP_ERROR_FILE_NOT_FOUND = 10

    # Burst read – smaller payload on UART so the FC serial MAVLink parser is not overrun
    BURST_READ_SIZE = 180
    BURST_RETRY_TIMEOUT = 1.0
    BURST_IDLE_TIMEOUT = 5.0
    BURST_GAP_COUNT_FALLBACK = 6
    BURST_EMPTY_STREAK_FALLBACK = 10

    # Pipelined ReadFile (MAVFTP): tuned for UART (921600-class links, CUAV + RPi)
    FTP_PIPELINE_WINDOW_SIZE = 4
    FTP_PIPELINE_WINDOW_MIN = 4
    FTP_PIPELINE_WINDOW_MAX = 6
    FTP_PIPELINE_CHUNK = 239
    FTP_PIPELINE_GAP_RETRY = 2.0
    FTP_PIPELINE_GAP_RETRY_MAX = 32.0
    FTP_PIPELINE_RECV_TIMEOUT = 0.2
    FTP_PIPELINE_READ_ISSUE_PACE_SEC = 0.003
    FTP_PIPELINE_GLOBAL_TIMEOUT = 900.0
    FTP_PIPELINE_STALL_NUDGE_SEC = 2.0
    FTP_PIPELINE_STALL_ABORT_SEC = 8.0
    FTP_PIPELINE_MAX_RETRIES_PER_OFFSET = 48
    BURST_LOG_INTERVAL_SEC = 2.0
    BURST_RESEND_BACKOFF_CAP = 2.5

    # ListDirectory / OpenFileRO (MAVFTP control plane): pymavlink ``mavftp.process_ftp_reply``
    # defaults to ~5s wall-clock with 0.1s recv slices — much looser than streaming ReadFile.
    # Opcode 3 (list) on UART often loses to LOG/HEARTBEAT traffic; 0.2s x 6 was too aggressive.
    FTP_CONTROL_RECV_TIMEOUT_SEC = float(os.environ.get("JECH_FC_MAVFTP_CONTROL_TIMEOUT", "1.25"))
    FTP_CONTROL_MAX_RETRIES = int(os.environ.get("JECH_FC_MAVFTP_CONTROL_RETRIES", "12"))

    def __init__(self, vehicle, cache_dir, web_root, host="0.0.0.0", port=8765):
        self.vehicle = vehicle
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.web_root = Path(web_root)
        self.host = host
        self.port = port
        self.enable_mavftp = os.environ.get("JECH_FC_ENABLE_MAVFTP", "1") == "1"

        self._download_lock = threading.RLock()
        self._list_lock = threading.Lock()
        self._list_entries = {}
        self._list_event = threading.Event()
        self._chunk_waiters = {}
        self._chunk_lock = threading.Lock()
        self._ftp_lock = threading.Lock()
        self._ftp_message_queue = queue.Queue()
        self._ftp_seq = 0
        self._http_thread = None
        self._auto_download_in_progress = False
        # Exposed via GET /api/mavftp-status for Log Finder (single FC link: no parallel MAVFTP)
        self._client_activity_lock = threading.Lock()
        self._flight_log_download_active = False
        self._flight_log_download_started_mono = 0.0
        self._flight_log_download_label = ""
        self._mavftp_wire_active = False
        self._auto_download_status_msg = ""

        self.vehicle.add_message_listener("LOG_ENTRY", self._on_log_entry)
        self.vehicle.add_message_listener("LOG_DATA", self._on_log_data)
        self.vehicle.add_message_listener("FILE_TRANSFER_PROTOCOL", self._on_file_transfer_protocol)

    def _set_auto_download_msg(self, msg: str) -> None:
        with self._client_activity_lock:
            self._auto_download_status_msg = msg

    def _flight_log_download_enter(self, label: str) -> None:
        with self._client_activity_lock:
            self._flight_log_download_active = True
            self._flight_log_download_label = label
            self._flight_log_download_started_mono = time.monotonic()

    def _flight_log_download_leave(self) -> None:
        with self._client_activity_lock:
            self._flight_log_download_active = False
            self._flight_log_download_label = ""
            self._flight_log_download_started_mono = 0.0

    def _mavftp_wire_enter(self) -> None:
        with self._client_activity_lock:
            self._mavftp_wire_active = True

    def _mavftp_wire_leave(self) -> None:
        with self._client_activity_lock:
            self._mavftp_wire_active = False

    def mavftp_public_status(self):
        """Snapshot for Log Finder: one FC link, so MAVFTP and log saves are serialized."""
        with self._client_activity_lock:
            auto_wait = self._auto_download_in_progress and not self._flight_log_download_active
            busy = self._flight_log_download_active or auto_wait
            elapsed = 0.0
            if self._flight_log_download_active and self._flight_log_download_started_mono > 0:
                elapsed = time.monotonic() - self._flight_log_download_started_mono
            bits = []
            if auto_wait:
                bits.append(self._auto_download_status_msg or "Auto log download (waiting)")
            if self._mavftp_wire_active:
                bits.append("MAVFTP active on FC link (do not start another download)")
            if self._flight_log_download_active and not self._mavftp_wire_active:
                bits.append(self._flight_log_download_label or "Flight log transfer")
            return {
                "busy": busy,
                "mavftp_active": self._mavftp_wire_active,
                "detail": " — ".join(bits) if bits else "",
                "elapsed_sec": round(elapsed, 1) if busy else 0.0,
                "auto_download_pending": auto_wait,
            }

    def _clear_ftp_message_queue(self):
        while True:
            try:
                self._ftp_message_queue.get_nowait()
            except queue.Empty:
                return

    def start(self):
        if self._http_thread is not None:
            return

        self._http_thread = threading.Thread(target=self._serve_http, name="FCLogHTTPServer", daemon=True)
        self._http_thread.start()
        print(f"[LOG SERVICE] HTTP server listening on http://{self.host}:{self.port}")

    def _serve_http(self):
        service = self

        class Handler(BaseHTTPRequestHandler):
            def do_OPTIONS(self):
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    self.send_response(302)
                    self.send_header("Location", "/webtools/LogFinder/index.html?source=/api/logs&inventory=/api/fc-logs")
                    self.end_headers()
                    return

                if parsed.path == "/api/logs":
                    self._send_json({"logs": service.list_cached_logs()})
                    return

                if parsed.path == "/api/fc-logs":
                    try:
                        self._send_json({"logs": service.list_flight_controller_logs()})
                    except Exception as error:
                        self._send_json({"error": str(error)}, status=500)
                    return

                if parsed.path == "/api/mavftp-status":
                    self._send_json(service.mavftp_public_status())
                    return

                if parsed.path == "/api/logs/latest":
                    try:
                        metadata = service.download_latest_log()
                        self._send_json(metadata)
                    except Exception as error:
                        self._send_json({"error": str(error)}, status=500)
                    return

                if parsed.path.startswith("/api/fc-logs/download/"):
                    try:
                        log_id = int(parsed.path.rsplit("/", 1)[-1])
                        metadata = service.download_log_by_id(log_id)
                        self._send_json(metadata)
                    except Exception as error:
                        self._send_json({"error": str(error)}, status=500)
                    return

                if parsed.path.startswith("/logs/"):
                    relative_name = unquote(parsed.path[len("/logs/"):])
                    try:
                        file_path = service.resolve_log_path(relative_name)
                    except FileNotFoundError:
                        self._send_json({"error": "Log not found"}, status=404)
                        return

                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(file_path.stat().st_size))
                    self.send_header("Content-Disposition", f'inline; filename="{file_path.name}"')
                    self.end_headers()
                    with file_path.open("rb") as file_obj:
                        self.wfile.write(file_obj.read())
                    return

                if parsed.path.startswith("/webtools/"):
                    try:
                        asset_path = service.resolve_web_asset(parsed.path[len("/webtools/"):])
                    except FileNotFoundError:
                        self._send_json({"error": "Asset not found"}, status=404)
                        return

                    content_type, _ = mimetypes.guess_type(str(asset_path))
                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", content_type or "application/octet-stream")
                    self.send_header("Content-Length", str(asset_path.stat().st_size))
                    self.end_headers()
                    with asset_path.open("rb") as file_obj:
                        self.wfile.write(file_obj.read())
                    return

                self._send_json({"error": "Not found"}, status=404)

            def log_message(self, fmt, *args):
                print(f"[LOG SERVICE] {self.address_string()} - {fmt % args}")

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def _send_json(self, payload, status=200):
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self._cors()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        httpd.serve_forever()

    def _on_log_entry(self, vehicle, name, message):
        entry = {
            "id": int(message.id),
            "num_logs": int(message.num_logs),
            "last_log_num": int(message.last_log_num),
            "size": int(message.size),
            "time_utc": int(message.time_utc),
        }

        with self._list_lock:
            self._list_entries[entry["id"]] = entry
            expected = max((item["num_logs"] for item in self._list_entries.values()), default=0)
            if expected > 0 and len(self._list_entries) >= expected:
                self._list_event.set()

    def _on_log_data(self, vehicle, name, message):
        key = (int(message.id), int(message.ofs))
        payload = bytes(message.data[:int(message.count)])

        with self._chunk_lock:
            waiter = self._chunk_waiters.get(key)
            if waiter is None:
                return
            waiter["data"] = payload
            waiter["event"].set()

    def _on_file_transfer_protocol(self, vehicle, name, message):
        try:
            self._ftp_message_queue.put_nowait(message)
        except queue.Full:
            pass

    def _send_log_request_list(self):
        master = self.vehicle._master
        master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xFFFF)

    def _send_log_request_data(self, log_id, offset, count):
        master = self.vehicle._master
        master.mav.log_request_data_send(master.target_system, master.target_component, log_id, offset, count)

    def _send_log_request_end(self):
        master = self.vehicle._master
        master.mav.log_request_end_send(master.target_system, master.target_component)

    def _next_ftp_seq(self):
        seq = self._ftp_seq
        self._ftp_seq = (self._ftp_seq + 1) % 65536
        return seq

    def _pack_ftp_payload(self, seq, session, opcode, size=0, req_opcode=0, burst_complete=0, offset=0, data=b""):
        data = bytes(data)
        if len(data) > self.FTP_MAX_DATA:
            raise ValueError(f"FTP payload too large: {len(data)} bytes")
        header = struct.pack(
            self.FTP_HEADER_FORMAT,
            seq,
            session,
            opcode,
            size,
            req_opcode,
            burst_complete,
            0,
            offset,
        )
        payload = header + data
        return payload.ljust(self.FTP_HEADER_SIZE + self.FTP_MAX_DATA, b"\x00")

    def _parse_ftp_message(self, message):
        payload = bytes(message.payload)
        seq, session, opcode, size, req_opcode, burst_complete, _padding, offset = struct.unpack(
            self.FTP_HEADER_FORMAT,
            payload[:self.FTP_HEADER_SIZE],
        )
        data = payload[self.FTP_HEADER_SIZE:self.FTP_HEADER_SIZE + size]
        return {
            "seq": seq,
            "session": session,
            "opcode": opcode,
            "size": size,
            "req_opcode": req_opcode,
            "burst_complete": burst_complete,
            "offset": offset,
            "data": data,
            "target_system": getattr(message, "target_system", None),
            "target_component": getattr(message, "target_component", None),
        }

    def _ftp_targets_acceptable(self, parsed, master):
        """Accept FTP replies addressed to broadcast (0) or this link's MAVLink system id."""
        ts = parsed.get("target_system")
        if ts is None:
            return True
        return ts == 0 or ts == master.source_system

    def _ftp_component_acceptable(self, parsed, master):
        """FC replies name our component; accept 0 (wildcard) and common GCS-style ids."""
        tc = parsed.get("target_component")
        if tc is None or tc == 0:
            return True
        ours = getattr(master, "source_component", None)
        if ours is not None and tc == ours:
            return True
        # Mission Planner / QGC / companion defaults ArduPilot sometimes uses when addressing the link
        if tc in (190, 195, 240, 250):
            return True
        return False

    def _ftp_link_prefers_large_window(self, master):
        """UDP transports can safely run a wider ReadFile pipeline than UART."""
        port = str(getattr(master, "port", "") or getattr(master, "name", "") or "").lower()
        return "udp" in port or "udpin" in port or "udpout" in port

    def _send_ftp_command(self, seq, session, opcode, size=0, offset=0, data=b""):
        master = self.vehicle._master
        payload = self._pack_ftp_payload(
            seq=seq,
            session=session,
            opcode=opcode,
            size=size,
            req_opcode=0,
            burst_complete=0,
            offset=offset,
            data=data,
        )
        master.mav.file_transfer_protocol_send(
            0,
            master.target_system,
            self.AUTOPILOT_COMPONENT_ID,
            payload,
        )

    def _wait_for_ftp_reply(self, seq, req_opcode, timeout):
        deadline = time.time() + timeout
        master = self.vehicle._master
        while True:
            remaining = max(0, deadline - time.time())
            if remaining == 0:
                return None
            try:
                message = self._ftp_message_queue.get(timeout=remaining)
            except queue.Empty:
                return None

            parsed = self._parse_ftp_message(message)
            if not self._ftp_targets_acceptable(parsed, master):
                continue
            if not self._ftp_component_acceptable(parsed, master):
                continue
            if parsed["seq"] != seq or parsed["req_opcode"] != req_opcode:
                continue
            return parsed

    def _ftp_request(self, opcode, session=0, offset=0, data=b"", size=None, timeout=0.05, retries=6):
        payload_size = len(data) if size is None else size
        seq = self._next_ftp_seq()
        for _ in range(retries):
            self._send_ftp_command(seq, session, opcode, size=payload_size, offset=offset, data=data)
            reply = self._wait_for_ftp_reply(seq, opcode, timeout)
            if reply is not None:
                return reply
        raise RuntimeError(f"FTP opcode {opcode} timed out after {retries} retries")

    def _ftp_reset_sessions(self):
        try:
            self._ftp_request(
                self.FTP_OPCODE_RESET_SESSIONS,
                timeout=self.FTP_CONTROL_RECV_TIMEOUT_SEC,
                retries=max(2, self.FTP_CONTROL_MAX_RETRIES // 4),
            )
        except Exception:
            return

    def _ftp_list_directory(self, directory):
        entries = []
        offset = 0
        encoded_dir = directory.encode("utf-8")

        while True:
            reply = self._ftp_request(
                self.FTP_OPCODE_LIST_DIRECTORY,
                offset=offset,
                data=encoded_dir,
                timeout=self.FTP_CONTROL_RECV_TIMEOUT_SEC,
                retries=self.FTP_CONTROL_MAX_RETRIES,
            )
            if reply["opcode"] == self.FTP_OPCODE_NAK:
                error_code = reply["data"][0] if reply["data"] else self.FTP_ERROR_FAIL
                if error_code == self.FTP_ERROR_EOF:
                    return entries
                if error_code == self.FTP_ERROR_FILE_NOT_FOUND:
                    return []
                raise RuntimeError(f"FTP list for {directory} failed with error {error_code}")

            decoded_entries = [item for item in reply["data"].split(b"\x00") if item]
            for raw_entry in decoded_entries:
                text = raw_entry.decode("utf-8", errors="ignore")
                if not text:
                    continue
                entries.append(text)
            offset += len(decoded_entries)

    def _ftp_open_file_ro(self, remote_path):
        reply = self._ftp_request(
            self.FTP_OPCODE_OPEN_FILE_RO,
            data=remote_path.encode("utf-8"),
            timeout=self.FTP_CONTROL_RECV_TIMEOUT_SEC,
            retries=self.FTP_CONTROL_MAX_RETRIES,
        )
        if reply["opcode"] == self.FTP_OPCODE_NAK:
            error_code = reply["data"][0] if reply["data"] else self.FTP_ERROR_FAIL
            raise RuntimeError(f"FTP open failed for {remote_path} with error {error_code}")
        file_size = struct.unpack("<I", reply["data"][:4].ljust(4, b"\x00"))[0]
        return reply["session"], file_size

    def _ftp_read_file(self, session, file_size, file_path):
        chunk_size = self.FTP_MAX_DATA
        offset = 0
        with file_path.open("wb") as output:
            while offset < file_size:
                requested = min(chunk_size, file_size - offset)
                reply = self._ftp_request(
                    self.FTP_OPCODE_READ_FILE,
                    session=session,
                    offset=offset,
                    size=requested,
                    timeout=0.05,
                    retries=6,
                )
                if reply["opcode"] == self.FTP_OPCODE_NAK:
                    error_code = reply["data"][0] if reply["data"] else self.FTP_ERROR_FAIL
                    if error_code == self.FTP_ERROR_EOF:
                        break
                    raise RuntimeError(f"FTP read failed at offset {offset} with error {error_code}")

                data = reply["data"]
                if not data:
                    raise RuntimeError(f"FTP read returned empty data at offset {offset}")
                output.write(data)
                offset += len(data)

    def _ftp_pipelined_read_file(self, session, file_size, file_path):
        """
        Pipelined MAVFTP ReadFile for UART: small window, paced issues, exponential gap backoff.

        Packet loss: missing ``buffer_map`` entries wait for ``FTP_PIPELINE_GAP_RETRY`` backoff
        (exponential in ``n``) before a single offset is re-requested—no immediate resend spam.
        Stalls: no contiguous progress for ``FTP_PIPELINE_STALL_NUDGE_SEC`` re-issues pending
        chunks once (``stall_nudge``); if still no progress for ``FTP_PIPELINE_STALL_ABORT_SEC``,
        raise so the outer caller can fall back to sequential ReadFile.
        """
        chunk = self.FTP_PIPELINE_CHUNK
        master = self.vehicle._master
        win = self.FTP_PIPELINE_WINDOW_SIZE
        if self._ftp_link_prefers_large_window(master):
            win = min(self.FTP_PIPELINE_WINDOW_MAX, win + 2)

        if file_size <= 0:
            file_path.write_bytes(b"")
            return

        total_chunks = (file_size + chunk - 1) // chunk

        def chunk_start(idx):
            return idx * chunk

        def chunk_byte_size(idx):
            off = chunk_start(idx)
            return min(chunk, file_size - off)

        buffer_map = {}
        pending = {}
        next_chunk_idx = 0
        start_time = time.time()
        last_log = start_time
        last_progress_mono = time.monotonic()
        bytes_flushed = 0
        last_log_bytes = 0
        gap_retries_period = 0
        had_gap_retry_this_cycle = False
        stall_start_mono = None
        last_stall_nudge_mono = 0.0

        def issue_read_for_chunk(idx, bump_retry=False, stall_nudge=False):
            off = chunk_start(idx)
            sz = chunk_byte_size(idx)
            seq = self._next_ftp_seq()
            self._send_ftp_command(
                seq, session, self.FTP_OPCODE_READ_FILE, size=sz, offset=off, data=b""
            )
            prev = pending.get(off)
            base = 0 if prev is None else prev["n"]
            if stall_nudge:
                n = base
            elif bump_retry:
                n = base + 1
            else:
                n = 0
            pending[off] = {"sent": time.monotonic(), "n": n}

        def fill_window():
            fill_cap = win
            if had_gap_retry_this_cycle:
                fill_cap = min(win, self.FTP_PIPELINE_WINDOW_MIN)
            while len(pending) < fill_cap and next_chunk_idx < total_chunks:
                issued = False
                for idx in range(next_chunk_idx, total_chunks):
                    off = chunk_start(idx)
                    if off in buffer_map or off in pending:
                        continue
                    issue_read_for_chunk(idx)
                    time.sleep(self.FTP_PIPELINE_READ_ISSUE_PACE_SEC)
                    issued = True
                    break
                if not issued:
                    break

        def flush_contiguous(output):
            nonlocal next_chunk_idx, last_progress_mono, bytes_flushed
            parts = []
            while next_chunk_idx < total_chunks:
                off = chunk_start(next_chunk_idx)
                if off not in buffer_map:
                    break
                parts.append(buffer_map.pop(off))
                pending.pop(off, None)
                next_chunk_idx += 1
            if parts:
                block = b"".join(parts)
                output.write(block)
                bytes_flushed += len(block)
                last_progress_mono = time.monotonic()

        def retry_stale_pending():
            nonlocal gap_retries_period, win, had_gap_retry_this_cycle
            now = time.monotonic()
            for off in list(pending.keys()):
                if off in buffer_map:
                    pending.pop(off, None)
                    continue
                meta = pending.get(off)
                if meta is None:
                    continue
                backoff = min(
                    self.FTP_PIPELINE_GAP_RETRY_MAX,
                    self.FTP_PIPELINE_GAP_RETRY * (2 ** min(meta["n"], 6)),
                )
                if now - meta["sent"] < backoff:
                    continue
                if meta["n"] >= self.FTP_PIPELINE_MAX_RETRIES_PER_OFFSET:
                    raise RuntimeError(
                        f"Pipelined ReadFile: offset {off} exceeded retry budget (loss or FC stall)"
                    )
                gap_retries_period += 1
                had_gap_retry_this_cycle = True
                idx = off // chunk
                issue_read_for_chunk(idx, bump_retry=True)
                time.sleep(self.FTP_PIPELINE_READ_ISSUE_PACE_SEC)

        with file_path.open("wb", buffering=512 * 1024) as output:
            fill_window()
            global_deadline = start_time + self.FTP_PIPELINE_GLOBAL_TIMEOUT

            while next_chunk_idx < total_chunks or pending:
                had_gap_retry_this_cycle = False
                if time.time() > global_deadline:
                    raise RuntimeError(
                        f"Pipelined MAVFTP timed out after {self.FTP_PIPELINE_GLOBAL_TIMEOUT}s "
                        f"({next_chunk_idx}/{total_chunks} chunks, {len(pending)} pending)"
                    )

                mono = time.monotonic()
                if pending and (mono - last_progress_mono) > self.FTP_PIPELINE_STALL_NUDGE_SEC:
                    if stall_start_mono is None:
                        stall_start_mono = mono
                    if (mono - stall_start_mono) > self.FTP_PIPELINE_STALL_ABORT_SEC:
                        raise RuntimeError(
                            f"Pipelined ReadFile stalled >{self.FTP_PIPELINE_STALL_ABORT_SEC}s "
                            f"(idx={next_chunk_idx}/{total_chunks}, pending={len(pending)})"
                        )
                    if (mono - last_stall_nudge_mono) >= self.FTP_PIPELINE_STALL_NUDGE_SEC:
                        last_stall_nudge_mono = mono
                        for off in list(pending.keys()):
                            if off in buffer_map:
                                continue
                            idx = off // chunk
                            issue_read_for_chunk(idx, stall_nudge=True)
                            time.sleep(self.FTP_PIPELINE_READ_ISSUE_PACE_SEC)
                else:
                    stall_start_mono = None

                retry_stale_pending()
                fill_window()

                try:
                    message = self._ftp_message_queue.get(timeout=self.FTP_PIPELINE_RECV_TIMEOUT)
                except queue.Empty:
                    continue

                parsed = self._parse_ftp_message(message)
                if not self._ftp_targets_acceptable(parsed, master):
                    continue
                if not self._ftp_component_acceptable(parsed, master):
                    continue
                if parsed["session"] != session:
                    continue
                if parsed["req_opcode"] != self.FTP_OPCODE_READ_FILE:
                    continue

                off = int(parsed["offset"])

                if off in buffer_map:
                    pending.pop(off, None)
                    continue

                if parsed["opcode"] == self.FTP_OPCODE_NAK:
                    continue

                if parsed["opcode"] != self.FTP_OPCODE_ACK:
                    continue

                data = parsed["data"]
                if not data:
                    continue

                first_missing = chunk_start(next_chunk_idx)
                if off < first_missing:
                    continue

                buffer_map[off] = data
                pending.pop(off, None)
                flush_contiguous(output)
                fill_window()

                now = time.time()
                if now - last_log >= 2.0:
                    done_bytes = min(bytes_flushed, file_size)
                    dt = max(now - last_log, 0.001)
                    inst_rate = ((done_bytes - last_log_bytes) / 1024.0) / dt
                    avg_rate = (done_bytes / 1024.0) / max(now - start_time, 0.001)
                    if gap_retries_period > win:
                        win = max(self.FTP_PIPELINE_WINDOW_MIN, win - 1)
                    elif gap_retries_period == 0 and win < self.FTP_PIPELINE_WINDOW_MAX:
                        win += 1
                    gap_retries_period = 0
                    print(
                        f"[LOG SERVICE] Pipelined ReadFile: {done_bytes}/{file_size} B "
                        f"~{inst_rate:.1f} KB/s (avg {avg_rate:.1f}) win={win} "
                        f"pend={len(pending)} buf={len(buffer_map)}"
                    )
                    last_log = now
                    last_log_bytes = done_bytes

            if buffer_map or pending:
                raise RuntimeError(
                    f"Pipelined MAVFTP finished with leftover state: "
                    f"buffered={len(buffer_map)} pending={len(pending)}"
                )

        elapsed = time.time() - start_time
        rate = (file_size / 1024.0) / max(elapsed, 0.001)
        print(
            f"[LOG SERVICE] Pipelined ReadFile complete: {file_size} bytes in {elapsed:.1f}s ({rate:.1f} KB/s)"
        )

    def _ftp_burst_read_file(self, session, file_size, file_path):
        """
        MAVFTP BurstReadFile (opcode 15): high throughput like QGC.

        Packet loss: gap tuples (start, length) record missing byte ranges; only those
        ranges are re-read via ReadFile in ``_fill_read_gaps``. Idle stall uses exponential
        backoff before re-issuing BurstReadFile so the FC is not spammed.
        """
        burst_size = self.BURST_READ_SIZE
        offset = 0
        gaps = []
        reached_eof = False
        read_total = 0
        start_time = time.time()
        last_activity = time.time()
        last_log = start_time
        empty_streak = 0
        master = self.vehicle._master

        with file_path.open("wb", buffering=512 * 1024) as output:
            seq = self._next_ftp_seq()
            self._send_ftp_command(seq, session, self.FTP_OPCODE_BURST_READ_FILE,
                                   size=burst_size, offset=0)

            while True:
                now = time.time()

                if now - last_activity > self.BURST_IDLE_TIMEOUT:
                    if reached_eof and not gaps:
                        break
                    if gaps:
                        self._fill_read_gaps(session, gaps, output)
                        if not gaps:
                            break
                    print(
                        f"[LOG SERVICE] Burst read idle timeout after {now - start_time:.1f}s, "
                        f"read {read_total}/{file_size} B, {len(gaps)} gaps"
                    )
                    break

                try:
                    message = self._ftp_message_queue.get(timeout=self.FTP_PIPELINE_RECV_TIMEOUT)
                except queue.Empty:
                    empty_streak += 1
                    if empty_streak > self.BURST_EMPTY_STREAK_FALLBACK:
                        raise RuntimeError(
                            f"BurstReadFile UART stall (empty_streak={empty_streak}) — pipelined fallback"
                        )
                    backoff = min(
                        self.BURST_RESEND_BACKOFF_CAP,
                        self.BURST_RETRY_TIMEOUT * (2 ** min(empty_streak - 1, 5)),
                    )
                    if not reached_eof and (now - last_activity) >= backoff:
                        seq = self._next_ftp_seq()
                        self._send_ftp_command(
                            seq, session, self.FTP_OPCODE_BURST_READ_FILE,
                            size=burst_size, offset=offset,
                        )
                        last_activity = now
                        empty_streak = 0
                    continue

                parsed = self._parse_ftp_message(message)
                if not self._ftp_targets_acceptable(parsed, master):
                    continue
                if not self._ftp_component_acceptable(parsed, master):
                    continue
                if parsed["session"] != session:
                    continue
                if parsed["req_opcode"] not in (
                    self.FTP_OPCODE_BURST_READ_FILE,
                    self.FTP_OPCODE_READ_FILE,
                ):
                    continue

                empty_streak = 0
                last_activity = time.time()

                if parsed["opcode"] == self.FTP_OPCODE_ACK:
                    pkt_offset = parsed["offset"]
                    pkt_data = parsed["data"]
                    if not pkt_data:
                        continue

                    if pkt_offset > offset:
                        gap_start = offset
                        gap_len = pkt_offset - offset
                        while gap_len > 0:
                            chunk = min(gap_len, burst_size)
                            gaps.append((gap_start, chunk))
                            gap_start += chunk
                            gap_len -= chunk
                        if len(gaps) > self.BURST_GAP_COUNT_FALLBACK:
                            raise RuntimeError(
                                f"BurstReadFile gap count {len(gaps)} — switching to pipelined UART path"
                            )

                    output.seek(pkt_offset)
                    output.write(pkt_data)
                    read_total += len(pkt_data)
                    new_offset = pkt_offset + len(pkt_data)
                    if new_offset > offset:
                        offset = new_offset

                    now_log = time.time()
                    if now_log - last_log >= self.BURST_LOG_INTERVAL_SEC:
                        rate = (read_total / 1024.0) / max(now_log - start_time, 0.001)
                        pct = (read_total / file_size * 100) if file_size else 0
                        print(
                            f"[LOG SERVICE] Burst read: {read_total}/{file_size} B "
                            f"({pct:.0f}%) ~{rate:.1f} KB/s gaps={len(gaps)}"
                        )
                        last_log = now_log

                    if parsed["burst_complete"]:
                        if len(pkt_data) < burst_size:
                            reached_eof = True
                        else:
                            seq = self._next_ftp_seq()
                            self._send_ftp_command(
                                seq, session, self.FTP_OPCODE_BURST_READ_FILE,
                                size=burst_size, offset=offset,
                            )

                elif parsed["opcode"] == self.FTP_OPCODE_NAK:
                    error_code = parsed["data"][0] if parsed["data"] else self.FTP_ERROR_FAIL
                    if error_code == self.FTP_ERROR_EOF:
                        reached_eof = True
                    else:
                        print(f"[LOG SERVICE] Burst read NAK error {error_code} at offset {offset}")

                if reached_eof and not gaps and (read_total >= file_size or file_size == 0):
                    break
                if reached_eof and gaps:
                    self._fill_read_gaps(session, gaps, output)
                    if not gaps:
                        break

        elapsed = time.time() - start_time
        rate = (read_total / 1024.0) / max(elapsed, 0.001)
        print(f"[LOG SERVICE] Burst read complete: {read_total} bytes in {elapsed:.1f}s ({rate:.1f} KB/s)")

        written = file_path.stat().st_size if file_path.exists() else 0
        if file_size > 0 and written < file_size:
            raise RuntimeError(
                f"BurstReadFile incomplete ({written}/{file_size} B on disk) — use pipelined fallback"
            )

    def _fill_read_gaps(self, session, gaps, output):
        """Fill remaining read gaps using sequential ReadFile requests."""
        filled = []
        for gap_offset, gap_length in gaps:
            try:
                reply = self._ftp_request(
                    self.FTP_OPCODE_READ_FILE,
                    session=session,
                    offset=gap_offset,
                    size=gap_length,
                    timeout=0.3,
                    retries=4,
                )
                if reply["opcode"] == self.FTP_OPCODE_ACK and reply["data"]:
                    output.seek(gap_offset)
                    output.write(reply["data"])
                    filled.append((gap_offset, gap_length))
                elif reply["opcode"] == self.FTP_OPCODE_NAK:
                    error_code = reply["data"][0] if reply["data"] else self.FTP_ERROR_FAIL
                    if error_code == self.FTP_ERROR_EOF:
                        filled.append((gap_offset, gap_length))
            except Exception as err:
                print(f"[LOG SERVICE] Gap fill failed at {gap_offset}: {err}")
        for item in filled:
            gaps.remove(item)

    def _ftp_terminate_session(self, session):
        try:
            self._ftp_request(
                self.FTP_OPCODE_TERMINATE_SESSION,
                session=session,
                timeout=0.2,
                retries=2,
            )
        except Exception:
            return

    def _parse_log_id_from_name(self, remote_name):
        stem = Path(remote_name).stem
        match = re.search(r"(\d+)$", stem)
        if match is None:
            return None
        return int(match.group(1))

    def _list_logs_via_mavftp(self):
        if not self.enable_mavftp:
            return []

        for log_dir in self.MAVFTP_LOG_DIRS:
            try:
                with self._ftp_lock:
                    self._clear_ftp_message_queue()
                    self._ftp_reset_sessions()
                    raw_entries = self._ftp_list_directory(log_dir)

                entries = []
                for text in raw_entries:
                    if not text or text[0] != "F":
                        continue
                    try:
                        name, size_text = text[1:].split("\t", 1)
                    except ValueError:
                        continue
                    if not name.lower().endswith(".bin"):
                        continue

                    log_id = self._parse_log_id_from_name(name)
                    if log_id is None:
                        continue

                    entries.append({
                        "id": log_id,
                        "ftp_name": name,
                        "ftp_path": f"{log_dir.rstrip('/')}/{name}",
                        "size": int(size_text),
                    })

                if entries:
                    return sorted(entries, key=lambda item: item["id"], reverse=True)
            except Exception as error:
                print(f"[LOG SERVICE] MAVFTP inventory failed for {log_dir}: {error}")

        return []

    def fetch_log_list(self, timeout=8.0, attempts=2):
        try:
            for _ in range(attempts):
                with self._list_lock:
                    self._list_entries = {}
                    self._list_event.clear()

                self._send_log_request_list()
                self._list_event.wait(timeout=timeout)
                time.sleep(0.5)

                with self._list_lock:
                    entries = list(self._list_entries.values())
                if entries:
                    return entries

            return []
        except Exception as error:
            print(f"[LOG SERVICE] LOG_REQUEST_LIST failed: {error}")
            return []

    def get_latest_entry(self):
        entries = self.fetch_log_list()
        if not entries:
            raise RuntimeError("No logs reported by the flight controller")
        return max(entries, key=lambda item: item["id"])

    def list_flight_controller_logs(self):
        # Use the legacy log inventory path as the primary source because it is
        # what ArduPilot tools already rely on for DataFlash log enumeration.
        mav_entries = {item["id"]: item for item in self.fetch_log_list()}
        if not mav_entries:
            return []

        all_ids = sorted(mav_entries.keys(), reverse=True)
        latest_id = max(all_ids)
        payload = []
        for log_id in all_ids:
            mav_entry = mav_entries.get(log_id, {})
            entry = {
                "id": log_id,
                "size": mav_entry.get("size", 0),
                "time_utc": mav_entry.get("time_utc", 0),
            }
            file_name = self._build_filename(entry)
            cached_path = self.cache_dir / file_name
            payload.append({
                "id": log_id,
                "size": entry["size"],
                "time_utc": entry["time_utc"],
                "is_latest": log_id == latest_id,
                "cached": cached_path.exists() and (entry["size"] == 0 or cached_path.stat().st_size == entry["size"]),
                "cached_name": file_name,
                "transfer_method": "mavftp" if self.enable_mavftp else "log_request_data",
            })
        return payload

    def _wait_for_chunk(self, log_id, offset, count, retries=3, timeout=2.0):
        for _ in range(retries):
            key = (log_id, offset)
            event = threading.Event()
            with self._chunk_lock:
                self._chunk_waiters[key] = {"event": event, "data": None}

            self._send_log_request_data(log_id, offset, count)
            if event.wait(timeout=timeout):
                with self._chunk_lock:
                    waiter = self._chunk_waiters.pop(key, None)
                if waiter and waiter["data"] is not None:
                    return waiter["data"]
            else:
                with self._chunk_lock:
                    self._chunk_waiters.pop(key, None)

        raise RuntimeError(f"Timed out while reading log {log_id} at offset {offset}")

    def _build_filename(self, entry):
        if entry["time_utc"] > 0:
            timestamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(entry["time_utc"]))
        else:
            timestamp = "unknown-time"
        return f"log_{entry['id']:05d}_{timestamp}.bin"

    def download_latest_log(self):
        with self._download_lock:
            logs = self.list_flight_controller_logs()
            if not logs:
                raise RuntimeError("No logs reported by the flight controller")
            latest = next((item for item in logs if item["is_latest"]), None)
            if latest is None:
                raise RuntimeError("No latest log reported by the flight controller")
            entry = {
                "id": latest["id"],
                "size": latest["size"],
                "time_utc": latest["time_utc"],
                "ftp_path": self._ftp_path_for_log_id(latest["id"]),
            }
            return self._download_entry(entry)

    def _ftp_path_for_log_id(self, log_id):
        ftp_entry = next((item for item in self._list_logs_via_mavftp() if item["id"] == log_id), None)
        if ftp_entry is None:
            return None
        return ftp_entry["ftp_path"]

    def _download_entry_via_mavftp(self, entry, file_path):
        if not self.enable_mavftp:
            raise RuntimeError("MAVFTP is disabled for this process")
        remote_path = entry.get("ftp_path")
        if not remote_path:
            raise RuntimeError(f"No MAVFTP path available for log {entry['id']}")

        print(
            f"[LOG SERVICE] Downloading FC log {entry['id']} via MAVFTP "
            f"(BurstReadFile → pipelined ReadFile fallback) from {remote_path} to {file_path}"
        )
        self._mavftp_wire_enter()
        try:
            with self._ftp_lock:
                self._clear_ftp_message_queue()
                self._ftp_reset_sessions()
                session = None
                try:
                    session, remote_file_size = self._ftp_open_file_ro(remote_path)
                    expected_size = entry["size"] if entry["size"] > 0 else remote_file_size
                    try:
                        self._ftp_burst_read_file(session, expected_size, file_path)
                    except Exception as burst_err:
                        print(f"[LOG SERVICE] Burst read failed or stalled ({burst_err}); trying pipelined ReadFile")
                        self._ftp_terminate_session(session)
                        session = None
                        self._clear_ftp_message_queue()
                        self._ftp_reset_sessions()
                        session, remote_file_size = self._ftp_open_file_ro(remote_path)
                        expected_size = entry["size"] if entry["size"] > 0 else remote_file_size
                        try:
                            self._ftp_pipelined_read_file(session, expected_size, file_path)
                        except Exception as pipe_err:
                            print(f"[LOG SERVICE] Pipelined ReadFile failed ({pipe_err}); sequential ReadFile")
                            self._ftp_terminate_session(session)
                            session = None
                            self._clear_ftp_message_queue()
                            self._ftp_reset_sessions()
                            session, remote_file_size = self._ftp_open_file_ro(remote_path)
                            expected_size = entry["size"] if entry["size"] > 0 else remote_file_size
                            self._ftp_read_file(session, expected_size, file_path)
                finally:
                    if session is not None:
                        self._ftp_terminate_session(session)
        finally:
            self._mavftp_wire_leave()

        if not file_path.exists():
            raise RuntimeError(f"MAVFTP download for log {entry['id']} did not produce a file")

        if entry["size"] > 0 and file_path.stat().st_size != entry["size"]:
            print(f"[LOG SERVICE] MAVFTP download size mismatch for log {entry['id']}: "
                  f"expected {entry['size']} bytes, got {file_path.stat().st_size} "
                  f"(non-fatal, log may still be writable on FC)")
            

    def _download_entry_via_log_request(self, entry, file_path):
        print(f"[LOG SERVICE] Downloading FC log {entry['id']} via LOG_REQUEST_DATA to {file_path}")
        offset = 0
        with file_path.open("wb") as output:
            while offset < entry["size"]:
                requested = min(self.CHUNK_SIZE, entry["size"] - offset)
                chunk = self._wait_for_chunk(entry["id"], offset, requested)
                if len(chunk) == 0:
                    raise RuntimeError(f"Received empty chunk while downloading log {entry['id']}")
                output.write(chunk)
                offset += len(chunk)

        self._send_log_request_end()

    def _download_entry(self, entry):
        with self._download_lock:
            self._flight_log_download_enter(f"Saving flight log id {entry['id']} to companion")
            try:
                file_name = self._build_filename(entry)
                file_path = self.cache_dir / file_name

                if file_path.exists() and file_path.stat().st_size == entry["size"]:
                    return self._metadata_for_path(file_path, entry)

                download_error = None
                if entry.get("ftp_path"):
                    try:
                        self._download_entry_via_mavftp(entry, file_path)
                    except Exception as error:
                        download_error = error
                        print(f"[LOG SERVICE] MAVFTP download failed for log {entry['id']}: {error}")
                        if file_path.exists():
                            file_path.unlink()

                if not file_path.exists():
                    if entry["size"] <= 0:
                        raise RuntimeError(
                            f"Log {entry['id']} cannot be downloaded because the controller did not report a valid size"
                        ) from download_error
                    self._download_entry_via_log_request(entry, file_path)

                return self._metadata_for_path(file_path, entry)
            finally:
                self._flight_log_download_leave()

    def download_log_by_id(self, log_id):
        inventory = self.list_flight_controller_logs()
        match = next((entry for entry in inventory if entry["id"] == log_id), None)
        if match is None:
            raise RuntimeError(f"Log id {log_id} not reported by the flight controller")
        entry = {
            "id": match["id"],
            "size": match["size"],
            "time_utc": match["time_utc"],
            "ftp_path": self._ftp_path_for_log_id(match["id"]),
        }
        return self._download_entry(entry)

    def _metadata_for_path(self, file_path, entry=None):
        stat = file_path.stat()
        payload = {
            "name": file_path.name,
            "rel_path": file_path.name,
            "url": f"/logs/{file_path.name}",
            "size": stat.st_size,
            "modified_at": int(stat.st_mtime),
        }
        if entry is not None:
            payload["log_id"] = entry["id"]
            payload["time_utc"] = entry["time_utc"]
        return payload

    def list_cached_logs(self):
        logs = []
        for file_path in sorted(self.cache_dir.glob("*.bin"), key=lambda item: item.stat().st_mtime, reverse=True):
            logs.append(self._metadata_for_path(file_path))
        return logs

    def resolve_log_path(self, relative_name):
        candidate = (self.cache_dir / relative_name).resolve()
        cache_root = self.cache_dir.resolve()
        if cache_root not in candidate.parents and candidate != cache_root / relative_name:
            raise FileNotFoundError(relative_name)
        if not candidate.exists():
            raise FileNotFoundError(relative_name)
        return candidate

    def resolve_web_asset(self, relative_name):
        relative_name = relative_name or "index.html"
        candidate = (self.web_root / relative_name).resolve()
        web_root = self.web_root.resolve()
        if web_root not in candidate.parents and candidate != web_root / relative_name:
            raise FileNotFoundError(relative_name)
        if candidate.is_dir():
            candidate = candidate / "index.html"
        if not candidate.exists():
            raise FileNotFoundError(relative_name)
        return candidate

    # ----------------------------------------------------------------
    # Auto-download latest log on disarm (MAVFTP; status via /api/mavftp-status)
    # ----------------------------------------------------------------

    def auto_download_latest_log(self):
        """
        Triggered when the drone disarms. Downloads the latest log file
        from the FC (MAVFTP-first) in a background thread.
        Skips if a download is already in progress.
        """
        if self._auto_download_in_progress:
            print("[LOG SERVICE] Auto-download already in progress, skipping.")
            return
        t = threading.Thread(
            target=self._auto_download_worker,
            name="AutoLogDownload",
            daemon=True,
        )
        t.start()

    def _auto_download_worker(self):
        self._auto_download_in_progress = True
        self._set_auto_download_msg("Disarm: waiting 5s for FC to finalize log file…")
        print("[LOG SERVICE] Drone disarmed – waiting 5s for FC to finalize log...")
        time.sleep(5)
        try:
            self._set_auto_download_msg("Disarm: downloading latest log (MAVFTP / fallback)…")
            print("[LOG SERVICE] Starting auto-download of latest flight log via MAVFTP...")
            metadata = self.download_latest_log()
            print(f"[LOG SERVICE] [OK] Auto-download complete: {metadata['name']} "
                  f"({metadata['size']} bytes) -> {self.cache_dir / metadata['name']}")
        except Exception as err:
            print(f"[LOG SERVICE] [FAIL] Auto-download of latest log failed: {err}")
        finally:
            self._auto_download_in_progress = False
            self._set_auto_download_msg("")


def start_log_services(vehicle):
    cache_dir = os.environ.get(
        "JECH_FC_LOG_CACHE_DIR",
        str(Path(__file__).resolve().parent.parent / "downloaded_logs"),
    )
    web_root = os.environ.get(
        "JECH_WEBTOOLS_DIR",
        str(Path(__file__).resolve().parent.parent / "webtools"),
    )
    host = os.environ.get("JECH_FC_LOG_HOST", "0.0.0.0")
    port = int(os.environ.get("JECH_FC_LOG_PORT", "8765"))

    service = FlightControllerLogService(vehicle, cache_dir=cache_dir, web_root=web_root, host=host, port=port)
    service.start()
    return service
