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
    FTP_OPCODE_ACK = 128
    FTP_OPCODE_NAK = 129
    FTP_ERROR_NONE = 0
    FTP_ERROR_FAIL = 1
    FTP_ERROR_EOF = 6
    FTP_ERROR_FILE_NOT_FOUND = 10

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

        self.vehicle.add_message_listener("LOG_ENTRY", self._on_log_entry)
        self.vehicle.add_message_listener("LOG_DATA", self._on_log_data)
        self.vehicle.add_message_listener("FILE_TRANSFER_PROTOCOL", self._on_file_transfer_protocol)

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
            master.target_component,
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
            if parsed["target_system"] != master.source_system or parsed["target_component"] != master.source_component:
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
            self._ftp_request(self.FTP_OPCODE_RESET_SESSIONS, timeout=0.2, retries=2)
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
                timeout=0.2,
                retries=6,
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
            timeout=0.2,
            retries=6,
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

        print(f"[LOG SERVICE] Downloading FC log {entry['id']} via MAVFTP from {remote_path} to {file_path}")
        with self._ftp_lock:
            self._clear_ftp_message_queue()
            self._ftp_reset_sessions()
            session = None
            try:
                session, remote_file_size = self._ftp_open_file_ro(remote_path)
                expected_size = entry["size"] if entry["size"] > 0 else remote_file_size
                self._ftp_read_file(session, expected_size, file_path)
            finally:
                if session is not None:
                    self._ftp_terminate_session(session)

        if not file_path.exists():
            raise RuntimeError(f"MAVFTP download for log {entry['id']} did not produce a file")

        if entry["size"] > 0 and file_path.stat().st_size != entry["size"]:
            raise RuntimeError(
                f"MAVFTP download size mismatch for log {entry['id']}: expected {entry['size']} bytes, got {file_path.stat().st_size}"
            )

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
