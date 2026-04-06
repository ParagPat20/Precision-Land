import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse
import mimetypes


class FlightControllerLogService:
    CHUNK_SIZE = 90

    def __init__(self, vehicle, cache_dir, web_root, host="0.0.0.0", port=8765):
        self.vehicle = vehicle
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.web_root = Path(web_root)
        self.host = host
        self.port = port

        self._download_lock = threading.Lock()
        self._list_lock = threading.Lock()
        self._list_entries = {}
        self._list_event = threading.Event()
        self._chunk_waiters = {}
        self._chunk_lock = threading.Lock()
        self._http_thread = None

        self.vehicle.add_message_listener("LOG_ENTRY", self._on_log_entry)
        self.vehicle.add_message_listener("LOG_DATA", self._on_log_data)

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

    def _send_log_request_list(self):
        master = self.vehicle._master
        master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xFFFF)

    def _send_log_request_data(self, log_id, offset, count):
        master = self.vehicle._master
        master.mav.log_request_data_send(master.target_system, master.target_component, log_id, offset, count)

    def _send_log_request_end(self):
        master = self.vehicle._master
        master.mav.log_request_end_send(master.target_system, master.target_component)

    def fetch_log_list(self, timeout=4.0):
        with self._list_lock:
            self._list_entries = {}
            self._list_event.clear()

        self._send_log_request_list()
        self._list_event.wait(timeout=timeout)
        time.sleep(0.5)

        with self._list_lock:
            return list(self._list_entries.values())

    def get_latest_entry(self):
        entries = self.fetch_log_list()
        if not entries:
            raise RuntimeError("No logs reported by the flight controller")
        return max(entries, key=lambda item: item["id"])

    def list_flight_controller_logs(self):
        entries = self.fetch_log_list()
        if not entries:
            return []

        latest_id = max(item["id"] for item in entries)
        payload = []
        for entry in sorted(entries, key=lambda item: item["id"], reverse=True):
            file_name = self._build_filename(entry)
            cached_path = self.cache_dir / file_name
            payload.append({
                "id": entry["id"],
                "size": entry["size"],
                "time_utc": entry["time_utc"],
                "is_latest": entry["id"] == latest_id,
                "cached": cached_path.exists() and cached_path.stat().st_size == entry["size"],
                "cached_name": file_name,
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
            entry = self.get_latest_entry()
            return self._download_entry(entry)

    def _download_entry(self, entry):
        with self._download_lock:
            file_name = self._build_filename(entry)
            file_path = self.cache_dir / file_name

            if file_path.exists() and file_path.stat().st_size == entry["size"]:
                return self._metadata_for_path(file_path, entry)

            print(f"[LOG SERVICE] Downloading latest FC log {entry['id']} to {file_path}")
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
            return self._metadata_for_path(file_path, entry)

    def download_log_by_id(self, log_id):
        entries = self.fetch_log_list()
        match = next((entry for entry in entries if entry["id"] == log_id), None)
        if match is None:
            raise RuntimeError(f"Log id {log_id} not reported by the flight controller")
        return self._download_entry(match)

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
