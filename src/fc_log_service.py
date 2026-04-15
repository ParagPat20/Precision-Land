# Flight-controller log cache HTTP service: list/download DataFlash logs over the existing
# DroneKit MAVLink link using QGC-style LOG_REQUEST_LIST / LOG_REQUEST_DATA chunks.
# No MAVFTP — avoids FILE_TRANSFER_PROTOCOL contention on UART telemetry.

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse
import mimetypes
from pymavlink import mavutil


class FlightControllerLogService:
    # QGroundControl downloads DataFlash logs in 512 LOG_DATA bins per chunk.
    LOG_DATA_LEN = 90
    QGC_CHUNK_BINS = 512
    QGC_CHUNK_SIZE = LOG_DATA_LEN * QGC_CHUNK_BINS
    QGC_DATA_TIMEOUT_SEC = float(os.environ.get("JECH_FC_LOG_QGC_RETRY_SEC", "0.5"))
    LOG_DOWNLOAD_WALL_TIMEOUT_SEC = float(os.environ.get("JECH_FC_LOG_DOWNLOAD_TIMEOUT_SEC", "7200"))
    PAUSE_TELEMETRY_DURING_DOWNLOAD = os.environ.get("JECH_FC_LOG_PAUSE_TELEMETRY", "1") != "0"

    def __init__(self, vehicle, cache_dir, web_root, host="0.0.0.0", port=8765):
        self.vehicle = vehicle
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.web_root = Path(web_root)
        self.host = host
        self.port = port

        self._download_lock = threading.RLock()
        self._list_lock = threading.Lock()
        self._list_entries = {}
        self._list_event = threading.Event()
        self._http_thread = None
        self._auto_download_in_progress = False
        self._client_activity_lock = threading.Lock()
        self._flight_log_download_active = False
        self._flight_log_download_started_mono = 0.0
        self._flight_log_download_label = ""
        self._auto_download_status_msg = ""

        # Streaming download state (QGC-style chunk table).
        self._stream_cv = threading.Condition(threading.RLock())
        self._stream_state = None

        self.vehicle.add_message_listener("LOG_ENTRY", self._on_log_entry)
        self.vehicle.add_message_listener("LOG_DATA", self._on_log_data)

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

    def log_download_public_status(self):
        """Snapshot for Log Finder (GET /api/log-download-status): bytes received vs expected, elapsed time."""
        with self._stream_cv:
            st = self._stream_state
            bytes_downloaded = int(st.get("bytes_written", 0)) if st else 0
            expected_bytes = int(st.get("expected_size", 0)) if st else 0
            log_id = st.get("log_id") if st else None
            current_kbps = round(float(st.get("current_rate_bps", 0.0)) / 1024.0, 1) if st else 0.0
            average_kbps = round(float(st.get("average_rate_bps", 0.0)) / 1024.0, 1) if st else 0.0
            packet_rate = round(float(st.get("packet_rate", 0.0)), 1) if st else 0.0
            missing_bins = int(st.get("missing_bins", 0)) if st else 0
            retries = int(st.get("retries", 0)) if st else 0
            transfer_method = st.get("transfer_method") if st else ""
        with self._client_activity_lock:
            auto_wait = self._auto_download_in_progress and not self._flight_log_download_active
            busy = self._flight_log_download_active or auto_wait
            elapsed = 0.0
            if self._flight_log_download_active and self._flight_log_download_started_mono > 0:
                elapsed = time.monotonic() - self._flight_log_download_started_mono
            bits = []
            if auto_wait:
                bits.append(self._auto_download_status_msg or "Auto log download (waiting)")
            if self._flight_log_download_active:
                bits.append(self._flight_log_download_label or "LOG_REQUEST_DATA log transfer")
            detail = " — ".join(bits) if bits else ""
        return {
            "busy": busy,
            "detail": detail,
            "elapsed_sec": round(elapsed, 1) if busy else 0.0,
            "auto_download_pending": auto_wait,
            "bytes_downloaded": bytes_downloaded,
            "expected_bytes": expected_bytes,
            "log_id": log_id,
            "current_kbps": current_kbps,
            "average_kbps": average_kbps,
            "packet_rate": packet_rate,
            "missing_bins": missing_bins,
            "retries": retries,
            "transfer_method": transfer_method,
        }

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

                if parsed.path == "/api/log-download-status":
                    self._send_json(service.log_download_public_status())
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
                    relative_web_path = unquote(parsed.path[len("/webtools/"):])
                    if relative_web_path:
                        possible_dir = (service.web_root / relative_web_path).resolve()
                        if possible_dir.is_dir() and not parsed.path.endswith("/"):
                            self.send_response(301)
                            location = parsed.path + "/"
                            if parsed.query:
                                location += "?" + parsed.query
                            self.send_header("Location", location)
                            self.end_headers()
                            return
                    try:
                        asset_path = service.resolve_web_asset(relative_web_path)
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
        """Route LOG_DATA to an active QGC-style chunk download if log id matches."""
        with self._stream_cv:
            st = self._stream_state
            if st is None or int(message.id) != st["log_id"]:
                return
            fh = st["fh"]
            ofs = int(message.ofs)
            cnt = int(message.count)
            expected = int(st["expected_size"])
            if ofs % self.LOG_DATA_LEN != 0 or ofs > expected:
                return

            chunk = ofs // self.QGC_CHUNK_SIZE
            if chunk != st["current_chunk"]:
                return

            bin_index = (ofs - chunk * self.QGC_CHUNK_SIZE) // self.LOG_DATA_LEN
            if bin_index >= len(st["chunk_table"]):
                return

            payload = bytes(message.data[:cnt])
            fh.seek(ofs)
            fh.write(payload)

            if ofs not in st["written_offsets"]:
                st["written_offsets"].add(ofs)
                st["written_ranges"].append((ofs, cnt))
                st["bytes_written"] += cnt
                st["rate_bytes"] += cnt
                st["rate_packets"] += 1

            st["chunk_table"][bin_index] = True
            st["last_ts"] = time.time()

            self._qgc_update_rates_locked(st)
            if self._qgc_log_complete_locked(st):
                st["complete"] = True
            elif self._qgc_chunk_complete_locked(st):
                self._qgc_advance_chunk_locked(st)
                self._qgc_request_missing_range_locked(st)
            elif bin_index < len(st["chunk_table"]) - 1 and st["chunk_table"][bin_index + 1]:
                self._qgc_request_missing_range_locked(st)

            st["missing_bins"] = st["chunk_table"].count(False)
            self._stream_cv.notify_all()

    def _send_log_request_list(self):
        master = self.vehicle._master
        master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xFFFF)

    def _send_log_request_data(self, log_id, offset, count):
        master = self.vehicle._master
        master.mav.log_request_data_send(master.target_system, master.target_component, log_id, offset, count)

    def _send_log_request_end(self):
        master = self.vehicle._master
        master.mav.log_request_end_send(master.target_system, master.target_component)

    def _set_telemetry_streams(self, rate_hz):
        master = self.vehicle._master
        streams = (
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        )
        for stream_id in streams:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream_id,
                rate_hz,
                1 if rate_hz > 0 else 0,
            )

    def _qgc_num_chunks(self, expected_size):
        return max(1, (int(expected_size) + self.QGC_CHUNK_SIZE - 1) // self.QGC_CHUNK_SIZE)

    def _qgc_chunk_bins(self, expected_size, chunk):
        remaining = max(0, int(expected_size) - int(chunk) * self.QGC_CHUNK_SIZE)
        bins = (remaining + self.LOG_DATA_LEN - 1) // self.LOG_DATA_LEN
        return max(1, min(self.QGC_CHUNK_BINS, bins))

    def _qgc_chunk_complete_locked(self, st):
        return bool(st["chunk_table"]) and all(st["chunk_table"])

    def _qgc_log_complete_locked(self, st):
        return self._qgc_chunk_complete_locked(st) and (
            st["current_chunk"] + 1 == self._qgc_num_chunks(st["expected_size"])
        )

    def _qgc_advance_chunk_locked(self, st):
        st["current_chunk"] += 1
        st["chunk_table"] = [False] * self._qgc_chunk_bins(st["expected_size"], st["current_chunk"])
        st["missing_bins"] = len(st["chunk_table"])

    def _qgc_update_rates_locked(self, st):
        now = time.time()
        dt = now - st["last_rate_ts"]
        if dt < 1.0:
            return
        st["current_rate_bps"] = st["rate_bytes"] / max(dt, 0.000001)
        st["packet_rate"] = st["rate_packets"] / max(dt, 0.000001)
        total_dt = max(now - st["start_ts"], 0.000001)
        st["average_rate_bps"] = st["bytes_written"] / total_dt
        st["rate_bytes"] = 0
        st["rate_packets"] = 0
        st["last_rate_ts"] = now

    def _qgc_request_missing_range_locked(self, st):
        if self._qgc_chunk_complete_locked(st):
            return
        start = 0
        while start < len(st["chunk_table"]) and st["chunk_table"][start]:
            start += 1
        end = start
        while end < len(st["chunk_table"]) and not st["chunk_table"][end]:
            end += 1
        offset = st["current_chunk"] * self.QGC_CHUNK_SIZE + start * self.LOG_DATA_LEN
        count = (end - start) * self.LOG_DATA_LEN
        if count <= 0:
            return
        master = self.vehicle._master
        master.mav.log_request_data_send(master.target_system, master.target_component, st["log_id"], offset, count)
        st["retries"] += 1
        st["last_request_ts"] = time.time()
        st["missing_bins"] = st["chunk_table"].count(False)

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
                "transfer_method": "qgc_log_request_data",
            })
        return payload

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
            }
            return self._download_entry(entry)

    def _download_entry_via_log_request(self, entry, file_path):
        """
        QGC-style LOG_REQUEST_DATA download: 512 LOG_DATA bins per chunk,
        retrying the first missing contiguous range until the chunk is complete.
        """
        log_id = entry["id"]
        expected = int(entry["size"])
        print(
            f"[LOG SERVICE] Downloading FC log {log_id} via QGC LOG_REQUEST_DATA chunks -> {file_path}"
        )
        fh = file_path.open("wb")
        fh.truncate(expected)
        now = time.time()
        st = {
            "log_id": log_id,
            "expected_size": expected,
            "bytes_written": 0,
            "fh": fh,
            "path": file_path,
            "current_chunk": 0,
            "chunk_table": [False] * self._qgc_chunk_bins(expected, 0),
            "missing_bins": self._qgc_chunk_bins(expected, 0),
            "written_offsets": set(),
            "written_ranges": [],
            "rate_bytes": 0,
            "rate_packets": 0,
            "current_rate_bps": 0.0,
            "average_rate_bps": 0.0,
            "packet_rate": 0.0,
            "retries": 0,
            "start_ts": now,
            "last_rate_ts": now,
            "last_ts": now,
            "last_request_ts": 0.0,
            "complete": False,
            "transfer_method": "qgc_log_request_data",
        }
        with self._stream_cv:
            self._stream_state = st
        try:
            if self.PAUSE_TELEMETRY_DURING_DOWNLOAD:
                self._set_telemetry_streams(0)
            with self._stream_cv:
                self._qgc_request_missing_range_locked(st)
            deadline = time.monotonic() + self.LOG_DOWNLOAD_WALL_TIMEOUT_SEC
            while time.monotonic() < deadline:
                with self._stream_cv:
                    if st["complete"]:
                        break
                    self._stream_cv.wait(timeout=0.05)
                    if st["complete"]:
                        break
                    if time.time() - st["last_request_ts"] >= self.QGC_DATA_TIMEOUT_SEC:
                        self._qgc_request_missing_range_locked(st)
                        st["last_ts"] = time.time()
                    self._qgc_update_rates_locked(st)
            with self._stream_cv:
                ok = st["complete"]
                unique_bytes = sum(count for _ofs, count in st["written_ranges"])
            if not ok:
                raise RuntimeError(
                    f"Log {log_id} download incomplete or timed out "
                    f"(got {unique_bytes} of {expected} B, chunk {st['current_chunk']}, "
                    f"missing {st['missing_bins']} bins)"
                )
        finally:
            fh.flush()
            fh.close()
            if self.PAUSE_TELEMETRY_DURING_DOWNLOAD:
                self._set_telemetry_streams(4)
            with self._stream_cv:
                self._stream_state = None
            self._send_log_request_end()

        file_size = file_path.stat().st_size
        unique_bytes = sum(count for _ofs, count in st["written_ranges"])
        if file_size != expected or unique_bytes != expected:
            raise RuntimeError(
                f"Log {log_id} failed size check: file_size={file_size}, "
                f"unique_payload_bytes={unique_bytes}, expected={expected}"
            )
        elapsed = max(time.time() - st["start_ts"], 0.000001)
        print(
            f"[LOG SERVICE] QGC download complete: log {log_id}, "
            f"{self._qgc_num_chunks(expected)} chunks, {len(st['written_offsets'])} LOG_DATA packets, "
            f"{expected / elapsed / 1024.0:.1f} KB/s"
        )

    def _download_entry(self, entry):
        with self._download_lock:
            self._flight_log_download_enter(f"Saving flight log id {entry['id']} to companion")
            try:
                file_name = self._build_filename(entry)
                file_path = self.cache_dir / file_name

                if file_path.exists() and file_path.stat().st_size == entry["size"]:
                    return self._metadata_for_path(file_path, entry)

                if entry["size"] <= 0:
                    raise RuntimeError(
                        f"Log {entry['id']} cannot be downloaded because the controller did not report a valid size"
                    )

                if file_path.exists():
                    file_path.unlink()
                try:
                    self._download_entry_via_log_request(entry, file_path)
                except Exception:
                    if file_path.exists():
                        file_path.unlink()
                    raise
                return self._metadata_for_path(file_path, entry)
            finally:
                self._flight_log_download_leave()

    def download_log_by_id(self, log_id):
        inventory = self.list_flight_controller_logs()
        match = next((item for item in inventory if item["id"] == log_id), None)
        if match is None:
            raise RuntimeError(f"Log id {log_id} not reported by the flight controller")
        entry = {
            "id": match["id"],
            "size": match["size"],
            "time_utc": match["time_utc"],
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
            "transfer_method": "qgc_log_request_data",
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
    # Auto-download latest log on disarm (QGC LOG_REQUEST_DATA chunks; progress via /api/log-download-status)
    # ----------------------------------------------------------------

    def auto_download_latest_log(self):
        """
        Triggered when the drone disarms. Auto-download is disabled.
        Please download manually from logfinder.
        """
        print("[LOG SERVICE] Auto-download disabled. Please download logs manually from logfinder.")
        return

    def _auto_download_worker(self):
        self._auto_download_in_progress = True
        self._set_auto_download_msg("Disarm: waiting 5s for FC to finalize log file…")
        print("[LOG SERVICE] Drone disarmed – waiting 5s for FC to finalize log...")
        time.sleep(5)
        try:
            self._set_auto_download_msg("Disarm: downloading latest log (QGC LOG_REQUEST_DATA)…")
            print("[LOG SERVICE] Starting auto-download of latest flight log…")
            metadata = self.download_latest_log()
            print(
                f"[LOG SERVICE] [OK] Auto-download complete: {metadata['name']} "
                f"({metadata['size']} bytes) -> {self.cache_dir / metadata['name']}"
            )
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
