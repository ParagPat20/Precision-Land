# Flight-controller log cache HTTP service: list/download DataFlash logs over the existing
# DroneKit MAVLink link using QGC-style LOG_REQUEST_LIST / LOG_REQUEST_DATA chunks.
# No MAVFTP — avoids FILE_TRANSFER_PROTOCOL contention on UART telemetry.

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse
import mimetypes
import queue
from pymavlink import mavutil
from scripts.mavftp import MAVFTP, MAVFTPSettings, FtpError

class FTPMasterWrapper:
    def __init__(self, real_master):
        self.real_master = real_master
        self.msg_queue = queue.Queue()
        self.source_system = real_master.source_system
        self.source_component = real_master.source_component
        self.target_system = real_master.target_system
        self.target_component = real_master.target_component
        self.mav = real_master.mav

    def recv_match(self, type=None, blocking=False, timeout=0):
        try:
            return self.msg_queue.get(block=blocking, timeout=timeout)
        except queue.Empty:
            return None



class FlightControllerLogService:
    # QGroundControl downloads DataFlash logs in 512 LOG_DATA bins per chunk.
    # Since we are over Ethernet/UDP, we can increase this drastically for faster downloads!
    LOG_DATA_LEN = 90
    QGC_CHUNK_BINS = int(os.environ.get("JECH_FC_LOG_QGC_CHUNK_BINS", "4096"))  # 368KB per chunk (was 512)
    QGC_CHUNK_SIZE = LOG_DATA_LEN * QGC_CHUNK_BINS
    QGC_DATA_TIMEOUT_SEC = float(os.environ.get("JECH_FC_LOG_QGC_RETRY_SEC", "0.15")) # Faster retries on UDP (was 0.5)
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
        self._param_cache = {}
        self._param_cache_loaded_at = 0.0
        self._param_value_waiters = {}
        self._param_value_lock = threading.Lock()

        # Streaming download state (QGC-style chunk table).
        self._stream_cv = threading.Condition(threading.RLock())
        self._stream_state = None

        self.ftp_master = FTPMasterWrapper(self.vehicle._master)

        self.vehicle.add_message_listener("LOG_ENTRY", self._on_log_entry)
        self.vehicle.add_message_listener("LOG_DATA", self._on_log_data)
        self.vehicle.add_message_listener("FILE_TRANSFER_PROTOCOL", self._on_ftp)
        self.vehicle.add_message_listener("PARAM_VALUE", self._on_param_value)

    def _on_ftp(self, vehicle, name, message):
        self.ftp_master.msg_queue.put(message)

    def _on_param_value(self, vehicle, name, message):
        raw_id = getattr(message, "param_id", "")
        if isinstance(raw_id, bytes):
            param_id = raw_id.decode("ascii", errors="ignore")
        else:
            param_id = str(raw_id)
        param_id = param_id.rstrip("\x00")
        value = float(getattr(message, "param_value", 0.0))
        ptype = int(getattr(message, "param_type", 0))
        with self._param_value_lock:
            waiter = self._param_value_waiters.get(param_id)
            if waiter is not None:
                waiter["value"] = value
                waiter["type"] = ptype
                waiter["event"].set()

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
                bits.append(self._flight_log_download_label or "MAVFTP log transfer")
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

                if parsed.path == "/api/params":
                    try:
                        query = parse_qs(parsed.query)
                        refresh = query.get("refresh", ["0"])[0] in {"1", "true", "yes"}
                        self._send_json(service.get_params(refresh=refresh))
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
                    self.send_header("Content-Disposition", f'attachment; filename="{file_path.name}"')
                    self.end_headers()
                    with file_path.open("rb") as file_obj:
                        self.wfile.write(file_obj.read())
                    return

                # Same as /logs/ but served inline (no forced download) — used by
                # the Log Finder "Open in Browser" button so the browser can read
                # the binary for in-page analysis tools.
                if parsed.path.startswith("/logs-open/"):
                    relative_name = unquote(parsed.path[len("/logs-open/"):])
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

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path == "/api/params/write":
                    try:
                        body = self._read_json_body()
                        changes = body.get("changes", [])
                        if not isinstance(changes, list):
                            raise ValueError("changes must be a list")
                        self._send_json(service.write_params(changes))
                    except Exception as error:
                        self._send_json({"error": str(error)}, status=500)
                    return

                if parsed.path == "/api/params/compare":
                    try:
                        body = self._read_json_body()
                        text = body.get("text", "")
                        if not isinstance(text, str):
                            raise ValueError("text must be a string")
                        self._send_json(service.compare_param_text(text))
                    except Exception as error:
                        self._send_json({"error": str(error)}, status=500)
                    return

                self._send_json({"error": "Not found"}, status=404)

            def log_message(self, fmt, *args):
                print(f"[LOG SERVICE] {self.address_string()} - {fmt % args}")

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def _read_json_body(self):
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                return json.loads(self.rfile.read(length).decode("utf-8"))

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

    @staticmethod
    def _param_type_to_mavlink(ptype):
        mapping = {
            1: mavutil.mavlink.MAV_PARAM_TYPE_INT8,
            2: mavutil.mavlink.MAV_PARAM_TYPE_INT16,
            3: mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            4: mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        }
        return mapping.get(int(ptype or 0), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    @staticmethod
    def _mavlink_type_to_pack_type(ptype):
        mapping = {
            mavutil.mavlink.MAV_PARAM_TYPE_INT8: 1,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT8: 1,
            mavutil.mavlink.MAV_PARAM_TYPE_INT16: 2,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16: 2,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32: 3,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT32: 3,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32: 4,
        }
        return mapping.get(int(ptype or 0), 4)

    @staticmethod
    def _param_close_enough(a, b):
        return abs(float(a) - float(b)) <= max(1.0e-6, abs(float(b)) * 1.0e-6)

    @staticmethod
    def parse_param_text(text):
        params = {}
        for lineno, raw_line in enumerate(text.splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "#" in line:
                line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if "," in line:
                parts = [part.strip() for part in line.split(",", 1)]
            else:
                parts = line.split()
            if len(parts) < 2:
                raise ValueError(f"Invalid .param line {lineno}: {raw_line}")
            name = parts[0].strip()
            if not name:
                raise ValueError(f"Invalid .param line {lineno}: missing parameter name")
            try:
                value = float(parts[1])
            except ValueError as exc:
                raise ValueError(f"Invalid value for {name} on line {lineno}") from exc
            params[name] = value
        if not params:
            raise ValueError("No parameters found in uploaded file")
        return params

    def _build_ftp_settings(self):
        return MAVFTPSettings([
            ("debug", int, 0),
            ("pkt_loss_tx", int, 0),
            ("pkt_loss_rx", int, 0),
            ("max_backlog", int, 40),
            ("burst_read_size", int, 239),
            ("write_size", int, 239),
            ("write_qsize", int, 5),
            ("idle_detection_time", float, 3.7),
            ("read_retry_time", float, 1.0),
            ("retry_time", float, 0.5),
        ])

    def refresh_params_from_fc(self):
        with self._download_lock:
            print("[PARAM SERVICE] Fetching CUAV parameters via MAVFTP param pack")
            while not self.ftp_master.msg_queue.empty():
                self.ftp_master.msg_queue.get()

            ftp = MAVFTP(self.ftp_master, self.ftp_master.target_system, self.ftp_master.target_component, self._build_ftp_settings())
            pack_path = self.cache_dir / "cuav_param.pck"
            res = ftp.cmd_get(["@PARAM/param.pck", str(pack_path)])
            if res.error_code != FtpError.Success:
                raise RuntimeError(f"MAVFTP parameter fetch initiation failed: {res.error_code}")
            res = ftp.process_ftp_reply("getparams", timeout=float(os.environ.get("JECH_PARAM_DOWNLOAD_TIMEOUT_SEC", "180")))
            if res.error_code != FtpError.Success:
                raise RuntimeError(f"MAVFTP parameter fetch failed: {res.error_code}")

            data = pack_path.read_bytes()
            pdata = MAVFTP.ftp_param_decode(data)
            if pdata is None:
                raise RuntimeError("Could not decode parameter pack")
            values = {}
            for name, value, ptype in pdata.params:
                values[name.decode("utf-8")] = (value, ptype)
            params = {
                name: {"value": float(value), "type": int(ptype)}
                for name, (value, ptype) in sorted(values.items(), key=lambda item: item[0].split("_"))
            }
            if not params:
                raise RuntimeError("No parameters decoded from CUAV")
            self._param_cache = params
            self._param_cache_loaded_at = time.time()
            print(f"[PARAM SERVICE] Loaded {len(params)} CUAV parameters")
            return self.get_params(refresh=False)

    def get_params(self, refresh=False):
        if refresh or not self._param_cache:
            return self.refresh_params_from_fc()
        return {
            "params": self._param_cache,
            "count": len(self._param_cache),
            "loaded_at": int(self._param_cache_loaded_at),
        }

    def compare_param_text(self, text):
        current = self.get_params(refresh=False)["params"]
        uploaded = self.parse_param_text(text)
        changes = []
        missing = []
        unchanged = 0
        for name in sorted(uploaded.keys(), key=lambda item: item.split("_")):
            target = uploaded[name]
            if name not in current:
                missing.append({"name": name, "to": target})
                continue
            old = float(current[name]["value"])
            if self._param_close_enough(old, target):
                unchanged += 1
                continue
            changes.append({
                "name": name,
                "from": old,
                "to": target,
                "type": int(current[name].get("type", 4)),
            })
        return {
            "changes": changes,
            "missing": missing,
            "unchanged": unchanged,
            "uploaded_count": len(uploaded),
            "current_count": len(current),
        }

    def write_params(self, changes):
        with self._download_lock:
            if not self._param_cache:
                self.refresh_params_from_fc()
            results = []
            for item in changes:
                name = str(item.get("name", "")).strip()
                if not name:
                    raise ValueError("Parameter name is required")
                if len(name.encode("ascii", errors="ignore")) > 16:
                    raise ValueError(f"Parameter name is too long for MAVLink PARAM_SET: {name}")
                if "to" not in item:
                    raise ValueError(f"Missing target value for {name}")
                target_value = float(item["to"])
                cached = self._param_cache.get(name, {})
                pack_type = int(item.get("type") or cached.get("type") or 4)
                mav_type = self._param_type_to_mavlink(pack_type)
                ack = self._write_one_param(name, target_value, mav_type)
                ack_pack_type = self._mavlink_type_to_pack_type(ack.get("type", mav_type))
                self._param_cache[name] = {"value": float(ack["value"]), "type": ack_pack_type}
                results.append({
                    "name": name,
                    "from": item.get("from", cached.get("value")),
                    "to": target_value,
                    "verified": self._param_close_enough(ack["value"], target_value),
                    "actual": float(ack["value"]),
                    "type": ack_pack_type,
                })
                time.sleep(float(os.environ.get("JECH_PARAM_WRITE_GAP_SEC", "0.05")))
            self._param_cache_loaded_at = time.time()
            return {"written": results, "count": len(results)}

    def _write_one_param(self, name, value, mav_type):
        master = self.vehicle._master
        pid = name.encode("ascii", errors="ignore")[:16].ljust(16, b"\x00")
        waiter = {"event": threading.Event(), "value": None, "type": None}
        with self._param_value_lock:
            self._param_value_waiters[name] = waiter
        try:
            for attempt in range(3):
                master.mav.param_set_send(master.target_system, master.target_component, pid, float(value), int(mav_type))
                if waiter["event"].wait(timeout=float(os.environ.get("JECH_PARAM_ACK_TIMEOUT_SEC", "2.0"))):
                    return {"value": float(waiter["value"]), "type": int(waiter["type"] or mav_type)}
                print(f"[PARAM SERVICE] PARAM_SET retry {attempt + 1} for {name}")
            raise RuntimeError(f"No PARAM_VALUE acknowledgement for {name}")
        finally:
            with self._param_value_lock:
                self._param_value_waiters.pop(name, None)

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
                "transfer_method": "mavftp",
            })
        return payload

    def _build_filename(self, entry):
        if entry["time_utc"] > 0:
            timestamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(entry["time_utc"]))
        else:
            timestamp = "unknown-time"
        return f"log_{entry['id']:05d}_{timestamp}.bin"

    def download_latest_log(self):
        print("Not doing it")

    def _download_entry_via_mavftp(self, entry, file_path):
        log_id = entry["id"]
        expected = int(entry["size"])
        print(f"[LOG SERVICE] Downloading FC log {log_id} via MAVFTP -> {file_path}")
        
        now = time.time()
        st = {
            "log_id": log_id,
            "expected_size": expected,
            "bytes_written": 0,
            "fh": None,
            "path": file_path,
            "rate_bytes": 0,
            "rate_packets": 0,
            "current_rate_bps": 0.0,
            "average_rate_bps": 0.0,
            "packet_rate": 0.0,
            "start_ts": now,
            "last_rate_ts": now,
            "last_ts": now,
            "complete": False,
            "transfer_method": "mavftp",
        }
        with self._stream_cv:
            self._stream_state = st
            
        settings = MAVFTPSettings([
            ("debug", int, 0),
            ("pkt_loss_tx", int, 0),
            ("pkt_loss_rx", int, 0),
            ("max_backlog", int, 40),
            ("burst_read_size", int, 300),
            ("write_size", int, 300),
            ("write_qsize", int, 5),
            ("idle_detection_time", float, 3.7),
            ("read_retry_time", float, 1.0),
            ("retry_time", float, 0.5),
        ])
        
        try:
            ftp = MAVFTP(self.ftp_master, self.ftp_master.target_system, self.ftp_master.target_component, settings)
            remote_path = f"/APM/LOGS/{int(log_id):08d}.BIN"
            
            def ftp_progress_callback(percentage):
                now_cb = time.time()
                with self._stream_cv:
                    dt = max(now_cb - st["last_rate_ts"], 0.000001)
                    if dt >= 1.0 or ftp.read_total >= expected:
                        bytes_diff = ftp.read_total - st["bytes_written"]
                        st["current_rate_bps"] = bytes_diff / dt
                        st["last_rate_ts"] = now_cb
                        st["bytes_written"] = ftp.read_total
                    st["average_rate_bps"] = ftp.read_total / max(now_cb - st["start_ts"], 0.000001)

            # clear queue
            while not self.ftp_master.msg_queue.empty():
                self.ftp_master.msg_queue.get()
                
            res = ftp.cmd_get([remote_path, str(file_path)], progress_callback=ftp_progress_callback)
            if res.error_code != FtpError.Success:
                raise RuntimeError(f"MAVFTP download initiation failed: {res.error_code}")
            
            res = ftp.process_ftp_reply("get", timeout=self.LOG_DOWNLOAD_WALL_TIMEOUT_SEC)
            if res.error_code != FtpError.Success:
                raise RuntimeError(f"MAVFTP download failed: {res.error_code}")
                
            with self._stream_cv:
                st["complete"] = True
                
        finally:
            with self._stream_cv:
                self._stream_state = None
                
        file_size = file_path.stat().st_size
        elapsed = max(time.time() - st["start_ts"], 0.000001)
        print(f"[LOG SERVICE] MAVFTP download complete: log {log_id}, size {file_size}, {file_size / elapsed / 1024.0:.1f} KB/s")

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
                    self._download_entry_via_mavftp(entry, file_path)
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
            "transfer_method": "mavftp",
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
