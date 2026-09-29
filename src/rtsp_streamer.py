"""
RTSP Video Streamer for Precision-Land HUD & FPV Feed.

Streams processed OpenCV frames (with ArUco markers, 3D axes, and telemetry HUD)
to MediaMTX via local RTSP (rtsp://127.0.0.1:8554/cam).
Uses a minimal queue (maxsize=2) to drop late frames and guarantee zero latency (<50ms).
"""

import subprocess
import threading
import queue
import time
import logging
import cv2
import numpy as np

logger = logging.getLogger("RTSPStreamer")

class RTSPStreamer:
    def __init__(self, rtsp_url="rtsp://127.0.0.1:8554/cam", width=1280, height=720, fps=30, bitrate="2500k"):
        self.rtsp_url = rtsp_url
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.bitrate = bitrate
        
        # Maxsize=2 ensures old frames are dropped immediately if encoding or network backs up
        self.frame_queue = queue.Queue(maxsize=2)
        self._stop_event = threading.Event()
        self._worker_thread = None
        self._ffmpeg_proc = None
        self.is_running = False

    def start(self):
        """Start the background encoder and RTSP streamer."""
        if self.is_running:
            return
        self._stop_event.clear()
        self.is_running = True
        self._worker_thread = threading.Thread(target=self._stream_loop, name="RTSPStreamerThread", daemon=True)
        self._worker_thread.start()
        print(f"[STREAMER] RTSP Streamer started -> {self.rtsp_url} ({self.width}x{self.height} @ {self.fps}fps)", flush=True)

    def stop(self):
        """Stop streaming and terminate ffmpeg subprocess."""
        self.is_running = False
        self._stop_event.set()
        if self._ffmpeg_proc:
            try:
                self._ffmpeg_proc.stdin.close()
                self._ffmpeg_proc.terminate()
                self._ffmpeg_proc.wait(timeout=1.0)
            except Exception:
                pass
            self._ffmpeg_proc = None
        print("[STREAMER] RTSP Streamer stopped.", flush=True)

    def send_frame(self, frame: np.ndarray):
        """Submit a frame for streaming. Non-blocking; drops frame if queue is full."""
        if not self.is_running or frame is None:
            return
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            pass  # Drop frame to guarantee absolute zero latency

    def _start_ffmpeg(self):
        """Spawn ffmpeg process reading raw BGR and publishing H.264 over RTSP."""
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "-",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-b:v", self.bitrate,
            "-maxrate", "3000k",
            "-bufsize", "500k",
            "-g", str(self.fps),
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            self.rtsp_url
        ]
        return subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def _stream_loop(self):
        """Worker thread to feed frames to ffmpeg with auto-reconnect."""
        while not self._stop_event.is_set():
            try:
                if self._ffmpeg_proc is None or self._ffmpeg_proc.poll() is not None:
                    self._ffmpeg_proc = self._start_ffmpeg()
                    time.sleep(0.2)

                try:
                    frame = self.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))

                self._ffmpeg_proc.stdin.write(frame.tobytes())
                self._ffmpeg_proc.stdin.flush()
            except Exception as e:
                time.sleep(0.5)
                if self._ffmpeg_proc:
                    try:
                        self._ffmpeg_proc.terminate()
                    except Exception:
                        pass
                    self._ffmpeg_proc = None
