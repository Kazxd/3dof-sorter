"""
camera.py - always-fresh frames from the phone stream.

cv2.VideoCapture queues frames internally. If the main loop is slower than the
stream (YOLO on a CPU usually is), every read() returns an older and older
frame and the robot ends up reaching for where a part *was*. A background
thread that reads continuously and keeps only the newest frame fixes that.

The thread also reconnects by itself when Wi-Fi drops, and can serve a still
image or a video file instead of a stream for offline testing.
"""
import logging
import threading
import time
from pathlib import Path

import cv2

import config as C

log = logging.getLogger("camera")
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".bmp"}


def _parse_source(source):
    if isinstance(source, int):
        return source
    s = str(source).strip()
    return int(s) if s.isdigit() else s


class FrameGrabber(threading.Thread):
    def __init__(self, source=None):
        super().__init__(daemon=True, name="FrameGrabber")
        self.source = _parse_source(C.CAMERA_URL if source is None else source)
        self._lock = threading.Lock()
        self._new = threading.Condition(self._lock)
        self._frame = None
        self._ts = 0.0
        self._count = 0           # increases by one per new frame
        self._running = True
        self.connected = False

    # ------------------------------------------------------------ thread body
    def run(self):
        if isinstance(self.source, str) and Path(self.source).suffix.lower() in IMAGE_EXT:
            self._serve_still(self.source)
            return
        while self._running:
            cap = cv2.VideoCapture(self.source)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # honoured by some backends only
            except cv2.error:
                pass
            if not cap.isOpened():
                self.connected = False
                log.warning("cannot open %s, retrying in %.0f s", self.source, C.CAMERA_RECONNECT_S)
                time.sleep(C.CAMERA_RECONNECT_S)
                continue
            log.info("camera connected: %s", self.source)
            self.connected = True
            is_file = isinstance(self.source, str) and Path(self.source).exists()
            while self._running:
                ok, frame = cap.read()
                if not ok or frame is None:
                    if is_file:                          # loop video files
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    break
                self._publish(frame)
                if is_file:
                    time.sleep(1 / 30.0)
            cap.release()
            self.connected = False
            if self._running:
                log.warning("camera stream lost, reconnecting")
                time.sleep(C.CAMERA_RECONNECT_S / 2)

    def _serve_still(self, path):
        frame = cv2.imread(path)
        if frame is None:
            log.error("could not read image %s", path)
            return
        self.connected = True
        while self._running:
            self._publish(frame.copy())
            time.sleep(1 / 15.0)

    def _publish(self, frame):
        with self._new:
            self._frame = frame
            self._ts = time.time()
            self._count += 1
            self._new.notify_all()

    # ------------------------------------------------------------ public API
    def stop(self):
        self._running = False

    def alive(self):
        """True if a frame arrived recently."""
        with self._lock:
            return self._frame is not None and time.time() - self._ts < C.CAMERA_STALE_S

    def latest(self):
        """(frame, timestamp) of the newest frame, or (None, 0)."""
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame.copy(), self._ts

    def fresh_frames(self, n, not_before, timeout=6.0):
        """Collect n *different* frames captured at or after time `not_before`.
        Returns fewer than n (possibly []) if the camera stalls before the timeout."""
        frames = []
        deadline = time.time() + timeout
        last_count = -1
        with self._new:
            while len(frames) < n:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                if self._frame is not None and self._count != last_count and self._ts >= not_before:
                    frames.append(self._frame.copy())
                    last_count = self._count
                    continue
                self._new.wait(timeout=min(remaining, 0.5))
        return frames
