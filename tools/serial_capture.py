"""Receive JPEG frames from the XIAO ESP32S3 over USB serial or Wi-Fi and display them.

Sources:
  USB serial (firmware/camera_hello), the default:
      ===IMG_START <len>===
      <base64 lines>
      ===IMG_END===
    Anything outside those markers (boot log, ESP_LOGx output) is printed and ignored.
    Close `idf.py monitor` first: only one program can hold the port.

  Wi-Fi MJPEG stream (firmware/camera_wifi), with --wifi [HOST or URL]:
    multipart/x-mixed-replace over HTTP, http://<host>:81/stream. Reconnects
    automatically if the camera reboots or the link drops.

Usage:
  python tools/serial_capture.py [--port /dev/ttyACM0 | --wifi [HOST|URL]]
                                 [--save [DIR]] [--aruco [DICT]]
                                 [--video [FILE]] [--video-overlay]
Stop with Ctrl+C (or `q` in the image window). With --video, the received frames are
then exported to an MP4 whose playback speed matches real time.
With --aruco, every frame is searched for ArUco markers and their outlines, corners
and ids are drawn in the live view (and in the video with --video-overlay).
"""

import argparse
import base64
import binascii
import datetime
import http.client
import re
import time
import urllib.parse
from pathlib import Path

import cv2
import numpy as np

DEFAULT_WIFI_HOST = "xiao-cam.local"
JPEG_SOI = b"\xff\xd8"


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_overlay(img, fps, n_bytes, n_markers=None):
    """Draw the fps / frame-size text used both on screen and (optionally) in the video."""
    text = f"{fps:4.1f} fps  {n_bytes / 1024:.1f} KB"
    if n_markers is not None:
        text += f"  {n_markers} marker{'s' if n_markers != 1 else ''}"
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


class MarkerDetector:
    """OpenCV ArUco detection. Returns corners in pixel coordinates, no pose estimation
    (that needs camera calibration, see tools/calibrate_camera.py)."""

    def __init__(self, dictionary_name):
        dict_id = getattr(cv2.aruco, dictionary_name, None)
        if dict_id is None:
            raise ValueError(f"unknown ArUco dictionary {dictionary_name!r} (e.g. DICT_4X4_50)")
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    def detect(self, img):
        """Return a list of (marker_id, corners) with corners as a (4, 2) float array,
        ordered top-left, top-right, bottom-right, bottom-left (marker's own orientation)."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        return [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)]


CORNER_COLORS = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]  # BGR: TL red, TR green,
                                                                       # BR blue, BL yellow


def draw_markers(img, markers):
    """Outline each marker, dot its 4 corners (colour = corner index), label its id."""
    for marker_id, corners in markers:
        pts = corners.astype(np.int32)
        cv2.polylines(img, [pts], isClosed=True, color=(255, 255, 0), thickness=2)
        for k, (x, y) in enumerate(pts):
            cv2.circle(img, (int(x), int(y)), 5, CORNER_COLORS[k], -1)
        cx, cy = pts.mean(axis=0).astype(int)
        cv2.putText(img, f"id {marker_id}", (int(cx) - 20, int(cy) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------

class StopCapture(Exception):
    """Raised when the user presses `q` in the image window."""


class VideoRecorder:
    """Keeps received JPEGs in memory (~15 KB each) and writes an MP4 at the end.

    Frames arrive at an irregular rate, so instead of writing them back to back, the
    exporter samples them on a fixed output clock: each output frame shows the newest
    received frame at that moment. Playback duration therefore matches wall-clock time.
    """

    def __init__(self, path, fps=None, overlay=False):
        self.path = Path(path)
        self.fps = fps
        self.overlay = overlay
        # (monotonic timestamp, jpeg bytes, live fps, detected markers or None)
        self.frames = []

    def add(self, jpeg, live_fps, markers=None):
        self.frames.append((time.monotonic(), jpeg, live_fps, markers))

    def export(self):
        if len(self.frames) < 2:
            print("[video] fewer than 2 frames received, nothing to export")
            return
        t0, t_end = self.frames[0][0], self.frames[-1][0]
        duration = t_end - t0
        fps = self.fps or max(1, round((len(self.frames) - 1) / duration))

        first = cv2.imdecode(np.frombuffer(self.frames[0][1], np.uint8), cv2.IMREAD_COLOR)
        height, width = first.shape[:2]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                 (width, height))
        if not writer.isOpened():
            print(f"[video] could not open {self.path} for writing")
            return

        n_out = int(duration * fps) + 1
        src = 0
        decoded_idx, decoded = -1, None
        for k in range(n_out):
            t = t0 + k / fps
            while src + 1 < len(self.frames) and self.frames[src + 1][0] <= t:
                src += 1
            if src != decoded_idx:  # decode each source frame only once
                img = cv2.imdecode(np.frombuffer(self.frames[src][1], np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is not None:
                    if img.shape[:2] != (height, width):
                        img = cv2.resize(img, (width, height))
                    if self.overlay:
                        _, jpeg, live_fps, markers = self.frames[src]
                        if markers is not None:
                            draw_markers(img, markers)
                        draw_overlay(img, live_fps, len(jpeg),
                                     None if markers is None else len(markers))
                    decoded = img
                decoded_idx = src
            writer.write(decoded)
        writer.release()
        print(f"[video] wrote {self.path}: {n_out} frames @ {fps} fps, {duration:.1f} s "
              f"({len(self.frames)} received frames)")


class FpsMeter:
    """Smoothed frames-per-second from the time between consecutive frames."""

    def __init__(self, smoothing=0.9):
        self.smoothing = smoothing
        self.fps = 0.0
        self._last = None

    def tick(self):
        now = time.monotonic()
        if self._last is not None:
            inst = 1.0 / max(now - self._last, 1e-6)
            self.fps = inst if self.fps == 0.0 else (
                self.smoothing * self.fps + (1 - self.smoothing) * inst)
        self._last = now
        return self.fps


# ---------------------------------------------------------------------------
# Frame sources: each is a generator yielding complete, validated JPEG bytes
# ---------------------------------------------------------------------------

START_RE = re.compile(r"^===IMG_START (\d+)===$")
END_MARKER = "===IMG_END==="
B64_RE = re.compile(r"^[A-Za-z0-9+/=]+$")  # a valid base64 line, nothing else
B64_LINE_LEN = 76  # firmware encodes 57-byte chunks -> every line but the last is 76 chars


def decode_serial_frame(b64_text, expected_len):
    """Base64 text -> JPEG bytes, or None (with a message) if it is damaged."""
    try:
        data = base64.b64decode(b64_text, validate=True)
    except (binascii.Error, ValueError) as e:
        print(f"[ERROR] bad base64, dropping frame: {e}")
        return None
    if len(data) != expected_len:
        print(f"[ERROR] decoded {len(data)} bytes, expected {expected_len}")
        return None
    if data[:2] != JPEG_SOI:
        print("[ERROR] not a JPEG (missing FF D8 start marker)")
        return None
    return data


def frames_from_serial(port):
    """Yield JPEGs sent by firmware/camera_hello over USB serial."""
    import serial  # only needed for this source

    # Baud rate is ignored by native USB CDC, but pyserial needs a value.
    ser = serial.Serial(port, 115200, timeout=1)
    ser.dtr = False  # avoid DTR/RTS toggling that can reset the ESP32-S3
    ser.rts = False
    print(f"listening on {port}, Ctrl+C to quit")

    collecting = False
    corrupt = False
    expected_len = 0
    chunks = []
    dropped = 0
    try:
        while True:
            raw = ser.readline()  # returns b"" after `timeout` seconds with no data
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()

            m = START_RE.match(line)
            if m:
                if collecting:
                    print("[warn] new START before END, dropping incomplete frame")
                collecting = True
                corrupt = False
                expected_len = int(m.group(1))
                chunks = []
                continue

            if collecting and line == END_MARKER:
                collecting = False
                # The camera driver prints from an interrupt (e.g. "cam_hal: FB-OVF"), which
                # can land in the middle of a base64 line and split it. Unrecoverable.
                if corrupt or any(len(c) != B64_LINE_LEN for c in chunks[:-1]):
                    dropped += 1
                    print(f"[warn] frame corrupted by interleaved output, dropped "
                          f"({dropped} total)")
                else:
                    data = decode_serial_frame("".join(chunks), expected_len)
                    if data is not None:
                        yield data
                continue

            if collecting and B64_RE.match(line):
                chunks.append(line)
            elif line:
                if collecting:
                    corrupt = True  # something else was printed inside the frame
                print(f"[dev] {line}")  # log line from the device
    finally:
        ser.close()


def stream_url(target):
    """'xiao-cam.local' / '192.168.1.57' / full URL -> MJPEG stream URL."""
    if "://" in target:
        return target
    return f"http://{target}:81/stream"


def frames_from_http(url, timeout_s=5.0, retry_s=1.0):
    """Yield JPEGs from a multipart/x-mixed-replace (MJPEG) HTTP stream.

    Parsed by hand instead of cv2.VideoCapture so we keep the raw JPEG bytes (for
    --save / --video) and the per-frame headers. http.client transparently undoes the
    chunked transfer encoding that esp_http_server uses.
    """
    u = urllib.parse.urlsplit(url)
    path = (u.path or "/") + (f"?{u.query}" if u.query else "")
    while True:
        conn = None
        try:
            conn = http.client.HTTPConnection(u.hostname, u.port or 80, timeout=timeout_s)
            conn.request("GET", path)
            resp = conn.getresponse()
            if resp.status != 200:
                raise ConnectionError(f"HTTP {resp.status} {resp.reason}")
            ctype = resp.getheader("Content-Type", "")
            m = re.search(r"boundary=\"?([^\";]+)\"?", ctype)
            if not ctype.startswith("multipart/") or not m:
                raise ConnectionError(f"not an MJPEG stream (Content-Type: {ctype!r})")
            boundary = b"--" + m.group(1).encode()
            print(f"connected to {url}, Ctrl+C to quit")

            while True:
                # Skip to the next boundary line (the stream starts with a bare "\r\n").
                line = resp.readline()
                if not line:
                    raise ConnectionError("stream closed by camera")
                if line.strip() != boundary:
                    continue
                headers = {}
                while True:
                    h = resp.readline()
                    if not h:
                        raise ConnectionError("stream closed by camera")
                    h = h.strip()
                    if not h:
                        break  # blank line ends the part headers
                    key, _, value = h.decode("latin-1").partition(":")
                    headers[key.strip().lower()] = value.strip()
                length = int(headers.get("content-length", "0"))
                if length <= 0:
                    print("[warn] part without Content-Length, skipping")
                    continue
                data = resp.read(length)
                if len(data) != length:
                    raise ConnectionError("stream ended mid-frame")
                if data[:2] != JPEG_SOI:
                    print("[warn] part is not a JPEG, skipping")
                    continue
                yield data
        except (OSError, http.client.HTTPException, ConnectionError, ValueError) as e:
            # OSError covers timeouts, refused connections and unknown hostnames.
            print(f"[wifi] {e}; retrying in {retry_s:g} s...")
            time.sleep(retry_s)
        finally:
            if conn is not None:
                conn.close()


# ---------------------------------------------------------------------------
# Shared pipeline
# ---------------------------------------------------------------------------

def handle_image(data, fps_meter, save_dir=None, recorder=None, detector=None):
    """Optionally save/record, run detection on, and display one JPEG frame."""
    if save_dir is not None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        (save_dir / f"img_{ts}.jpg").write_bytes(data)

    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print("[ERROR] OpenCV could not decode the JPEG")
        return
    fps = fps_meter.tick()
    markers = detector.detect(img) if detector is not None else None
    if recorder is not None:
        # raw JPEG + detections; overlay is re-drawn at export if requested
        recorder.add(data, fps, markers)
    if markers is not None:
        draw_markers(img, markers)
    draw_overlay(img, fps, len(data), None if markers is None else len(markers))
    cv2.imshow("xiao", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        raise StopCapture


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--port", default="/dev/ttyACM0",
                     help="USB serial port (default source: /dev/ttyACM0)")
    src.add_argument("--wifi", nargs="?", const=DEFAULT_WIFI_HOST, default=None,
                     metavar="HOST|URL",
                     help=f"read the Wi-Fi MJPEG stream instead (default host: "
                          f"{DEFAULT_WIFI_HOST},\nURL http://<host>:81/stream)")
    p.add_argument("--save", "--saved", nargs="?", const="captures", default=None, metavar="DIR",
                   help="save every received JPEG to DIR (default: captures/)")
    p.add_argument("--aruco", nargs="?", const="DICT_4X4_50", default=None, metavar="DICT",
                   help="detect ArUco markers and draw them (default dictionary: DICT_4X4_50,\n"
                        "same as config/default.yaml and tools/generate_markers.py)")
    p.add_argument("--video", nargs="?", const="", default=None, metavar="FILE",
                   help="after capture ends, export frames to an MP4\n"
                        "(default: captures/video_<timestamp>.mp4)")
    p.add_argument("--video-fps", type=int, default=None,
                   help="output video fps (default: measured average capture fps)")
    p.add_argument("--video-overlay", action="store_true",
                   help="draw the live fps / frame size / markers into the exported video")
    args = p.parse_args()
    if args.video is None and (args.video_overlay or args.video_fps):
        p.error("--video-overlay / --video-fps require --video")

    detector = None
    if args.aruco is not None:
        try:
            detector = MarkerDetector(args.aruco)
        except ValueError as e:
            p.error(str(e))
        print(f"ArUco detection on ({args.aruco})")

    recorder = None
    if args.video is not None:
        path = args.video or datetime.datetime.now().strftime("captures/video_%Y%m%d_%H%M%S.mp4")
        recorder = VideoRecorder(path, args.video_fps, args.video_overlay)
        print(f"recording; video will be written to {path} when capture ends")

    save_dir = None
    if args.save is not None:
        save_dir = Path(args.save)
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f"saving frames to {save_dir}/")

    if args.wifi is not None:
        frames = frames_from_http(stream_url(args.wifi))
    else:
        frames = frames_from_serial(args.port)

    fps_meter = FpsMeter()
    try:
        for jpeg in frames:
            handle_image(jpeg, fps_meter, save_dir, recorder, detector)
    except (KeyboardInterrupt, StopCapture):
        pass
    finally:
        frames.close()  # closes the serial port / HTTP connection
        cv2.destroyAllWindows()
    if recorder is not None:
        recorder.export()


if __name__ == "__main__":
    main()
