"""Receive base64-encoded JPEG frames from the XIAO ESP32S3 over USB serial.

Protocol (see firmware/camera_hello/main/camera_hello.c):
    ===IMG_START <len>===
    <base64 lines>
    ===IMG_END===
Anything outside those markers (boot log, ESP_LOGx output) is printed and ignored.

Usage: python tools/serial_capture.py [--port /dev/ttyACM0] [--save [DIR]] [--video [FILE]] [--video-overlay]
Stop with Ctrl+C (or `q` in the image window). With --video, the received frames are
then exported to an MP4 whose playback speed matches real time.
Close `idf.py monitor` first: only one program can hold the port.
"""

import argparse
import base64
import binascii
import datetime
import re
import time
from pathlib import Path

import cv2
import numpy as np
import serial

START_RE = re.compile(r"^===IMG_START (\d+)===$")
END_MARKER = "===IMG_END==="
B64_RE = re.compile(r"^[A-Za-z0-9+/=]+$")  # a valid base64 line, nothing else
B64_LINE_LEN = 76  # firmware encodes 57-byte chunks -> every line but the last is 76 chars


def draw_overlay(img, fps, n_bytes):
    """Draw the fps / frame-size text used both on screen and (optionally) in the video."""
    cv2.putText(img, f"{fps:4.1f} fps  {n_bytes / 1024:.1f} KB", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


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
        self.frames = []  # (monotonic timestamp, jpeg bytes, live fps when received)

    def add(self, jpeg, live_fps):
        self.frames.append((time.monotonic(), jpeg, live_fps))

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
                        _, jpeg, live_fps = self.frames[src]
                        draw_overlay(img, live_fps, len(jpeg))
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


def handle_image(b64_text, expected_len, fps_meter, save_dir=None, recorder=None):
    """Decode, validate, optionally save/record, and display one frame."""
    try:
        data = base64.b64decode(b64_text, validate=True)
    except (binascii.Error, ValueError) as e:
        print(f"[ERROR] bad base64, dropping frame: {e}")
        return
    if len(data) != expected_len:
        print(f"[ERROR] decoded {len(data)} bytes, expected {expected_len}")
        return
    if data[:2] != b"\xff\xd8":
        print("[ERROR] not a JPEG (missing FF D8 start marker)")
        return

    if save_dir is not None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        (save_dir / f"img_{ts}.jpg").write_bytes(data)

    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print("[ERROR] OpenCV could not decode the JPEG")
        return
    fps = fps_meter.tick()
    if recorder is not None:
        recorder.add(data, fps)  # raw JPEG; overlay is re-drawn at export if requested
    draw_overlay(img, fps, expected_len)
    cv2.imshow("xiao", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        raise StopCapture


def read_frames(ser, save_dir=None, recorder=None):
    fps_meter = FpsMeter()
    collecting = False
    corrupt = False
    expected_len = 0
    chunks = []
    dropped = 0

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
            # The camera driver prints from an interrupt (e.g. "cam_hal: FB-OVF"), which can
            # land in the middle of a base64 line and split it. Such frames are unrecoverable.
            if corrupt or any(len(c) != B64_LINE_LEN for c in chunks[:-1]):
                dropped += 1
                print(f"[warn] frame corrupted by interleaved output, dropped ({dropped} total)")
            else:
                handle_image("".join(chunks), expected_len, fps_meter, save_dir, recorder)
            continue

        if collecting and B64_RE.match(line):
            chunks.append(line)
        elif line:
            if collecting:
                corrupt = True  # something else was printed inside the frame
            print(f"[dev] {line}")  # log line from the device


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--save", "--saved", nargs="?", const="captures", default=None, metavar="DIR",
                   help="save every received JPEG to DIR (default: captures/)")
    p.add_argument("--video", nargs="?", const="", default=None, metavar="FILE",
                   help="after capture ends, export frames to an MP4\n"
                        "(default: captures/video_<timestamp>.mp4)")
    p.add_argument("--video-fps", type=int, default=None,
                   help="output video fps (default: measured average capture fps)")
    p.add_argument("--video-overlay", action="store_true",
                   help="draw the live fps / frame size text into the exported video")
    args = p.parse_args()
    if args.video is None and (args.video_overlay or args.video_fps):
        p.error("--video-overlay / --video-fps require --video")

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

    # Baud rate is ignored by native USB CDC, but pyserial needs a value.
    ser = serial.Serial(args.port, 115200, timeout=1)
    ser.dtr = False  # avoid DTR/RTS toggling that can reset the ESP32-S3
    ser.rts = False
    print(f"listening on {args.port}, Ctrl+C to quit")
    try:
        read_frames(ser, save_dir, recorder)
    except (KeyboardInterrupt, StopCapture):
        pass
    finally:
        ser.close()
        cv2.destroyAllWindows()
    if recorder is not None:
        recorder.export()


if __name__ == "__main__":
    main()
