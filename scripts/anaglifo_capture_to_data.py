#!/usr/bin/env python3
"""Capture grayscale image streamed by anaglifo_receiver.ino and save to data/.

Expected receiver serial framing:
- IMG_BEGIN <id> <width> <height> <len>
- <len> raw grayscale bytes
- IMG_END <id> <ok:0|1> <chunks> <bytes>
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path
import re
import sys

import serial
import serial.tools.list_ports

BEGIN_RE = re.compile(r"^IMG_BEGIN\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")
END_RE = re.compile(r"^IMG_END\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")


def auto_detect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found")

    preferred_keywords = (
        "usb",
        "cp210",
        "ch340",
        "uart",
        "xiao",
        "esp32",
    )

    for p in ports:
        hay = f"{p.device} {p.description} {p.manufacturer}".lower()
        if any(k in hay for k in preferred_keywords):
            return p.device

    # Fallback to first available port.
    return ports[0].device


def read_ascii_line(ser: serial.Serial) -> str:
    line = ser.readline()
    if not line:
        return ""
    return line.decode("utf-8", errors="ignore").strip()


def save_pgm(path: Path, width: int, height: int, pixels: bytes) -> None:
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + pixels)


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture images from anaglifo receiver over serial")
    parser.add_argument("--port", help="Serial port (for example /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=2_000_000, help="Serial baud rate")
    parser.add_argument(
        "--output-dir",
        default=str((Path(__file__).resolve().parent.parent / "data")),
        help="Folder where output PGM files are saved",
    )
    parser.add_argument("--one", action="store_true", help="Capture one image and exit")
    args = parser.parse_args()

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    port = args.port or auto_detect_port()
    print(f"[CAPTURE] Using serial port: {port}")
    print(f"[CAPTURE] Output directory: {output_dir}")

    try:
        ser = serial.Serial(port, args.baud, timeout=2.0)
    except Exception as exc:
        print(f"[CAPTURE] Could not open serial port: {exc}")
        return 1

    saved_count = 0

    with ser:
        print("[CAPTURE] Waiting for IMG_BEGIN...")
        while True:
            line = read_ascii_line(ser)
            if not line:
                continue

            m = BEGIN_RE.match(line)
            if not m:
                # Show useful receiver status lines without stopping.
                if line.startswith("RECEIVER_") or line.startswith("IMG_"):
                    print(f"[RX] {line}")
                continue

            image_id = int(m.group(1))
            width = int(m.group(2))
            height = int(m.group(3))
            data_len = int(m.group(4))

            print(
                f"[CAPTURE] BEGIN id={image_id} width={width} "
                f"height={height} len={data_len}"
            )

            pixels = ser.read(data_len)
            if len(pixels) != data_len:
                print(
                    f"[CAPTURE] Incomplete payload: got={len(pixels)} expected={data_len}. "
                    "Waiting for next frame."
                )
                continue

            end_line = read_ascii_line(ser)
            end_match = END_RE.match(end_line)
            if not end_match:
                print(f"[CAPTURE] Invalid end line: {end_line!r}")
                continue

            end_id = int(end_match.group(1))
            ok = int(end_match.group(2))
            chunks = int(end_match.group(3))
            end_bytes = int(end_match.group(4))

            if end_id != image_id:
                print(f"[CAPTURE] ID mismatch: begin={image_id} end={end_id}")
                continue
            if ok != 1:
                print(f"[CAPTURE] Receiver reported failure for image {image_id}")
                continue
            if end_bytes != data_len:
                print(f"[CAPTURE] Byte mismatch in end line: end={end_bytes} expected={data_len}")
                continue

            ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_name = f"espnow_rx_{image_id:04d}_{width}x{height}_{ts}.pgm"
            out_path = output_dir / out_name
            save_pgm(out_path, width, height, pixels)

            saved_count += 1
            print(f"[CAPTURE] Saved {out_path} (chunks={chunks})")

            if args.one:
                print("[CAPTURE] Done (--one).")
                return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[CAPTURE] Stopped by user")
        raise SystemExit(0)
