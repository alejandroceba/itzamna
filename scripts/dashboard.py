# %% [markdown]
# Unified Telemetry + Image Dashboard (VS Code Interactive)
#
# Usage:
# 1) Open this file in VS Code.
# 2) Run cells from top to bottom using Run Cell.
# 3) The plot appears in the Interactive Window.

# %%
# Required for VS Code Interactive/Jupyter rendering
%matplotlib widget

# %%
from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import re
import threading
import time

from collections import deque
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports


LABELS = [
    "Paquete #",
    "Distancia",
    "RSSI",
    "Latencia",
    "Tasa de Perdida",
    "Rendimiento",
    "Temp BME280",
    "Temp DS18B20",
    "Presion",
    "Altitud",
    "Velocidad X",
    "Velocidad Y",
    "Velocidad Z",
    "Aceleracion X",
    "Aceleracion Y",
    "Aceleracion Z",
]
UNITS = [
    "#",
    "m",
    "dBm",
    "ms",
    "%",
    "kbps",
    "degC",
    "degC",
    "hPa",
    "m",
    "m/s",
    "m/s",
    "m/s",
    "m/s^2",
    "m/s^2",
    "m/s^2",
]

NUM_VARS = len(LABELS)
NUM_COLS = 4
NUM_ROWS = math.ceil(NUM_VARS / NUM_COLS)

# Image line protocol from merged receiver.ino
BEGIN_RE = re.compile(r"^IMG_BEGIN\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")
CHUNK_RE = re.compile(r"^IMG_CHUNK\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9A-Fa-f]+)\s*$")
END_RE = re.compile(r"^IMG_END\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")
PROTOCOL_TOKEN_RE = re.compile(r"IMG_BEGIN|IMG_CHUNK|IMG_END|IMG_DBG|RELAY_STATUS|RECEIVER_[A-Z_]+|SENSOR_MAC|IMAGE_MAC")

# Binary image frame protocol from receiver.ino
SERIAL_IMG_SOF = b"\xA5\x5A"
SERIAL_IMG_FRAME_BEGIN = 1
SERIAL_IMG_FRAME_CHUNK = 2
SERIAL_IMG_FRAME_END = 3
SERIAL_IMG_MAX_PAYLOAD = 240

# ---------- USER CONFIG ----------
SERIAL_PORT = "/dev/ttyACM1"
BAUD = 460_800
MAX_POINTS = 120
SENSOR_TIMEOUT_SEC = 2.5
# ESP-NOW image bursts can pause for several seconds; keep assembly alive longer.
IMAGE_ASSEMBLY_TIMEOUT_SEC = 20.0

DATA_DIR = (Path(__file__).resolve().parent.parent / "data")
SENSOR_LOG_PATH = DATA_DIR / "telemetry_merged.csv"
IMAGE_DIR = DATA_DIR
DEBUG_LOG_PATH = DATA_DIR / "image_debug.log"
RAW_SERIAL_LOG_PATH = DATA_DIR / "receiver_serial_raw.log"
LATENCY_MD_PATH = (Path(__file__).resolve().parent.parent / "docs" / "espnow_image_repro_2026-04-11.md")


@dataclass
class ImageAssembly:
    image_id: int
    width: int
    height: int
    data_len: int
    next_chunk: int
    payload: bytearray


def list_candidate_ports() -> list[serial.tools.list_ports_common.ListPortInfo]:
    ports = list(serial.tools.list_ports.comports())

    def score(p: serial.tools.list_ports_common.ListPortInfo) -> int:
        device = (p.device or "").lower()
        desc = (p.description or "").lower()
        man = (p.manufacturer or "").lower()
        text = f"{device} {desc} {man}"

        # Prefer USB CDC/USB UART ports and avoid legacy ttyS.
        s = 0
        if "/dev/ttyacm" in device:
            s += 100
        if "/dev/ttyusb" in device:
            s += 90
        if "usb" in text or "cp210" in text or "ch340" in text or "xiao" in text or "esp32" in text:
            s += 30
        if "/dev/ttys" in device:
            s -= 200
        return s

    return sorted(ports, key=score, reverse=True)


def pick_serial_port(explicit: str | None) -> str:
    if explicit:
        return explicit

    candidates = list_candidate_ports()
    if not candidates:
        raise RuntimeError("No serial ports found")

    # Pick the first likely usable USB serial port.
    for c in candidates:
        dev = (c.device or "").lower()
        if "/dev/ttyacm" in dev or "/dev/ttyusb" in dev:
            return c.device

    return candidates[0].device


def open_serial_with_fallback(port_name: str, baud: int) -> serial.Serial:
    # Try selected port first.
    try:
        return serial.Serial(port_name, baud, timeout=0.02)
    except Exception as first_exc:
        print(f"Primary port failed ({port_name}): {first_exc}")

    # If the user set an explicit port, fail fast to avoid attaching to
    # the wrong board silently.
    if SERIAL_PORT:
        raise RuntimeError(f"Could not open configured SERIAL_PORT {port_name}: {first_exc}")

    # Try other candidates if auto-selection picked a bad device.
    for c in list_candidate_ports():
        if c.device == port_name:
            continue
        try:
            print(f"Trying fallback port: {c.device}")
            return serial.Serial(c.device, baud, timeout=0.02)
        except Exception:
            continue

    raise RuntimeError("Could not open any serial port. Set SERIAL_PORT explicitly, e.g. /dev/ttyACM0")


def parse_sensor_csv(line_data: str) -> list[float] | None:
    values = line_data.split(",")
    if len(values) != NUM_VARS:
        return None
    try:
        return [float(v) for v in values]
    except ValueError:
        return None


def save_pgm(path: Path, width: int, height: int, pixels: bytes) -> None:
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + pixels)


def save_ppm(path: Path, width: int, height: int, pixels: bytes) -> None:
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + pixels)


def rgb565_to_rgb888(payload: bytes) -> bytes:
    p = np.frombuffer(payload, dtype=">u2")
    r5 = (p >> 11) & 0x1F
    g6 = (p >> 5) & 0x3F
    b5 = p & 0x1F

    r8 = (r5 * 255 // 31).astype(np.uint8)
    g8 = (g6 * 255 // 63).astype(np.uint8)
    b8 = (b5 * 255 // 31).astype(np.uint8)

    rgb = np.empty((p.size, 3), dtype=np.uint8)
    rgb[:, 0] = r8
    rgb[:, 1] = g8
    rgb[:, 2] = b8
    return rgb.tobytes()


# %%
# Runtime setup
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(IMAGE_DIR, exist_ok=True)

selected_port = pick_serial_port(SERIAL_PORT)
print(f"Serial port: {selected_port}")
print(f"Registro de sensores: {SENSOR_LOG_PATH}")
print(f"Directorio de imagenes: {IMAGE_DIR}")

ser = open_serial_with_fallback(selected_port, BAUD)
print(f"Conectado a: {ser.port}")

need_header = not SENSOR_LOG_PATH.exists()
sensor_log_file = open(SENSOR_LOG_PATH, "a", newline="")
sensor_writer = csv.writer(sensor_log_file)
if need_header:
    sensor_writer.writerow(["Timestamp"] + LABELS)
    sensor_log_file.flush()

debug_log_file = open(DEBUG_LOG_PATH, "a", newline="")
raw_serial_log_file = open(RAW_SERIAL_LOG_PATH, "a", newline="")


def dbg(msg: str) -> None:
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    debug_log_file.write(f"{stamp} {msg}\n")


def log_raw_serial_line(line: str) -> None:
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    raw_serial_log_file.write(f"{stamp} {line}\n")


def append_latency_markdown(image_id: int, width: int, height: int, chunks: int, latency_ms: float, file_name: str) -> None:
    """Append one latency record per received image into the reproducibility markdown file."""
    try:
        LATENCY_MD_PATH.parent.mkdir(parents=True, exist_ok=True)

        if not LATENCY_MD_PATH.exists():
            LATENCY_MD_PATH.write_text("# ESP-NOW Image Relay Reproducibility Notes\n\n", encoding="utf-8")

        current = LATENCY_MD_PATH.read_text(encoding="utf-8")
        if "## Image Latency Log" not in current:
            with LATENCY_MD_PATH.open("a", encoding="utf-8") as f:
                f.write("\n## Image Latency Log\n\n")
                f.write("| Timestamp | Image ID | Resolution | Chunks | Latency (ms) | File |\n")
                f.write("|---|---:|---|---:|---:|---|\n")

        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with LATENCY_MD_PATH.open("a", encoding="utf-8") as f:
            f.write(
                f"| {stamp} | {image_id} | {width}x{height} | {chunks} | {latency_ms:.0f} | {file_name} |\n"
            )
    except Exception as exc:
        dbg(f"LATENCY_MD_WRITE_FAIL err={exc}")


time_buffer = deque([""] * MAX_POINTS, maxlen=MAX_POINTS)
initial_values = [0.0] * NUM_VARS
initial_values[6] = 25.0   # Temp BME280
initial_values[7] = 25.0   # Temp DS18B20
initial_values[8] = 800.0  # Presion
initial_values[9] = 1200.0 # Altitud
data_buffers = [deque([initial_values[i]] * MAX_POINTS, maxlen=MAX_POINTS) for i in range(NUM_VARS)]

# Re-running this cell in VS Code Interactive can leave a previous widget alive.
# Clean it up so only one dashboard is displayed.
try:
    prev_ani = globals().get("_dashboard_ani")
    if prev_ani is not None:
        prev_ani.event_source.stop()
except Exception:
    pass

try:
    prev_timer = globals().get("_dashboard_timer")
    if prev_timer is not None:
        prev_timer.stop()
except Exception:
    pass

try:
    prev_fig = globals().get("_dashboard_fig")
    if prev_fig is not None:
        plt.close(prev_fig)
except Exception:
    pass

fig = plt.figure(num="", figsize=(16, 9), constrained_layout=True)
fig.patch.set_facecolor("white")
gs = fig.add_gridspec(NUM_ROWS, NUM_COLS + 1, width_ratios=[1.0, 1.0, 1.0, 1.0, 1.35])
try:
    # Hide the ipympl header that shows "Figure N" in VS Code Interactive.
    fig.canvas.header_visible = False
    fig.canvas.footer_visible = False
    fig.canvas.toolbar_visible = False
    fig.canvas.toolbar_position = "bottom"
    # Keep the widget inside the visible interactive viewport at 100% zoom.
    fig.canvas.layout.width = "100%"
    fig.canvas.layout.height = "78vh"
    fig.canvas.manager.set_window_title("")
    fig.set_label("")
except Exception:
    pass

axes = []
lines = []
value_texts = []
for i in range(NUM_VARS):
    r = i // NUM_COLS
    c = i % NUM_COLS
    ax = fig.add_subplot(gs[r, c])
    line, = ax.plot(range(MAX_POINTS), data_buffers[i], color="#0072B2", linewidth=1.9)
    ax.set_facecolor("white")
    ax.grid(True, axis="y", color="#111111", alpha=0.14, linewidth=0.6)
    ax.spines["top"].set_visible(False)
    ax.spines["left"].set_visible(False)
    ax.set_xticks([])
    ax.tick_params(axis="x", length=0)
    ax.tick_params(axis="y", labelsize=8, colors="#111111")
    ax.text(
        0.02,
        0.93,
        LABELS[i],
        transform=ax.transAxes,
        fontsize=9,
        fontweight="bold",
        color="white",
        bbox=dict(facecolor="#111111", edgecolor="none", boxstyle="round,pad=0.25"),
    )
    val_txt = ax.text(
        0.98,
        0.93,
        f"0 {UNITS[i]}",
        transform=ax.transAxes,
        ha="right",
        va="center",
        fontsize=10,
        fontweight="bold",
        color="#111111",
    )
    axes.append(ax)
    lines.append(line)
    value_texts.append(val_txt)

right_gs = gs[:, NUM_COLS].subgridspec(3, 1, height_ratios=[4.2, 1.0, 1.0], hspace=0.14)

ax_img = fig.add_subplot(right_gs[0, 0])
preview = np.zeros((120, 120), dtype=np.uint8)
img_artist = ax_img.imshow(preview, cmap="gray", vmin=0, vmax=255)
ax_img.set_xticks([])
ax_img.set_yticks([])
ax_img.set_xlabel("")
ax_img.xaxis.label.set_visible(False)

status_text = ax_img.text(
    0.02,
    0.02,
    "Sin imagen aun",
    transform=ax_img.transAxes,
    color="white",
    fontsize=10,
    bbox=dict(facecolor="black", alpha=0.55, edgecolor="none"),
)

ax_anaglyph = fig.add_subplot(right_gs[1, 0])
ax_anaglyph.set_facecolor("white")
ax_anaglyph.set_xticks([])
ax_anaglyph.set_yticks([])
for sp in ax_anaglyph.spines.values():
    sp.set_color("#111111")
anaglyph_card_text = ax_anaglyph.text(
    0.03,
    0.92,
    "Anaglifo\nGuardadas: 0\nDescartadas: 0\nUltima imagen: --",
    transform=ax_anaglyph.transAxes,
    va="top",
    fontsize=10,
    color="#111111",
)

ax_system = fig.add_subplot(right_gs[2, 0])
ax_system.set_facecolor("white")
ax_system.set_xticks([])
ax_system.set_yticks([])
for sp in ax_system.spines.values():
    sp.set_color("#111111")
system_card_text = ax_system.text(
    0.03,
    0.92,
    f"Sistema\nHora: {datetime.now().strftime('%H:%M:%S')}\nSensores: esperando",
    transform=ax_system.transAxes,
    va="top",
    fontsize=10,
    color="#111111",
)

current_image: ImageAssembly | None = None
current_image_last_activity_mono = 0.0
current_image_begin_mono = 0.0
binary_image_mode_active = False
last_sensor_time = 0.0
last_sensor_timestamp = "--:--:--"
last_image_time = 0.0
last_image_latency_ms = 0.0
saved_images = 0
dropped_images = 0
latest_relay_status = "Relay: sin datos"

max_lines_per_update = 6000
serial_rx_buffer = bytearray()
serial_parse_buffer = bytearray()
serial_rx_lock = threading.Lock()
serial_reader_stop = threading.Event()
serial_reader_thread: threading.Thread | None = None
last_file_flush_mono = 0.0


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def serial_reader_loop() -> None:
    """Continuously drain serial bytes so UI/render stalls do not drop frame data."""
    global serial_rx_buffer
    read_error_count = 0

    while not serial_reader_stop.is_set():
        try:
            chunk = ser.read(4096)
        except Exception as exc:
            read_error_count += 1
            dbg(f"SERIAL_READ_ERROR count={read_error_count} err={exc}")
            time.sleep(0.05)
            continue

        if read_error_count:
            dbg(f"SERIAL_READ_RECOVERED count={read_error_count}")
            read_error_count = 0

        if not chunk:
            continue

        with serial_rx_lock:
            serial_rx_buffer.extend(chunk)
            if len(serial_rx_buffer) > 4_000_000:
                dbg(f"RX_BUFFER_TRIM size={len(serial_rx_buffer)}")
                serial_rx_buffer = serial_rx_buffer[-2_000_000:]


def start_serial_reader() -> None:
    global serial_reader_thread

    if serial_reader_thread is not None and serial_reader_thread.is_alive():
        return

    serial_reader_stop.clear()
    serial_reader_thread = threading.Thread(target=serial_reader_loop, name="serial-reader", daemon=True)
    serial_reader_thread.start()


def stop_serial_reader() -> None:
    global serial_reader_thread

    serial_reader_stop.set()
    if serial_reader_thread is not None:
        serial_reader_thread.join(timeout=1.0)
    serial_reader_thread = None


def handle_img_begin(line: str) -> None:
    global current_image, current_image_last_activity_mono, current_image_begin_mono
    m = BEGIN_RE.match(line)
    if not m:
        return

    image_id = int(m.group(1))
    width = int(m.group(2))
    height = int(m.group(3))
    data_len = int(m.group(4))

    current_image = ImageAssembly(
        image_id=image_id,
        width=width,
        height=height,
        data_len=data_len,
        next_chunk=0,
        payload=bytearray(),
    )
    current_image_last_activity_mono = time.monotonic()
    current_image_begin_mono = current_image_last_activity_mono
    dbg(f"BEGIN id={image_id} w={width} h={height} len={data_len}")
    status_text.set_text(f"Recibiendo imagen id={image_id} ({width}x{height})")


def handle_img_chunk(line: str) -> None:
    global current_image, dropped_images, current_image_last_activity_mono, current_image_begin_mono
    m = CHUNK_RE.match(line)
    if not m:
        return

    image_id = int(m.group(1))
    chunk_index = int(m.group(2))
    n = int(m.group(3))
    hex_payload = m.group(4)

    if current_image is None:
        dbg(f"CHUNK_WITHOUT_IMAGE line={line}")
        return
    if image_id != current_image.image_id:
        dropped_images += 1
        dbg(f"DROP id_mismatch got={image_id} expected={current_image.image_id} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (id no coincide)")
        return
    if chunk_index != current_image.next_chunk:
        # Redundancy mode: tolerate duplicate retransmissions of the previous chunk.
        if current_image.next_chunk > 0 and chunk_index == (current_image.next_chunk - 1):
            dbg(f"DUP_CHUNK_IGNORED idx={chunk_index} expected={current_image.next_chunk} line={line}")
            current_image_last_activity_mono = time.monotonic()
            return
        dropped_images += 1
        dbg(f"DROP chunk_order got={chunk_index} expected={current_image.next_chunk} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (orden de bloques)")
        return
    if len(hex_payload) != n * 2:
        dropped_images += 1
        dbg(f"DROP chunk_size hex_len={len(hex_payload)} expected={n*2} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (tamano de bloque no coincide)")
        return

    try:
        chunk = bytes.fromhex(hex_payload)
    except ValueError:
        dropped_images += 1
        dbg(f"DROP invalid_hex line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (hex invalido)")
        return

    current_image.payload.extend(chunk)
    current_image.next_chunk += 1
    current_image_last_activity_mono = time.monotonic()


def handle_img_end(line: str) -> None:
    global current_image, current_image_last_activity_mono, current_image_begin_mono, last_image_time, last_image_latency_ms, saved_images, dropped_images
    m = END_RE.match(line)
    if not m:
        return

    image_id = int(m.group(1))
    ok = int(m.group(2))
    chunks = int(m.group(3))
    end_bytes = int(m.group(4))

    if current_image is None:
        dbg(f"END_WITHOUT_IMAGE line={line}")
        return

    payload = bytes(current_image.payload)
    expected_pixels = current_image.width * current_image.height
    expected_rgb565 = expected_pixels * 2
    valid = (
        image_id == current_image.image_id
        and ok == 1
        and chunks == current_image.next_chunk
        and end_bytes == current_image.data_len
        and len(payload) == current_image.data_len
        and (len(payload) == expected_pixels or len(payload) == expected_rgb565)
    )

    if not valid:
        dropped_images += 1
        dbg(
            "DROP end_validation "
            f"line={line} payload_len={len(payload)} expected={current_image.data_len} "
            f"chunks={current_image.next_chunk} expected_pixels={expected_pixels} expected_rgb565={expected_rgb565}"
        )
        status_text.set_text("Imagen descartada (validacion final)")
        current_image = None
        current_image_begin_mono = 0.0
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    is_rgb565 = len(payload) == expected_rgb565
    is_rgb = is_rgb565
    ext = "ppm" if is_rgb else "pgm"
    out_name = f"espnow_rx_{image_id:04d}_{current_image.width}x{current_image.height}_{ts}.{ext}"
    out_path = IMAGE_DIR / out_name
    if is_rgb565:
        rgb_payload = rgb565_to_rgb888(payload)
        save_ppm(out_path, current_image.width, current_image.height, rgb_payload)
        img_arr = np.frombuffer(rgb_payload, dtype=np.uint8).reshape((current_image.height, current_image.width, 3))
    else:
        save_pgm(out_path, current_image.width, current_image.height, payload)
        img_arr = np.frombuffer(payload, dtype=np.uint8).reshape((current_image.height, current_image.width))
        img_artist.set_cmap("gray")

    img_artist.set_data(img_arr)
    ax_img.set_aspect("equal")

    saved_images += 1
    last_image_time = time.monotonic()
    if current_image_begin_mono > 0.0:
        last_image_latency_ms = (last_image_time - current_image_begin_mono) * 1000.0
    current_image_last_activity_mono = 0.0
    dbg(f"END_OK line={line} file={out_name}")
    status_text.set_text(
        f"Imagen guardada id={image_id} bloques={chunks} latencia={last_image_latency_ms:.0f} ms\n{out_name}"
    )
    append_latency_markdown(image_id, current_image.width, current_image.height, chunks, last_image_latency_ms, out_name)
    current_image = None
    current_image_begin_mono = 0.0


def handle_img_begin_bin(image_id: int, width: int, height: int, data_len: int) -> None:
    global current_image, current_image_last_activity_mono, current_image_begin_mono, binary_image_mode_active

    current_image = ImageAssembly(
        image_id=image_id,
        width=width,
        height=height,
        data_len=data_len,
        next_chunk=0,
        payload=bytearray(),
    )
    current_image_last_activity_mono = time.monotonic()
    current_image_begin_mono = current_image_last_activity_mono
    binary_image_mode_active = True
    dbg(f"BEGIN id={image_id} w={width} h={height} len={data_len}")
    status_text.set_text(f"Recibiendo imagen id={image_id} ({width}x{height})")


def handle_img_chunk_bin(image_id: int, chunk_index: int, n: int, chunk: bytes) -> None:
    global current_image, dropped_images, current_image_last_activity_mono, current_image_begin_mono

    line = f"IMG_CHUNK {image_id} {chunk_index} {n} <bin:{len(chunk)}>"
    if current_image is None:
        dbg(f"CHUNK_WITHOUT_IMAGE line={line}")
        return
    if image_id != current_image.image_id:
        dropped_images += 1
        dbg(f"DROP id_mismatch got={image_id} expected={current_image.image_id} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (id no coincide)")
        return
    if chunk_index != current_image.next_chunk:
        # Redundancy mode: tolerate duplicate retransmissions of the previous chunk.
        if current_image.next_chunk > 0 and chunk_index == (current_image.next_chunk - 1):
            dbg(f"DUP_CHUNK_IGNORED idx={chunk_index} expected={current_image.next_chunk} line={line}")
            current_image_last_activity_mono = time.monotonic()
            return
        dropped_images += 1
        dbg(f"DROP chunk_order got={chunk_index} expected={current_image.next_chunk} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (orden de bloques)")
        return
    if len(chunk) != n:
        dropped_images += 1
        dbg(f"DROP chunk_size len={len(chunk)} expected={n} line={line}")
        current_image = None
        current_image_begin_mono = 0.0
        status_text.set_text("Imagen descartada (tamano de bloque no coincide)")
        return

    current_image.payload.extend(chunk)
    current_image.next_chunk += 1
    current_image_last_activity_mono = time.monotonic()


def handle_img_end_bin(image_id: int, ok: int, chunks: int, end_bytes: int) -> None:
    global current_image, current_image_last_activity_mono, current_image_begin_mono, last_image_time, last_image_latency_ms, saved_images, dropped_images, binary_image_mode_active

    line = f"IMG_END {image_id} {ok} {chunks} {end_bytes}"
    if current_image is None:
        binary_image_mode_active = False
        dbg(f"END_WITHOUT_IMAGE line={line}")
        return

    payload = bytes(current_image.payload)
    expected_pixels = current_image.width * current_image.height
    expected_rgb565 = expected_pixels * 2
    valid = (
        image_id == current_image.image_id
        and ok == 1
        and chunks == current_image.next_chunk
        and end_bytes == current_image.data_len
        and len(payload) == current_image.data_len
        and (len(payload) == expected_pixels or len(payload) == expected_rgb565)
    )

    if not valid:
        dropped_images += 1
        dbg(
            "DROP end_validation "
            f"line={line} payload_len={len(payload)} expected={current_image.data_len} "
            f"chunks={current_image.next_chunk} expected_pixels={expected_pixels} expected_rgb565={expected_rgb565}"
        )
        status_text.set_text("Imagen descartada (validacion final)")
        current_image = None
        current_image_begin_mono = 0.0
        binary_image_mode_active = False
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    is_rgb565 = len(payload) == expected_rgb565
    is_rgb = is_rgb565
    ext = "ppm" if is_rgb else "pgm"
    out_name = f"espnow_rx_{image_id:04d}_{current_image.width}x{current_image.height}_{ts}.{ext}"
    out_path = IMAGE_DIR / out_name
    if is_rgb565:
        rgb_payload = rgb565_to_rgb888(payload)
        save_ppm(out_path, current_image.width, current_image.height, rgb_payload)
        img_arr = np.frombuffer(rgb_payload, dtype=np.uint8).reshape((current_image.height, current_image.width, 3))
    else:
        save_pgm(out_path, current_image.width, current_image.height, payload)
        img_arr = np.frombuffer(payload, dtype=np.uint8).reshape((current_image.height, current_image.width))
        img_artist.set_cmap("gray")

    img_artist.set_data(img_arr)
    ax_img.set_aspect("equal")

    saved_images += 1
    last_image_time = time.monotonic()
    if current_image_begin_mono > 0.0:
        last_image_latency_ms = (last_image_time - current_image_begin_mono) * 1000.0
    current_image_last_activity_mono = 0.0
    dbg(f"END_OK line={line} file={out_name}")
    status_text.set_text(
        f"Imagen guardada id={image_id} bloques={chunks} latencia={last_image_latency_ms:.0f} ms\n{out_name}"
    )
    append_latency_markdown(image_id, current_image.width, current_image.height, chunks, last_image_latency_ms, out_name)
    current_image = None
    current_image_begin_mono = 0.0
    binary_image_mode_active = False


def process_binary_image_frame(frame_type: int, payload: bytes) -> None:
    if frame_type == SERIAL_IMG_FRAME_BEGIN:
        if len(payload) != 10:
            dbg(f"MALFORMED_BEGIN_BIN len={len(payload)}")
            return
        image_id = int.from_bytes(payload[0:2], "little")
        width = int.from_bytes(payload[2:4], "little")
        height = int.from_bytes(payload[4:6], "little")
        data_len = int.from_bytes(payload[6:10], "little")
        handle_img_begin_bin(image_id, width, height, data_len)
        return

    if frame_type == SERIAL_IMG_FRAME_CHUNK:
        if len(payload) < 6:
            dbg(f"MALFORMED_CHUNK_BIN len={len(payload)}")
            return
        image_id = int.from_bytes(payload[0:2], "little")
        chunk_index = int.from_bytes(payload[2:4], "little")
        n = int.from_bytes(payload[4:6], "little")
        chunk = payload[6:]
        handle_img_chunk_bin(image_id, chunk_index, n, chunk)
        return

    if frame_type == SERIAL_IMG_FRAME_END:
        if len(payload) != 9:
            dbg(f"MALFORMED_END_BIN len={len(payload)}")
            return
        image_id = int.from_bytes(payload[0:2], "little")
        ok = payload[2]
        chunks = int.from_bytes(payload[3:5], "little")
        end_bytes = int.from_bytes(payload[5:9], "little")
        handle_img_end_bin(image_id, ok, chunks, end_bytes)
        return

    dbg(f"UNKNOWN_IMG_FRAME type={frame_type} len={len(payload)}")


def process_text_line(line: str) -> bool:
    sensor_vals = parse_sensor_csv(line)
    if sensor_vals is not None:
        now = datetime.now()
        ts_full = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        ts_short = now.strftime("%H:%M:%S")

        sensor_writer.writerow([ts_full] + sensor_vals)

        time_buffer.append(ts_short)
        for i in range(NUM_VARS):
            # Column 3 in the UI is repurposed for image latency in field monitoring.
            data_buffers[i].append(last_image_latency_ms if i == 3 else sensor_vals[i])

        global last_sensor_time, last_sensor_timestamp
        last_sensor_time = time.monotonic()
        last_sensor_timestamp = ts_short
        return True

    for frag in split_protocol_fragments(line):
        process_non_sensor_line(frag)
    return False


def is_printable_text_bytes(data: bytes) -> bool:
    """Heuristic guard to avoid treating binary payload as a text line."""
    if not data:
        return False
    for b in data:
        if b in (9, 10, 13):
            continue
        if b < 32 or b > 126:
            return False
    return True


def process_serial_buffer() -> bool:
    """Parse mixed CSV text lines and binary image frames from the RX buffer."""
    global serial_rx_buffer, serial_parse_buffer, binary_image_mode_active

    with serial_rx_lock:
        if serial_rx_buffer:
            serial_parse_buffer.extend(serial_rx_buffer)
            serial_rx_buffer.clear()

    sensor_updated = False
    processed_events = 0
    max_events = 12000

    while processed_events < max_events:
        sof_idx = serial_parse_buffer.find(SERIAL_IMG_SOF)
        nl_idx = serial_parse_buffer.find(b"\n")

        if sof_idx < 0:
            if binary_image_mode_active:
                # While receiving a binary image stream, do not interpret bytes
                # as text; wait for more binary frame data.
                break
            if nl_idx < 0:
                break
            raw = bytes(serial_parse_buffer[: nl_idx + 1])
            del serial_parse_buffer[: nl_idx + 1]
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                log_raw_serial_line(line)
                sensor_updated = process_text_line(line) or sensor_updated
            processed_events += 1
            continue

        if nl_idx >= 0 and nl_idx < sof_idx:
            if binary_image_mode_active:
                # In binary mode, prefer frame sync and skip any apparent text
                # before SOF to avoid consuming frame bytes accidentally.
                if sof_idx > 0:
                    junk = bytes(serial_parse_buffer[:sof_idx])
                    if junk:
                        dbg(f"RX_JUNK_BEFORE_SOF len={len(junk)}")
                    del serial_parse_buffer[:sof_idx]
                    processed_events += 1
                    continue
            # There is text before the next frame; only consume it if it is
            # truly printable text, otherwise keep searching for SOF.
            raw = bytes(serial_parse_buffer[: nl_idx + 1])
            if is_printable_text_bytes(raw):
                del serial_parse_buffer[: nl_idx + 1]
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    log_raw_serial_line(line)
                    sensor_updated = process_text_line(line) or sensor_updated
                processed_events += 1
                continue
            # Binary-looking prefix: drop directly to SOF resync path below.

        if sof_idx > 0:
            # Drop non-frame bytes before SOF to realign quickly on binary frames.
            junk = bytes(serial_parse_buffer[:sof_idx])
            if junk and not is_printable_text_bytes(junk):
                dbg(f"RX_JUNK_BEFORE_SOF len={len(junk)}")
            del serial_parse_buffer[:sof_idx]
            processed_events += 1
            continue

        if len(serial_parse_buffer) < 7:
            break

        frame_type = serial_parse_buffer[2]
        payload_len = int(serial_parse_buffer[3]) | (int(serial_parse_buffer[4]) << 8)
        if payload_len > SERIAL_IMG_MAX_PAYLOAD:
            dbg(f"FRAME_LEN_TOO_BIG len={payload_len}")
            del serial_parse_buffer[0]
            processed_events += 1
            continue

        total = 7 + payload_len
        if len(serial_parse_buffer) < total:
            break

        payload = bytes(serial_parse_buffer[5 : 5 + payload_len])
        rx_crc = int(serial_parse_buffer[5 + payload_len]) | (int(serial_parse_buffer[6 + payload_len]) << 8)
        crc_data = bytes(serial_parse_buffer[2 : 5 + payload_len])
        calc_crc = crc16_ccitt(crc_data)

        if calc_crc != rx_crc:
            dbg(f"FRAME_CRC_FAIL type={frame_type} len={payload_len} rx={rx_crc} calc={calc_crc}")
            del serial_parse_buffer[0]
            processed_events += 1
            continue

        process_binary_image_frame(frame_type, payload)
        del serial_parse_buffer[:total]
        processed_events += 1

    return sensor_updated


def process_non_sensor_line(line: str) -> None:
    global latest_relay_status

    if line.startswith("IMG_BEGIN"):
        if BEGIN_RE.match(line):
            handle_img_begin(line)
        else:
            dbg(f"MALFORMED_BEGIN line={line}")
        return
    if line.startswith("IMG_CHUNK"):
        if CHUNK_RE.match(line):
            handle_img_chunk(line)
        else:
            dbg(f"MALFORMED_CHUNK line={line}")
        return
    if line.startswith("IMG_END"):
        if END_RE.match(line):
            handle_img_end(line)
        else:
            dbg(f"MALFORMED_END line={line}")
        return

    if line.startswith("IMG_DBG"):
        print(line)
        dbg(f"RXDBG {line}")
        return

    if line.startswith("RELAY_STATUS"):
        print(line)
        dbg(f"RXSTAT {line}")
        latest_relay_status = line
        return

    if line.startswith("RECEIVER_") or line.startswith("SENSOR_MAC") or line.startswith("IMAGE_MAC"):
        print(line)
        dbg(f"RXSTAT {line}")


def split_protocol_fragments(line: str) -> list[str]:
    """Split a raw serial line when multiple protocol records were concatenated."""
    starts = [m.start() for m in PROTOCOL_TOKEN_RE.finditer(line)]
    if not starts:
        return [line]

    parts: list[str] = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if (i + 1) < len(starts) else len(line)
        frag = line[start:end].strip()
        if frag:
            parts.append(frag)
    return parts if parts else [line]


def update_plot(_frame: int):
    global current_image, current_image_last_activity_mono, current_image_begin_mono, last_sensor_time, last_file_flush_mono, dropped_images

    sensor_updated = process_serial_buffer()

    if sensor_updated:
        for i in range(NUM_VARS):
            yvals = list(data_buffers[i])
            lines[i].set_ydata(yvals)
            axes[i].relim()
            axes[i].autoscale_view()

            val = yvals[-1]
            unit = UNITS[i]
            if unit == "#":
                value_texts[i].set_text(f"{int(val):d} {unit}")
            elif abs(val) >= 100:
                value_texts[i].set_text(f"{val:.1f} {unit}")
            else:
                value_texts[i].set_text(f"{val:.2f} {unit}")

    now_mono = time.monotonic()
    # Flushing every frame can stall ingestion and cause serial buffer overrun at high rates.
    if now_mono - last_file_flush_mono >= 0.5:
        sensor_log_file.flush()
        debug_log_file.flush()
        raw_serial_log_file.flush()
        last_file_flush_mono = now_mono

    if current_image is not None and current_image_last_activity_mono > 0.0:
        idle = now_mono - current_image_last_activity_mono
        if idle > IMAGE_ASSEMBLY_TIMEOUT_SEC:
            dropped_images += 1
            dbg(
                "DROP image_timeout "
                f"id={current_image.image_id} next_chunk={current_image.next_chunk} "
                f"bytes={len(current_image.payload)}/{current_image.data_len} "
                f"idle_s={idle:.2f} timeout_s={IMAGE_ASSEMBLY_TIMEOUT_SEC:.2f}"
            )
            status_text.set_text(
                f"Imagen descartada (timeout de recepcion {idle:.1f}s, bloques={current_image.next_chunk})"
            )
            current_image = None
            current_image_last_activity_mono = 0.0
            current_image_begin_mono = 0.0

    # Keep the area under the image frame clean even if previous interactive
    # runs left a stale xlabel.
    ax_img.set_xlabel("")
    ax_img.xaxis.label.set_visible(False)

    if last_sensor_time == 0.0:
        sensor_state = "esperando"
        sensor_color = "#6b7280"
    else:
        age = now_mono - last_sensor_time
        if age > SENSOR_TIMEOUT_SEC:
            sensor_state = f"DETENIDO ({age:.1f}s)"
            sensor_color = "#b91c1c"
        else:
            sensor_state = f"LEYENDO ({age:.2f}s)"
            sensor_color = "#166534"

    if last_image_time == 0.0:
        image_age_text = "--"
    else:
        img_age = now_mono - last_image_time
        image_age_text = f"hace {img_age:.1f}s"

    system_card_text.set_text(
        "Sistema\n"
        f"Hora: {last_sensor_timestamp}\n"
        f"Sensores: {sensor_state}"
    )
    system_card_text.set_color(sensor_color)

    anaglyph_card_text.set_text(
        "Anaglifo\n"
        f"Guardadas: {saved_images}\n"
        f"Descartadas: {dropped_images}\n"
        f"Ultima imagen: {image_age_text}"
    )
    anaglyph_card_text.set_color("#111111")

    return lines + value_texts + [img_artist, system_card_text, anaglyph_card_text, status_text]


# %%
plt.ion()
start_serial_reader()

def _safe_update_plot(frame: int):
    try:
        return update_plot(frame)
    except Exception as exc:
        try:
            dbg(f"DASHBOARD_UPDATE_ERROR err={exc}")
            status_text.set_text(f"Error de actualizacion: {exc}")
        except Exception:
            pass
        return lines + value_texts + [img_artist, system_card_text, anaglyph_card_text, status_text]

ani = animation.FuncAnimation(
    fig,
    _safe_update_plot,
    interval=50,
    blit=False,
    cache_frame_data=False,
)

# Keep handles so subsequent runs can close/replace cleanly.
_dashboard_fig = fig
_dashboard_ani = ani
_dashboard_is_running = True

try:
    ani.event_source.start()
except Exception:
    pass


def stop_dashboard():
    """Stop the dashboard cleanly. Call this from the interactive window prompt."""
    try:
        prev_ani = globals().get("_dashboard_ani")
        if prev_ani is not None:
            prev_ani.event_source.stop()
    except Exception:
        pass

    try:
        prev_timer = globals().get("_dashboard_timer")
        if prev_timer is not None:
            prev_timer.stop()
    except Exception:
        pass

    try:
        stop_serial_reader()
    except Exception:
        pass

    try:
        sensor_log_file.flush()
        sensor_log_file.close()
    except Exception:
        pass

    try:
        debug_log_file.flush()
        debug_log_file.close()
    except Exception:
        pass

    try:
        raw_serial_log_file.flush()
        raw_serial_log_file.close()
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass

    globals()["_dashboard_is_running"] = False
    print("✓ Dashboard detenido.")