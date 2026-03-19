# %% [markdown]
# # XIAO ESP32-S3 Secure Telemetry Dashboard
# Synchronized Y-axes, Timestamped X-axis, and Crash-proof CSV Logging.

# %%
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
from datetime import datetime
import os
import math

# --- INTERACTIVE BACKEND ---
# This is required for plots to render inside VS Code Interactive Window
%matplotlib widget

# --- CONFIGURATION ---
LABELS = [
    "Packet #",
    "Distance",
    "RSSI",
    "Smoothed RSSI",
    "Loss Rate",
    "Throughput",
    "Temp BME280",
    "Temp DS18B20",
    "Pressure",
    "Altitude",
    "Velocity X",
    "Velocity Y",
    "Velocity Z",
    "Accel X",
    "Accel Y",
    "Accel Z",
]
UNITS = [
    "#",
    "m",
    "dBm",
    "dBm",
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
NUM_COLS = 2
NUM_ROWS = math.ceil(NUM_VARS / NUM_COLS)
MAX_POINTS = 50
LOG_FILE = "../data/esp32_telemetry_secure.csv"

# Ensure the data directory exists
os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

def find_esp32_port():
    """Auto-detects ESP32/XIAO on Windows, Mac, or Linux."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Detect common USB-Serial device identifiers
        if any(key in port.description for key in ["CP210", "USB", "UART", "JTAG", "ACM"]):
            print(f"✅ Found ESP32 on {port.device}")
            return port.device
    return None

# --- INITIALIZATION ---
port_name = find_esp32_port()
if not port_name:
    raise Exception("❌ ESP32 not found. Check cable and Serial Monitor.")

ser = serial.Serial(port_name, 115200, timeout=0.01)

# Data buffers for plotting
time_buffer = deque([""]*MAX_POINTS, maxlen=MAX_POINTS)
data_buffers = [deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS) for _ in range(NUM_VARS)]

# Setup CSV Header
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp"] + LABELS)

# --- UI SETUP ---
# sharex=True ensures all graphs stay aligned in time
fig, axs = plt.subplots(NUM_ROWS, NUM_COLS, figsize=(14, 2.1 * NUM_ROWS), sharex=True)
plt.subplots_adjust(hspace=0.4, bottom=0.15)
axs_flat = axs.flatten()
lines = []

for i, ax in enumerate(axs_flat[:NUM_VARS]):
    line, = ax.plot(range(MAX_POINTS), data_buffers[i], label=LABELS[i], color=f"C{i}")
    lines.append(line)
    ax.set_ylabel(UNITS[i], fontsize='small')
    ax.legend(loc="upper left", fontsize='xx-small')
    ax.grid(True, alpha=0.2)

# Hide any unused axes when the grid has extra cells
for ax in axs_flat[NUM_VARS:]:
    ax.set_visible(False)

# Set X-labels on the bottom row only
bottom_row_start = (NUM_ROWS - 1) * NUM_COLS
for idx in range(bottom_row_start, min(bottom_row_start + NUM_COLS, NUM_VARS)):
    axs_flat[idx].set_xlabel("Timestamp")


def parse_csv_line(line_data):
    """Parse a CSV line from receiver.ino; return float list or None if invalid/header."""
    values = line_data.split(',')
    if len(values) != NUM_VARS:
        return None
    try:
        return [float(v) for v in values]
    except ValueError:
        # Header or status text line
        return None

# --- ANIMATION UPDATE LOGIC ---
def update_plot(frame):
    if ser.in_waiting:
        try:
            line_data = ser.readline().decode('utf-8').strip()
            float_vals = parse_csv_line(line_data)

            if float_vals is not None:
                now = datetime.now()
                ts_full = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                ts_short = now.strftime("%H:%M:%S")

                # 1. SECURE CSV WRITE (Immediate Flush to Disk)
                with open(LOG_FILE, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([ts_full] + float_vals)
                    f.flush()
                    os.fsync(f.fileno()) # Forces OS to physical write

                # 2. UPDATE PLOT BUFFERS
                time_buffer.append(ts_short)
                for i in range(NUM_VARS):
                    data_buffers[i].append(float_vals[i])
                    lines[i].set_ydata(list(data_buffers[i]))
                    
                    # Auto-scale each Y-axis independently
                    axs_flat[i].relim()
                    axs_flat[i].autoscale_view()

                # 3. UPDATE X-AXIS TICKS
                indices = range(0, MAX_POINTS, 10)
                for idx in range(bottom_row_start, min(bottom_row_start + NUM_COLS, NUM_VARS)):
                    axs_flat[idx].set_xticks(indices)
                    axs_flat[idx].set_xticklabels(
                        [time_buffer[j] for j in indices],
                        rotation=45,
                        fontsize='x-small'
                    )
        except Exception:
            pass
    return lines

# --- RUN ANIMATION ---
# FuncAnimation is the non-blocking way to update graphs in Jupyter/VS Code
print(f"🚀 Streaming data to {LOG_FILE}...")
ani = animation.FuncAnimation(
    fig, 
    update_plot, 
    interval=50, 
    blit=False, 
    cache_frame_data=False
)

plt.show()