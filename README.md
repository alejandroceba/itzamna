# itzamna
Itzamná IER-UNAM

3 XIAOs, mandan por separado ESP-NOW 2 esclavos, trigger foto pin digital paralelo

agregar alineamiento, rafaga

18 dBm

## Recent receiver/dashboard updates (2026-04-08)

- Receiver image debug output uses text protocol (`IMG_BEGIN`, `IMG_CHUNK`, `IMG_END`) so serial logs are readable and dashboard-compatible.
- Receiver sensor telemetry print no longer gets reduced during image emission; sensor CSV output stays at the normal `SENSOR_PRINT_DIVIDER` rate.
- Receiver is currently in image reconstruction test mode with `IMG_SERIAL_CHUNK_LINE_STRIDE = 1` (all chunk lines emitted).
- Dashboard now writes full timestamped raw receiver serial lines to `data/receiver_serial_raw.log`.
- Dashboard still writes protocol/debug events to `data/image_debug.log` and sensor CSV data to `data/telemetry_merged.csv`.

### Debug workflow (no long copy/paste needed)

1. Run `scripts/dashboard.py` and reproduce the issue.
2. Share files from `data/` instead of terminal copy/paste:
	- `receiver_serial_raw.log`
	- `image_debug.log`
	- `telemetry_merged.csv` (optional, for sensor correlation)

### Notes

- For low-log operation after reconstruction tests, increase `IMG_SERIAL_CHUNK_LINE_STRIDE` in `receiver/receiver.ino`.
- Keep `IMG_SERIAL_CHUNK_LINE_STRIDE = 1` when testing full text-mode image reconstruction in `scripts/dashboard.py`.