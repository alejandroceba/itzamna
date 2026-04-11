# ESP-NOW Image Relay Reproducibility Notes (2026-04-11)

## Goal
Stabilize image transfer from DER camera through sender relay to receiver/dashboard while keeping telemetry active.

## Hardware/Link Topology
- DER camera node running `DER_AG_T/DER_AG_T.ino`
- Relay + sensors node running `sender/sender.ino`
- Receiver node running `receiver/receiver.ino`
- Dashboard on host running `scripts/dashboard.py`

## Verified Radio Setup
- WiFi channel: `6`
- DER MAC: `10:20:ba:03:99:9c`
- Sender MAC: `34:85:18:8b:8a:34`
- Receiver MAC (configured in sender): `d8:3b:da:45:cd:24`
- Dashboard serial port used in tests: `/dev/ttyACM2`

## Changes Applied

### 1) DER image payload format and pacing
File: `DER_AG_T/DER_AG_T.ino`

- Active UART-integrated image path sends **RGB888 anaglyph payload**:
  - `dataLen = width * height * 3`
  - per-pixel payload bytes: `R=leftGray`, `G=rightGray`, `B=rightGray`
- DER ESP-NOW inter-packet delay increased:
  - `IMAGE_SEND_GAP_MS: 10 -> 15`

Reason:
- RGB888 aligns with existing sender/receiver/dashboard expectations in this integration stage.
- Extra DER pacing reduced TX queue pressure (`ESP_ERR_ESPNOW_NO_MEM` / 12391).

### 2) Sender relay throughput
File: `sender/sender.ino`

- Fast-mode relay forwarding increased:
  - `IMG_FORWARD_PACKETS_PER_LOOP_FAST: 4 -> 8`
  - `IMG_FORWARD_MIN_GAP_MS_FAST: 3 -> 0`

Reason:
- Relay queue reached saturation during image bursts; increasing dequeue/forward budget helps drain image packets faster.

## Observed Metrics During Tuning

### Earlier unstable runs
- DER reported `esp_now_send error: 12391` and aborted image send.
- Sender relay status showed queue pressure and drops:
  - `queued` reached max range (`~19-21`)
  - `qdrops` increased continuously

### After DER pacing + format alignment
- DER completed image sends without 12391 in shown runs:
  - Example: `Anaglifo enviado por ESP-NOW, imageId=5`
  - Example: `Tiempo total: 19919 ms (19.92 s)`
- Sender relay still required aggressive forwarding to avoid queue growth under burst load.

## Repro Procedure
1. Flash firmware:
   - DER with `DER_AG_T/DER_AG_T.ino`
   - Sender with `sender/sender.ino`
   - Receiver with `receiver/receiver.ino`
2. Verify MAC/channel values match the list above.
3. Start dashboard (`scripts/dashboard.py`) on receiver serial port.
4. Trigger stereo capture.
5. Monitor serial outputs:
   - DER: should show `Header recibido` then `Anaglifo enviado por ESP-NOW`
   - Sender: watch `RELAY_STATUS`
   - Receiver: look for `IMG_BEGIN/IMG_CHUNK/IMG_END` or `RECEIVER_IMG_DROP` diagnostics

## Acceptance Criteria
- DER has no `esp_now_send error: 12391` during image upload.
- Sender `qdrops` remains near zero during image burst.
- Receiver reconstructs full image sequences (dashboard image count increments).

## Notes
- `ESP_ERR_ESPNOW_NO_MEM` (12391) is queue pressure in the ESP-NOW send path, not a PSRAM requirement issue.
- PSRAM is helpful for frame buffering, but does not directly fix radio TX queue saturation.
