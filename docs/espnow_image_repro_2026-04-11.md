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

## Image Latency Log

| Timestamp | Image ID | Resolution | Chunks | Latency (ms) | File |
|---|---:|---|---:|---:|---|
| 2026-04-11 10:58:26 | 9 | 320x240 | 3600 | 25017 | espnow_rx_0009_320x240_20260411_105826.ppm |
| 2026-04-11 10:59:23 | 10 | 320x240 | 3600 | 25310 | espnow_rx_0010_320x240_20260411_105923.ppm |
| 2026-04-11 11:25:02 | 14 | 320x240 | 2400 | 30336 | espnow_rx_0014_320x240_20260411_112502.ppm |
| 2026-04-11 11:25:48 | 15 | 320x240 | 2400 | 31080 | espnow_rx_0015_320x240_20260411_112548.ppm |
| 2026-04-11 11:26:27 | 16 | 320x240 | 2400 | 30535 | espnow_rx_0016_320x240_20260411_112627.ppm |
| 2026-04-11 11:27:06 | 17 | 320x240 | 2400 | 30730 | espnow_rx_0017_320x240_20260411_112706.ppm |

## Latency Reduction Micro-Plan (One Change at a Time)

### Current Baseline From Log
- End-to-end latency is currently in the ~25-31 s range.
- Representative entries:
  - 25017 ms (`id=9`)
  - 25310 ms (`id=10`)
  - 30336-31080 ms (`id=14..17`)

### Step A (Applied)
- File: `receiver/receiver.ino`
- Changes:
  - `IMG_SERIAL_CHUNK_BYTES: 64 -> 96`
  - `IMG_SERIAL_CHUNK_INTERVAL_MS: 6 -> 4`
- Rationale:
  - Reduce host serial overhead while keeping conservative pacing.

### Test Protocol For Each Step
1. Flash only the changed node(s).
2. Capture 5 images under the same physical setup.
3. Record for each image:
   - DER `Tiempo total` (radio + processing time)
   - Dashboard/image log latency (ms)
   - Sender `RELAY_STATUS` (`queued`, `qdrops`, `dropped`, `invalid`)
   - Any `esp_now_send error: 12391`
4. Compare median and worst-case latency against the previous step.
5. Track end-to-end total latency as `DER Tiempo total + receiver image latency`.

### Pass/Fail Criteria Per Step
- Pass:
  - Median latency improves by >= 10% versus previous step.
  - `qdrops` and `dropped` remain `0` during the run.
  - No new sustained 12391 bursts.
- Fail:
  - Latency does not improve or reliability regresses.

### Next Planned Small Steps
- Step I: `IMAGE_SEND_GAP_MS: 6 -> 4` (DER) — ✅ **ACCEPTED** (13.3% improvement, 1 valid image id=9: 12.126s receiver latency, 19.2s total vs 22.1s baseline)
- Step J: `IMAGE_SEND_GAP_MS: 4 -> 2` (DER) — ❌ **FAILED** (median gain too small and reliability regressed)
- Step K: `320x240 -> 160x120` on both cameras (DER + IZQ) — ✅ **ACCEPTED**
- Current best operating point: `IMAGE_SEND_GAP_MS = 4` and `QQVGA (160x120)` on both cameras.

### Step Test Results Log
| Step | Change | Images tested | Median latency (ms) | Worst latency (ms) | qdrops | dropped | 12391 events | Verdict |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| A | Receiver chunk `64->96`, interval `6->4` | 5 (id 18..22) | 21191 | 23270 | — | — | 0 in this 5-image run | — |
| B | Receiver interval `4->2` | 5 (id 23..27) | 18640 | 19104 | — | — | 0 in this 5-image run | — |
| C | DER gap `15->12` | 5 complete (id 2,3,4,6,7) | 16046 | 16861 | — | — | 0 in this 5-image run (one header timeout observed separately) | — |
| D | Sender fast packets `8->10` | tested, no combined end-to-end win | _pending_ | _pending_ | — | — | _pending_ | no |
| E | DER gap `12->10` | 5 complete (id 2..6) | 15024 | 15425 | — | — | intermittent | latency gain, reliability regress |
| F | Sender fast packets `10->9` | 5 complete (id 2..6) | 15329 | 15445 | — | — | 0 in this 5-image run | no |
| G | DER gap `10->8` | 5 complete (id 2..6) | 13513 | 14295 | — | — | 0 in this 5-image run | pass |
| H | DER gap `8->6` | 6 complete (id 2..7) | 13557 | 14295 | 22146 / 22884 | 0 | 0 in this 5-image run | pass |
| I | DER gap `6->4` | 1 partial (id 9, receiver disconnected after) | 12126 | 12126 | 0 | 0 | 0 | **pass (13.3% improvement)** ⚠️ receiver USB lost mid-test |
| J | DER gap `4->2` | 5 complete (id 8..12) | 11852 | 11989 | 0 | 16 | 0 observed | **fail** (only ~1.6% total gain vs Step I; reliability regressed) |
| K | Resolution `320x240->160x120` (DER + IZQ), DER gap held at `4` | 5 complete (id 2..6) | 2673 | 2818 | — | — | 0 observed | **pass** (median total 4058 ms with DER=1385 ms; ~78.9% lower vs 19.2 s baseline) |
| 2026-04-11 11:34:58 | 18 | 320x240 | 1600 | 22603 | espnow_rx_0018_320x240_20260411_113458.ppm |
| 2026-04-11 11:35:28 | 19 | 320x240 | 1600 | 20979 | espnow_rx_0019_320x240_20260411_113528.ppm |
| 2026-04-11 11:35:58 | 20 | 320x240 | 1600 | 21200 | espnow_rx_0020_320x240_20260411_113558.ppm |
| 2026-04-11 11:36:34 | 21 | 320x240 | 1600 | 23270 | espnow_rx_0021_320x240_20260411_113634.ppm |
| 2026-04-11 11:37:31 | 22 | 320x240 | 1600 | 21191 | espnow_rx_0022_320x240_20260411_113731.ppm |
| 2026-04-11 11:48:47 | 23 | 320x240 | 1600 | 19104 | espnow_rx_0023_320x240_20260411_114847.ppm |
| 2026-04-11 11:49:19 | 24 | 320x240 | 1600 | 18790 | espnow_rx_0024_320x240_20260411_114919.ppm |
| 2026-04-11 11:49:49 | 25 | 320x240 | 1600 | 18603 | espnow_rx_0025_320x240_20260411_114949.ppm |
| 2026-04-11 11:50:42 | 26 | 320x240 | 1600 | 18640 | espnow_rx_0026_320x240_20260411_115042.ppm |
| 2026-04-11 11:51:10 | 27 | 320x240 | 1600 | 18486 | espnow_rx_0027_320x240_20260411_115110.ppm |
| 2026-04-11 11:58:38 | 2 | 320x240 | 1600 | 15243 | espnow_rx_0002_320x240_20260411_115838.ppm |
| 2026-04-11 11:59:04 | 3 | 320x240 | 1600 | 16861 | espnow_rx_0003_320x240_20260411_115904.ppm |
| 2026-04-11 11:59:30 | 4 | 320x240 | 1600 | 15897 | espnow_rx_0004_320x240_20260411_115930.ppm |
| 2026-04-11 12:00:26 | 6 | 320x240 | 1600 | 16467 | espnow_rx_0006_320x240_20260411_120026.ppm |
| 2026-04-11 12:00:48 | 7 | 320x240 | 1600 | 16046 | espnow_rx_0007_320x240_20260411_120048.ppm |
| 2026-04-11 12:08:00 | 8 | 320x240 | 1600 | 16147 | espnow_rx_0008_320x240_20260411_120800.ppm |
| 2026-04-11 12:08:25 | 9 | 320x240 | 1600 | 16216 | espnow_rx_0009_320x240_20260411_120825.ppm |
| 2026-04-11 12:09:03 | 10 | 320x240 | 1600 | 16737 | espnow_rx_0010_320x240_20260411_120903.ppm |
| 2026-04-11 12:09:30 | 11 | 320x240 | 1600 | 16264 | espnow_rx_0011_320x240_20260411_120930.ppm |
| 2026-04-11 12:09:48 | 12 | 320x240 | 1600 | 16595 | espnow_rx_0012_320x240_20260411_120948.ppm |
| 2026-04-11 12:10:17 | 13 | 320x240 | 1600 | 16330 | espnow_rx_0013_320x240_20260411_121017.ppm |
| 2026-04-11 14:29:12 | 4 | 320x240 | 1600 | 16583 | espnow_rx_0004_320x240_20260411_142912.ppm |
| 2026-04-11 14:32:47 | 2 | 320x240 | 1600 | 15024 | espnow_rx_0002_320x240_20260411_143247.ppm |
| 2026-04-11 14:33:16 | 3 | 320x240 | 1600 | 14405 | espnow_rx_0003_320x240_20260411_143316.ppm |
| 2026-04-11 14:33:41 | 4 | 320x240 | 1600 | 14567 | espnow_rx_0004_320x240_20260411_143341.ppm |
| 2026-04-11 14:34:04 | 5 | 320x240 | 1600 | 15425 | espnow_rx_0005_320x240_20260411_143404.ppm |
| 2026-04-11 14:34:24 | 6 | 320x240 | 1600 | 15318 | espnow_rx_0006_320x240_20260411_143424.ppm |
| 2026-04-11 14:54:47 | 2 | 320x240 | 1600 | 15028 | espnow_rx_0002_320x240_20260411_145447.ppm |
| 2026-04-11 14:55:21 | 3 | 320x240 | 1600 | 15329 | espnow_rx_0003_320x240_20260411_145521.ppm |
| 2026-04-11 14:55:44 | 4 | 320x240 | 1600 | 14317 | espnow_rx_0004_320x240_20260411_145544.ppm |
| 2026-04-11 14:56:07 | 5 | 320x240 | 1600 | 15445 | espnow_rx_0005_320x240_20260411_145607.ppm |
| 2026-04-11 14:56:32 | 6 | 320x240 | 1600 | 15351 | espnow_rx_0006_320x240_20260411_145632.ppm |
| 2026-04-11 15:02:42 | 2 | 320x240 | 1600 | 14295 | espnow_rx_0002_320x240_20260411_150242.ppm |
| 2026-04-11 15:03:03 | 3 | 320x240 | 1600 | 13630 | espnow_rx_0003_320x240_20260411_150303.ppm |
| 2026-04-11 15:03:34 | 4 | 320x240 | 1600 | 13600 | espnow_rx_0004_320x240_20260411_150334.ppm |
| 2026-04-11 15:03:55 | 5 | 320x240 | 1600 | 13470 | espnow_rx_0005_320x240_20260411_150355.ppm |
| 2026-04-11 15:04:16 | 6 | 320x240 | 1600 | 13513 | espnow_rx_0006_320x240_20260411_150416.ppm |
| 2026-04-11 15:04:40 | 7 | 320x240 | 1600 | 12555 | espnow_rx_0007_320x240_20260411_150440.ppm |
| 2026-04-11 15:11:31 | 2 | 320x240 | 1600 | 11831 | espnow_rx_0002_320x240_20260411_151131.ppm |
| 2026-04-11 15:11:50 | 3 | 320x240 | 1600 | 11719 | espnow_rx_0003_320x240_20260411_151150.ppm |
| 2026-04-11 15:12:09 | 4 | 320x240 | 1600 | 12019 | espnow_rx_0004_320x240_20260411_151209.ppm |
| 2026-04-11 15:12:35 | 5 | 320x240 | 1600 | 10918 | espnow_rx_0005_320x240_20260411_151235.ppm |
| 2026-04-11 15:12:54 | 6 | 320x240 | 1600 | 12147 | espnow_rx_0006_320x240_20260411_151254.ppm |
| 2026-04-11 15:13:12 | 7 | 320x240 | 1600 | 12144 | espnow_rx_0007_320x240_20260411_151312.ppm |
| 2026-04-11 15:13:29 | 8 | 320x240 | 1600 | 12420 | espnow_rx_0008_320x240_20260411_151329.ppm |
| 2026-04-11 15:22:53 | 9 | 320x240 | 1600 | 12127 | espnow_rx_0009_320x240_20260411_152253.ppm |
| 2026-04-11 15:24:27 | 10 | 320x240 | 1600 | 11816 | espnow_rx_0010_320x240_20260411_152427.ppm |
| 2026-04-11 15:35:36 | 15 | 320x240 | 1600 | 11973 | espnow_rx_0015_320x240_20260411_153536.ppm |
| 2026-04-11 15:37:08 | 16 | 320x240 | 1600 | 10897 | espnow_rx_0016_320x240_20260411_153708.ppm |
| 2026-04-11 15:38:58 | 17 | 320x240 | 1600 | 12430 | espnow_rx_0017_320x240_20260411_153858.ppm |
| 2026-04-11 15:40:19 | 18 | 320x240 | 1600 | 11855 | espnow_rx_0018_320x240_20260411_154019.ppm |
| 2026-04-11 15:41:38 | 19 | 320x240 | 1600 | 10664 | espnow_rx_0019_320x240_20260411_154138.ppm |
| 2026-04-11 15:42:32 | 20 | 320x240 | 1600 | 11769 | espnow_rx_0020_320x240_20260411_154232.ppm |
| 2026-04-11 15:44:12 | 21 | 320x240 | 1600 | 11230 | espnow_rx_0021_320x240_20260411_154412.ppm |
| 2026-04-11 15:51:04 | 2 | 320x240 | 1600 | 11352 | espnow_rx_0002_320x240_20260411_155104.ppm |
| 2026-04-11 15:53:43 | 3 | 320x240 | 1600 | 12126 | espnow_rx_0003_320x240_20260411_155343.ppm |
| 2026-04-11 15:58:35 | 4 | 320x240 | 1600 | 11410 | espnow_rx_0004_320x240_20260411_155835.ppm |
| 2026-04-11 16:00:48 | 5 | 320x240 | 1600 | 11633 | espnow_rx_0005_320x240_20260411_160048.ppm |
| 2026-04-11 16:02:06 | 8 | 320x240 | 1600 | 11989 | espnow_rx_0008_320x240_20260411_160206.ppm |
| 2026-04-11 16:02:23 | 9 | 320x240 | 1600 | 11370 | espnow_rx_0009_320x240_20260411_160223.ppm |
| 2026-04-11 16:02:48 | 10 | 320x240 | 1600 | 11890 | espnow_rx_0010_320x240_20260411_160248.ppm |
| 2026-04-11 16:03:06 | 11 | 320x240 | 1600 | 11852 | espnow_rx_0011_320x240_20260411_160306.ppm |
| 2026-04-11 16:03:26 | 12 | 320x240 | 1600 | 10906 | espnow_rx_0012_320x240_20260411_160326.ppm |
| 2026-04-11 16:03:43 | 13 | 320x240 | 1600 | 11834 | espnow_rx_0013_320x240_20260411_160343.ppm |
| 2026-04-11 16:04:16 | 16 | 320x240 | 1600 | 11975 | espnow_rx_0016_320x240_20260411_160416.ppm |
| 2026-04-11 16:21:27 | 1 | 160x120 | 400 | 2913 | espnow_rx_0001_160x120_20260411_162127.ppm |
| 2026-04-11 16:24:43 | 2 | 160x120 | 400 | 2675 | espnow_rx_0002_160x120_20260411_162443.ppm |
| 2026-04-11 16:24:52 | 3 | 160x120 | 400 | 2648 | espnow_rx_0003_160x120_20260411_162452.ppm |
| 2026-04-11 16:25:04 | 4 | 160x120 | 400 | 2818 | espnow_rx_0004_160x120_20260411_162504.ppm |
| 2026-04-11 16:25:14 | 5 | 160x120 | 400 | 2600 | espnow_rx_0005_160x120_20260411_162514.ppm |
| 2026-04-11 16:25:19 | 6 | 160x120 | 400 | 2673 | espnow_rx_0006_160x120_20260411_162519.ppm |
