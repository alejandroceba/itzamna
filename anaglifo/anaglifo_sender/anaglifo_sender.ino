#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int SD_CS = 21;

// UART
static const int TX_PIN = D6;
static const int RX_PIN = D7;
static const uint32_t BAUD = 460800;

// Protocol
// Header: [MAGIC(2)][WIDTH(2)][HEIGHT(2)][LEN(4)]
static const uint16_t MAGIC = 0xA55A;
static const uint8_t READY_BYTE     = 0x52; // 'R'
static const uint8_t ACK_HDR_BYTE   = 0x48; // 'H'
static const uint8_t ACK_CHUNK_BYTE = 0x43; // 'C'
static const uint8_t ACK_FINAL_BYTE = 0x06; // ACK
static const uint8_t ERR_BYTE       = 0x15; // NAK
static const uint16_t CHUNK_SIZE = 512;

// ================= ESP-NOW =================
static const uint8_t WIFI_CHANNEL = 6;
uint8_t receiverMAC[] = {0xD8, 0x3B, 0xDA, 0x45, 0xCD, 0x24};  // Update if needed

static const uint16_t PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;
static const uint8_t PKT_ACK = 4;
static const uint16_t CHUNK_PAYLOAD = 200;
static const uint8_t IMAGE_SEND_ATTEMPTS = 3;
static const uint32_t IMAGE_ACK_TIMEOUT_MS = 5000;
static const uint16_t IMAGE_RETRY_DELAY_MS = 300;

// ================= EXPOSURE / GAIN =================
static const int AEC_VALUE = 450;
static const int AGC_GAIN = 10;

HardwareSerial Link(1);

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ================= TRANSPORT PACKETS =================
struct __attribute__((packed)) BeginPacket {
  uint8_t type;
  uint16_t magic;
  uint16_t imageId;
  uint16_t width;
  uint16_t height;
  uint32_t dataLen;
  uint16_t chunkPayload;
};

struct __attribute__((packed)) EndPacket {
  uint8_t type;
  uint16_t imageId;
  uint16_t totalChunks;
  uint32_t dataLen;
};

struct __attribute__((packed)) AckPacket {
  uint8_t type;
  uint16_t imageId;
  uint8_t ok;
  uint16_t chunks;
  uint32_t bytesCount;
};

static bool g_busy = false;
static bool g_sdAvailable = false;
static volatile bool sendDone = false;
static volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
static volatile bool ackReceived = false;
static volatile uint16_t ackImageId = 0;
static volatile uint8_t ackOk = 0;
static volatile uint16_t ackChunks = 0;
static volatile uint32_t ackBytes = 0;
static uint16_t nextImageIdCounter = 1;

// ================= HELPERS BINARIOS =================
static uint16_t read_u16_le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

// ================= CONVERSION TO GRAYSCALE =================
uint8_t rgb565ToGray(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = (p & 0x1F) * 255 / 31;
  return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

// ================= PATHS =================
uint16_t nextIndex() {
  for (uint16_t i = 1; i < 10000; i++) {
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "/Right_%04u.pgm", i);
    if (!SD.exists(tmp)) return i;
  }
  return 9999;
}

// ================= CAMERA =================
bool lockExposureAndGain() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return false;

  for (int i = 0; i < 6; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(40);
  }

  s->set_gain_ctrl(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);

  s->set_aec_value(s, AEC_VALUE);
  s->set_agc_gain(s, AGC_GAIN);

  s->set_saturation(s, -2);
  s->set_contrast(s, 1);

  return true;
}

bool initCamera() {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;

  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;

  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 10000000;
  c.pixel_format = PIXFORMAT_RGB565;

  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 20;
  c.fb_count = 1;

  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
    c.frame_size = FRAMESIZE_QVGA;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }

  Serial.println("Camera init OK");

  if (lockExposureAndGain()) {
    Serial.printf("Exposure/Gain locked. AEC=%d AGC=%d\n", AEC_VALUE, AGC_GAIN);
  } else {
    Serial.println("Exposure/Gain lock FAIL");
  }
  return true;
}

// ================= SD =================
bool initSD() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Attempt #1: explicit SPI pins used in the original sketch.
  SPI.begin(7, 8, 9, SD_CS);
  if (SD.begin(SD_CS, SPI) && SD.cardType() != CARD_NONE) {
    return true;
  }

  // Attempt #2: board default SPI mapping.
  SD.end();
  SPI.end();
  delay(20);
  SPI.begin();
  if (SD.begin(SD_CS) && SD.cardType() != CARD_NONE) {
    return true;
  }

  Serial.printf("[SD] init failed. cardType=%u\n", (unsigned)SD.cardType());
  return false;
}

// ================= BUTTON =================
bool buttonPressed() {
  static bool last = HIGH;
  bool now = digitalRead(BTN_PIN);
  bool pressed = false;

  if (last == HIGH && now == LOW) {
    delay(20);
    if (digitalRead(BTN_PIN) == LOW) pressed = true;
  }
  last = now;
  return pressed;
}

void waitButtonRelease() {
  while (digitalRead(BTN_PIN) == LOW) delay(5);
  delay(20);
}

// ================= UART =================
bool readExact(uint8_t *dst, size_t n, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  size_t got = 0;

  while (got < n && (millis() - t0) < timeoutMs) {
    int a = Link.available();
    if (a > 0) {
      size_t take = min((size_t)a, n - got);
      got += Link.readBytes((char*)dst + got, take);
      t0 = millis();
    } else {
      delay(1);
    }
  }
  return got == n;
}

bool readHeaderWithReadyRetry(uint8_t *dst, size_t n, uint32_t timeoutMs) {
  if (!dst || n == 0) return false;

  uint32_t t0 = millis();
  uint32_t lastReady = 0;
  size_t got = 0;
  uint32_t checkCount = 0;
  uint32_t availableCount = 0;

  Serial.printf("[DER] Starting header read: expecting %u bytes with %lu ms timeout\n", (unsigned)n, timeoutMs);

  while (got < n && (millis() - t0) < timeoutMs) {
    if ((millis() - lastReady) > 300) {
      Link.write(READY_BYTE);
      Link.flush();
      Serial.printf("[DER] Sent READY (attempt %u)\n", (unsigned)((millis() - t0) / 300));
      lastReady = millis();
    }

    int a = Link.available();
    checkCount++;
    if (a > 0) {
      availableCount++;
      Serial.printf("[DER] Link.available()=%d at check #%lu\n", a, checkCount);
      size_t take = min((size_t)a, n - got);
      got += Link.readBytes((char *)dst + got, take);
      Serial.printf("[DER] Read %u bytes, total=%u/%u\n", (unsigned)take, (unsigned)got, (unsigned)n);
      t0 = millis();
    } else {
      delay(2);
    }
  }

  Serial.printf("[DER] Header read complete: got=%u/%u, checks=%lu, available_events=%u\n", 
    (unsigned)got, (unsigned)n, checkCount, availableCount);

  if (got > 0 && got < n) {
    Serial.printf("[DER] Partial header bytes=%u/%u: ", (unsigned)got, (unsigned)n);
    for (size_t i = 0; i < got; i++) {
      Serial.printf("%02X ", dst[i]);
    }
    Serial.println();
  }

  return got == n;
}

// ================= ESP-NOW SEND =================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  lastSendStatus = status;
  sendDone = true;
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (!data || len != (int)sizeof(AckPacket)) return;

  AckPacket ack;
  memcpy(&ack, data, sizeof(ack));
  if (ack.type != PKT_ACK) return;

  ackImageId = ack.imageId;
  ackOk = ack.ok;
  ackChunks = ack.chunks;
  ackBytes = ack.bytesCount;
  ackReceived = true;
}

bool waitForAck(uint16_t imageId, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (ackReceived && ackImageId == imageId) {
      return ackOk == 1;
    }
    delay(1);
  }
  return false;
}

bool sendPacketWithRetry(const uint8_t *data, size_t len, uint8_t retries = 4) {
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    sendDone = false;

    esp_err_t err = esp_now_send(receiverMAC, data, len);
    if (err != ESP_OK) {
      delay(8);
      continue;
    }

    uint32_t t0 = millis();
    while (!sendDone && millis() - t0 < 120) {
      delay(1);
    }

    if (sendDone && lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      return true;
    }

    delay(8);
  }

  return false;
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);

  esp_err_t err;

  err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  if (err != ESP_OK) Serial.printf("[SENDER] set_protocol failed: %d\n", err);

  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  if (err != ESP_OK) Serial.printf("[SENDER] set_bandwidth failed: %d\n", err);

  err = esp_wifi_set_max_tx_power(78);
  if (err != ESP_OK) Serial.printf("[SENDER] set_tx_power failed: %d\n", err);

  err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("[SENDER] set_channel failed: %d\n", err);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[SENDER] ESP-NOW init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[SENDER] Failed to add receiver peer");
    return false;
  }

  return true;
}

// ================= GRAYSCALE + FRAME HELPERS =================
bool validateRgb565Frame(camera_fb_t *fb) {
  if (!fb) return false;

  uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
  if (fb->len != expectedLen) {
    Serial.printf("[DER] Unexpected length in right frame: len=%u expected=%u\n",
                  (unsigned)fb->len, (unsigned)expectedLen);
    return false;
  }

  return true;
}

uint8_t grayFromRgb565FrameAt(camera_fb_t *fb, uint32_t pixelIndex) {
  uint32_t i = pixelIndex * 2;
  uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
  return rgb565ToGray(p);
}

bool saveGrayPGMFromFrame(const char *path, camera_fb_t *fb) {
  if (!fb) return false;
  if (!validateRgb565Frame(fb)) return false;

  uint16_t width = fb->width;
  uint16_t height = fb->height;

  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  f.printf("P5\n%u %u\n255\n", width, height);

  uint8_t grayChunk[256];
  size_t pending = 0;
  size_t writtenPixels = 0;

  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
    grayChunk[pending++] = rgb565ToGray(p);

    if (pending == sizeof(grayChunk)) {
      size_t w = f.write(grayChunk, pending);
      if (w != pending) {
        f.close();
        return false;
      }
      writtenPixels += pending;
      pending = 0;
    }
  }

  if (pending > 0) {
    size_t w = f.write(grayChunk, pending);
    if (w != pending) {
      f.close();
      return false;
    }
    writtenPixels += pending;
  }

  f.close();

  return writtenPixels == (size_t)width * height;
}

// ================= RECEIVE LEFT + BUILD ANAGLYPH =================
bool receiveLeftAndBuildOutputs(const char *leftPath,
                                const char *anaglyphPath,
                                uint16_t width,
                                uint16_t height,
                                uint32_t totalLen,
                                camera_fb_t *rightFb,
                                bool saveToSd,
                                uint16_t imageId) {
  if (!rightFb) return false;
  if (!validateRgb565Frame(rightFb)) return false;

  if (rightFb->width != width || rightFb->height != height) {
    Serial.println("[DER] Right frame dimensions do not match LEFT header");
    return false;
  }

  uint32_t expectedLen = (uint32_t)width * height * 2;
  if (totalLen != expectedLen) {
    Serial.printf("[DER] left len mismatch: len=%u expected=%u\n",
                  (unsigned)totalLen,
                  (unsigned)expectedLen);
    return false;
  }

  uint32_t anaglyphLen = (uint32_t)width * height * 3;

  File fLeft;
  File fAna;
  if (saveToSd) {
    fLeft = SD.open(leftPath, FILE_WRITE);
    if (!fLeft) {
      Serial.println("[DER] Could not open left image file");
      return false;
    }

    fAna = SD.open(anaglyphPath, FILE_WRITE);
    if (!fAna) {
      Serial.println("[DER] Could not open anaglyph file");
      fLeft.close();
      return false;
    }

    fLeft.printf("P5\n%u %u\n255\n", width, height);
    fAna.printf("P6\n%u %u\n255\n", width, height);
  }

  ackReceived = false;
  ackImageId = 0;
  ackOk = 0;
  ackChunks = 0;
  ackBytes = 0;

  BeginPacket beginPacket{};
  beginPacket.type = PKT_BEGIN;
  beginPacket.magic = PACKET_MAGIC;
  beginPacket.imageId = imageId;
  beginPacket.width = width;
  beginPacket.height = height;
  beginPacket.dataLen = anaglyphLen;
  beginPacket.chunkPayload = CHUNK_PAYLOAD;

  if (!sendPacketWithRetry((const uint8_t *)&beginPacket, sizeof(beginPacket))) {
    Serial.println("[SENDER] Failed to send BEGIN packet");
    if (saveToSd) {
      fLeft.close();
      fAna.close();
    }
    return false;
  }

  Link.write(ACK_HDR_BYTE);
  Link.flush();

  uint8_t packet[1 + 2 + 2 + 2 + CHUNK_PAYLOAD];
  uint16_t chunkIndex = 0;
  uint16_t nEsp = 0;
  uint16_t totalChunks = (anaglyphLen + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;

  uint32_t received = 0;
  uint32_t pixelIndex = 0;

  while (received < totalLen) {
    uint8_t lenBuf[2];
    if (!readExact(lenBuf, 2, 4000)) {
      Serial.println("[DER] Timeout reading chunk size");
      if (saveToSd) {
        fLeft.close();
        fAna.close();
      }
      return false;
    }

    uint16_t n = read_u16_le(lenBuf);
    if (n == 0 || n > CHUNK_SIZE || (received + n) > totalLen || (n % 2) != 0) {
      Serial.printf("[DER] Invalid chunk size: %u\n", n);
      if (saveToSd) {
        fLeft.close();
        fAna.close();
      }
      return false;
    }

    uint8_t buf[CHUNK_SIZE];
    if (!readExact(buf, n, 4000)) {
      Serial.println("[DER] Timeout reading chunk payload");
      if (saveToSd) {
        fLeft.close();
        fAna.close();
      }
      return false;
    }

    for (uint16_t i = 0; i < n; i += 2) {
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t leftGray = rgb565ToGray(p);
      uint8_t rightGray = grayFromRgb565FrameAt(rightFb, pixelIndex);

      uint8_t rgb[3] = {leftGray, rightGray, rightGray};

      if (saveToSd) {
        fLeft.write(leftGray);
        fAna.write(rgb, 3);
      }

      for (uint8_t k = 0; k < 3; k++) {
        packet[7 + nEsp] = rgb[k];
        nEsp++;

        if (nEsp == CHUNK_PAYLOAD) {
          packet[0] = PKT_CHUNK;
          memcpy(&packet[1], &imageId, sizeof(imageId));
          memcpy(&packet[3], &chunkIndex, sizeof(chunkIndex));
          memcpy(&packet[5], &nEsp, sizeof(nEsp));

          size_t sendLen = 7 + nEsp;
          if (!sendPacketWithRetry(packet, sendLen)) {
            Serial.printf("[SENDER] Failed chunk %u/%u\n", chunkIndex + 1, totalChunks);
            if (saveToSd) {
              fLeft.close();
              fAna.close();
            }
            return false;
          }

          chunkIndex++;
          nEsp = 0;
          delay(2);
        }
      }

      pixelIndex++;
    }

    received += n;
    Link.write(ACK_CHUNK_BYTE);
    Link.flush();
  }

  if (nEsp > 0) {
    packet[0] = PKT_CHUNK;
    memcpy(&packet[1], &imageId, sizeof(imageId));
    memcpy(&packet[3], &chunkIndex, sizeof(chunkIndex));
    memcpy(&packet[5], &nEsp, sizeof(nEsp));

    size_t sendLen = 7 + nEsp;
    if (!sendPacketWithRetry(packet, sendLen)) {
      Serial.printf("[SENDER] Failed chunk %u/%u\n", chunkIndex + 1, totalChunks);
      if (saveToSd) {
        fLeft.close();
        fAna.close();
      }
      return false;
    }

    chunkIndex++;
  }

  EndPacket endPacket{};
  endPacket.type = PKT_END;
  endPacket.imageId = imageId;
  endPacket.totalChunks = totalChunks;
  endPacket.dataLen = anaglyphLen;

  if (!sendPacketWithRetry((const uint8_t *)&endPacket, sizeof(endPacket))) {
    Serial.println("[SENDER] Failed to send END packet");
    if (saveToSd) {
      fLeft.close();
      fAna.close();
    }
    return false;
  }

  if (!waitForAck(imageId, IMAGE_ACK_TIMEOUT_MS)) {
    Serial.printf("[SENDER] ACK timeout/fail (id=%u)\n", imageId);
    if (saveToSd) {
      fLeft.close();
      fAna.close();
    }
    return false;
  }

  Serial.printf("[SENDER] ACK OK (id=%u chunks=%u bytes=%lu)\n",
                imageId,
                ackChunks,
                (unsigned long)ackBytes);

  if (saveToSd) {
    fLeft.close();
    fAna.close();
  }

  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Link.setRxBufferSize(4096);
  Link.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("[DER] UART Link initialized: TX=D%d RX=D%d BAUD=%lu\n", TX_PIN, RX_PIN, BAUD);

  if (!initCamera()) {
    Serial.println("Error initializing camera");
    while (true) delay(1000);
  }

  g_sdAvailable = initSD();
  if (!g_sdAvailable) {
    Serial.println("[SD] Not available. Continuing in send-only mode.");
  } else {
    Serial.println("[SD] Ready");
  }

  if (!initEspNow()) {
    Serial.println("Error initializing ESP-NOW");
    while (true) delay(1000);
  }

  Serial.println("Dual-camera UART anaglyph + ESP-NOW mode ready");
}

// ================= LOOP =================
void loop() {
  if (g_busy) return;
  if (!buttonPressed()) return;

  g_busy = true;

  uint32_t tStart = millis();
  uint16_t idx = 0;
  String rightPath = String("/Right_NA.pgm");
  if (g_sdAvailable) {
    idx = nextIndex();
    char rightName[40];
    snprintf(rightName, sizeof(rightName), "/Right_%04u.pgm", idx);
    rightPath = String(rightName);
  }

  char leftName[40];
  char anaName[40];
  snprintf(leftName, sizeof(leftName), "/Left_%04u.pgm", idx);
  snprintf(anaName, sizeof(anaName), "/Anaglyph_%04u.ppm", idx);
  String leftPath = String(leftName);
  String anaPath = String(anaName);

  Serial.println("\n[DER] Trigger detected");

  while (Link.available()) Link.read();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[DER] Error capturing right image");
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] width=%u height=%u len=%u format=%u\n",
                fb->width, fb->height, fb->len, fb->format);

  if (!validateRgb565Frame(fb)) {
    Serial.println("[DER] Invalid frame for grayscale conversion");
    esp_camera_fb_return(fb);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  if (g_sdAvailable) {
    if (!saveGrayPGMFromFrame(rightPath.c_str(), fb)) {
      Serial.println("[DER] Error saving right image");
      esp_camera_fb_return(fb);
      waitButtonRelease();
      g_busy = false;
      return;
    }

    Serial.printf("[DER] Right image saved to %s\n", rightPath.c_str());
  } else {
    Serial.println("[DER] SD not available: skipping save");
  }

  uint16_t rightW = fb->width;
  uint16_t rightH = fb->height;

  Serial.println("[DER] Waiting for LEFT header (sending READY)...");

  uint8_t hdr[10];
  if (!readHeaderWithReadyRetry(hdr, sizeof(hdr), 15000)) {
    Serial.println("[DER] Timeout reading LEFT header");
    esp_camera_fb_return(fb);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t magic = read_u16_le(&hdr[0]);
  uint16_t leftW = read_u16_le(&hdr[2]);
  uint16_t leftH = read_u16_le(&hdr[4]);
  uint32_t totalLen = read_u32_le(&hdr[6]);

  if (magic != MAGIC) {
    Serial.printf("[DER] BAD MAGIC 0x%04X\n", magic);
    esp_camera_fb_return(fb);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] LEFT header: w=%u h=%u len=%u\n",
                leftW,
                leftH,
                (unsigned)totalLen);

  if (leftW != rightW || leftH != rightH) {
    Serial.println("[DER] Left/Right dimension mismatch");
    esp_camera_fb_return(fb);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t imageId = nextImageIdCounter++;
  if (!receiveLeftAndBuildOutputs(leftPath.c_str(),
                                  anaPath.c_str(),
                                  leftW,
                                  leftH,
                                  totalLen,
                                  fb,
                                  g_sdAvailable,
                                  imageId)) {
    Serial.println("[DER] Error receiving left / building anaglyph");
    esp_camera_fb_return(fb);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  esp_camera_fb_return(fb);

  if (g_sdAvailable) {
    Serial.printf("[DER] Left image saved to %s\n", leftPath.c_str());
    Serial.printf("[DER] Anaglyph saved to %s\n", anaPath.c_str());
  }

  Serial.printf("[SENDER] ESP-NOW anaglyph sent (id=%u)\n", imageId);

  Link.write(ACK_FINAL_BYTE);
  Link.flush();
  Serial.println("[DER] Final ACK sent to LEFT camera");

  uint32_t elapsed = millis() - tStart;
  Serial.printf("[DER] Capture+UART+anaglyph+send time: %lu ms (%.2f s)\n",
                (unsigned long)elapsed,
                elapsed / 1000.0f);

  waitButtonRelease();
  g_busy = false;
}
