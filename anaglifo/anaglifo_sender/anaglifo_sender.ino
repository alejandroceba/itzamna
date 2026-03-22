#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int SD_CS = 21;

static const uint8_t WIFI_CHANNEL = 6;
uint8_t receiverMAC[] = {0xD8, 0x3B, 0xDA, 0x45, 0xCD, 0x24};  // Update if needed

static const uint16_t PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;
static const uint16_t CHUNK_PAYLOAD = 200;  // Keep packet size safely below ESP-NOW limit

static const int AEC_VALUE = 450;
static const int AGC_GAIN = 10;

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

// ================= GLOBALS =================
static bool g_busy = false;
static volatile bool sendDone = false;
static volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
static uint16_t nextImageIdCounter = 1;

// ================= ESP-NOW CALLBACK =================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  lastSendStatus = status;
  sendDone = true;
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
    c.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }

  if (lockExposureAndGain()) {
    Serial.printf("Exposure/Gain locked. AEC=%d AGC=%d\n", AEC_VALUE, AGC_GAIN);
  }

  return true;
}

// ================= SD =================
bool initSD() {
  SPI.begin(7, 8, 9, SD_CS);
  if (!SD.begin(SD_CS, SPI)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  return true;
}

uint16_t nextIndex() {
  for (uint16_t i = 1; i < 10000; i++) {
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "/Right_%04u.pgm", i);
    if (!SD.exists(tmp)) return i;
  }
  return 9999;
}

bool saveGrayPGM(const char *path, const uint8_t *gray, uint16_t width, uint16_t height) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  f.printf("P5\n%u %u\n255\n", width, height);
  size_t total = (size_t)width * height;
  size_t w = f.write(gray, total);
  f.close();

  return w == total;
}

// ================= IMAGE HELPERS =================
uint8_t rgb565ToGray(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = (p & 0x1F) * 255 / 31;
  return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

bool buildGrayBufferFromFrame(camera_fb_t *fb, uint8_t **outGray) {
  if (!fb || !outGray) return false;

  uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
  if (fb->len != expectedLen) {
    Serial.printf("[SENDER] Unexpected frame len=%u expected=%u\n",
                  (unsigned)fb->len,
                  (unsigned)expectedLen);
    return false;
  }

  uint32_t totalPixels = (uint32_t)fb->width * fb->height;
  uint8_t *gray = (uint8_t *)malloc(totalPixels);
  if (!gray) return false;

  uint32_t px = 0;
  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint16_t pixel = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
    gray[px++] = rgb565ToGray(pixel);
  }

  *outGray = gray;
  return true;
}

bool sendImageGrayESPNow(const uint8_t *gray, uint16_t width, uint16_t height, uint16_t imageId) {
  uint32_t dataLen = (uint32_t)width * height;

  BeginPacket b{};
  b.type = PKT_BEGIN;
  b.magic = PACKET_MAGIC;
  b.imageId = imageId;
  b.width = width;
  b.height = height;
  b.dataLen = dataLen;
  b.chunkPayload = CHUNK_PAYLOAD;

  if (!sendPacketWithRetry((const uint8_t *)&b, sizeof(b))) {
    Serial.println("[SENDER] Failed to send BEGIN packet");
    return false;
  }

  uint16_t totalChunks = (dataLen + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  uint8_t packet[1 + 2 + 2 + 2 + CHUNK_PAYLOAD];

  for (uint16_t chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
    uint32_t offset = (uint32_t)chunkIndex * CHUNK_PAYLOAD;
    uint16_t n = CHUNK_PAYLOAD;
    if (offset + n > dataLen) {
      n = dataLen - offset;
    }

    packet[0] = PKT_CHUNK;
    memcpy(&packet[1], &imageId, sizeof(imageId));
    memcpy(&packet[3], &chunkIndex, sizeof(chunkIndex));
    memcpy(&packet[5], &n, sizeof(n));
    memcpy(&packet[7], gray + offset, n);

    size_t sendLen = 7 + n;
    if (!sendPacketWithRetry(packet, sendLen)) {
      Serial.printf("[SENDER] Failed chunk %u/%u\n", chunkIndex + 1, totalChunks);
      return false;
    }

    delay(2);
  }

  EndPacket e{};
  e.type = PKT_END;
  e.imageId = imageId;
  e.totalChunks = totalChunks;
  e.dataLen = dataLen;

  if (!sendPacketWithRetry((const uint8_t *)&e, sizeof(e))) {
    Serial.println("[SENDER] Failed to send END packet");
    return false;
  }

  return true;
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

// ================= ESP-NOW INIT =================
bool initEspNow() {
  WiFi.mode(WIFI_STA);

  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
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

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(BTN_PIN, INPUT_PULLUP);

  if (!initCamera()) {
    Serial.println("Camera init failed");
    while (true) delay(1000);
  }

  if (!initSD()) {
    Serial.println("SD init failed");
    while (true) delay(1000);
  }

  if (!initEspNow()) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  Serial.println("anaglifo_sender ready: press button to capture, save, and send");
}

// ================= LOOP =================
void loop() {
  if (g_busy) return;
  if (!buttonPressed()) return;

  g_busy = true;
  uint32_t tStart = millis();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[SENDER] Capture failed");
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint8_t *gray = nullptr;
  if (!buildGrayBufferFromFrame(fb, &gray)) {
    Serial.println("[SENDER] Gray conversion failed");
    esp_camera_fb_return(fb);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t idx = nextIndex();
  char fileName[40];
  snprintf(fileName, sizeof(fileName), "/Right_%04u.pgm", idx);

  if (!saveGrayPGM(fileName, gray, fb->width, fb->height)) {
    Serial.println("[SENDER] Save to SD failed");
    free(gray);
    esp_camera_fb_return(fb);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t imageId = nextImageIdCounter++;
  bool sentOk = sendImageGrayESPNow(gray, fb->width, fb->height, imageId);

  if (sentOk) {
    Serial.printf("[SENDER] OK id=%u saved=%s size=%ux%u bytes=%u\n",
                  imageId,
                  fileName,
                  fb->width,
                  fb->height,
                  (unsigned)(fb->width * fb->height));
  } else {
    Serial.printf("[SENDER] SEND FAIL id=%u (image still saved as %s)\n", imageId, fileName);
  }

  free(gray);
  esp_camera_fb_return(fb);

  uint32_t elapsed = millis() - tStart;
  Serial.printf("[SENDER] Total time: %lu ms\n", (unsigned long)elapsed);

  waitButtonRelease();
  g_busy = false;
}
