#include <Arduino.h>
#include "esp_camera.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int TX_PIN  = D6;
static const int RX_PIN  = D7;
static const uint32_t BAUD = 921600;
static const uint8_t WIFI_CHANNEL = 6;

// Receiver image protocol (forwarded by sender.ino)
static const uint16_t IMAGE_PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;
static const uint16_t IMAGE_CHUNK_PAYLOAD = 200;
static const uint8_t IMAGE_SEND_GAP_MS = 10;

// TODO(INTEGRATION): when Sender MAC is known, set IMAGE_SEND_BROADCAST=false
// and replace SENDER_MAC with the real unicast target.
// Keep broadcast only for initial bring-up.
static const bool IMAGE_SEND_BROADCAST = false;
static uint8_t SENDER_MAC[6] = {0x34, 0x85, 0x18, 0x8b, 0x8a, 0x34};

// Protocolo
// Header: [MAGIC(2)][WIDTH(2)][HEIGHT(2)][LEN(4)]
static const uint16_t MAGIC = 0xA55A;
static const uint16_t CHUNK_MARKER = 0xCAFE;
static const char *PROTO_VERSION = "v4-marker";
static const uint8_t READY_BYTE     = 0x52; // 'R'
static const uint8_t ACK_HDR_BYTE   = 0x48; // 'H'
static const uint8_t ACK_CHUNK_BYTE = 0x43; // 'C'
static const uint8_t ACK_FINAL_BYTE = 0x06; // ACK
static const uint8_t ERR_BYTE       = 0x15; // NAK

static const uint16_t CHUNK_SIZE = 256;

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

// ================= EXPOSURE / GAIN =================
static const int AEC_VALUE = 450;
static const int AGC_GAIN  = 10;

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

static bool g_busy = false;
static uint16_t g_imageId = 1;
static uint8_t g_senderPeerAddr[6] = {0};

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

// ================= CONVERSIÓN A GRIS =================
uint8_t rgb565ToGray(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = ( p        & 0x1F) * 255 / 31;

  return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

static inline uint16_t packAnaglyphRGB565(uint8_t leftGray, uint8_t rightGray) {
  uint16_t r = ((uint16_t)(leftGray  >> 3) & 0x1F) << 11;
  uint16_t g = ((uint16_t)(rightGray >> 2) & 0x3F) << 5;
  uint16_t b = ((uint16_t)(rightGray >> 3) & 0x1F);
  return (uint16_t)(r | g | b);
}

// ================= CÁMARA =================
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
  c.ledc_timer   = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;

  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;

  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 10000000;
  c.pixel_format = PIXFORMAT_RGB565;

  c.frame_size   = FRAMESIZE_QVGA;
  c.jpeg_quality = 20;
  c.fb_count     = 1;

  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location  = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
    c.frame_size   = FRAMESIZE_QVGA;
    c.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }

  Serial.println("Camera init OK ✅");

  if (lockExposureAndGain()) {
    Serial.printf("Exposure/Gain locked ✅  AEC=%d  AGC=%d\n", AEC_VALUE, AGC_GAIN);
  } else {
    Serial.println("Exposure/Gain lock FAIL ⚠️");
  }
  return true;
}

// ================= ESP-NOW =================
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[DER] esp_now_init failed");
    return false;
  }

  if (IMAGE_SEND_BROADCAST) {
    uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(g_senderPeerAddr, bcast, sizeof(g_senderPeerAddr));
  } else {
    memcpy(g_senderPeerAddr, SENDER_MAC, sizeof(g_senderPeerAddr));
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, g_senderPeerAddr, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(g_senderPeerAddr)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("[DER] esp_now_add_peer failed");
      return false;
    }
  }

  return true;
}

bool sendEspNowPacket(const uint8_t *data, size_t len) {
  if (!data || len == 0 || len > 250) return false;

  esp_err_t err = ESP_FAIL;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    err = esp_now_send(g_senderPeerAddr, data, len);
    if (err == ESP_OK) {
      break;
    }

    delay((attempt + 1) * 3);
    yield();
  }

  if (err != ESP_OK) {
    Serial.printf("[DER] esp_now_send error: %d\n", err);
    return false;
  }
  if (IMAGE_SEND_GAP_MS > 0) {
    delay(IMAGE_SEND_GAP_MS);
  }
  return true;
}

bool sendImageBegin(uint16_t imageId, uint16_t width, uint16_t height, uint32_t dataLen) {
  BeginPacket b{};
  b.type = PKT_BEGIN;
  b.magic = IMAGE_PACKET_MAGIC;
  b.imageId = imageId;
  b.width = width;
  b.height = height;
  b.dataLen = dataLen;
  b.chunkPayload = IMAGE_CHUNK_PAYLOAD;
  return sendEspNowPacket((const uint8_t *)&b, sizeof(b));
}

bool sendImageChunk(uint16_t imageId, uint16_t chunkIndex, const uint8_t *payload, uint16_t n) {
  if (!payload || n == 0 || n > IMAGE_CHUNK_PAYLOAD) return false;

  uint8_t pkt[7 + IMAGE_CHUNK_PAYLOAD];
  pkt[0] = PKT_CHUNK;
  pkt[1] = (uint8_t)(imageId & 0xFF);
  pkt[2] = (uint8_t)((imageId >> 8) & 0xFF);
  pkt[3] = (uint8_t)(chunkIndex & 0xFF);
  pkt[4] = (uint8_t)((chunkIndex >> 8) & 0xFF);
  pkt[5] = (uint8_t)(n & 0xFF);
  pkt[6] = (uint8_t)((n >> 8) & 0xFF);
  memcpy(&pkt[7], payload, n);

  return sendEspNowPacket(pkt, (size_t)(7 + n));
}

bool sendImageEnd(uint16_t imageId, uint16_t totalChunks, uint32_t dataLen) {
  EndPacket e{};
  e.type = PKT_END;
  e.imageId = imageId;
  e.totalChunks = totalChunks;
  e.dataLen = dataLen;
  return sendEspNowPacket((const uint8_t *)&e, sizeof(e));
}

bool buildGrayBufferFromFrame(camera_fb_t *fb, uint8_t **outGray) {
  if (!fb || !outGray) return false;

  uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
  if (fb->len != expectedLen) {
    Serial.printf("[DER] len inesperado en derecha: len=%u esperado=%u\n",
                  (unsigned)fb->len, (unsigned)expectedLen);
    return false;
  }

  uint32_t totalPixels = (uint32_t)fb->width * fb->height;
  uint8_t *gray = (uint8_t*)malloc(totalPixels);
  if (!gray) {
    Serial.println("[DER] No se pudo reservar rightGray");
    return false;
  }

  uint32_t px = 0;
  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
    gray[px++] = rgb565ToGray(p);
  }

  *outGray = gray;
  return true;
}

bool receiveLeftAndSendAnaglyphUART(uint16_t imageId,
                                    uint16_t width,
                                    uint16_t height,
                                    uint32_t totalLen,
                                    const uint8_t *rightGray) {
  uint32_t expectedLen = (uint32_t)width * height * 2;
  if (totalLen != expectedLen) {
    Serial.printf("[DER] len no coincide con width/height: len=%u esperado=%u\n",
                  (unsigned)totalLen, (unsigned)expectedLen);
    return false;
  }

  bool espOk = true;
  if (!sendImageBegin(imageId, width, height, expectedLen)) {
    Serial.println("[DER] No se pudo enviar PKT_BEGIN de anaglifo");
    espOk = false;
  }

  Link.write(ACK_HDR_BYTE);
  Link.flush();

  uint32_t received = 0;
  uint32_t pixelIndex = 0;
  uint8_t imgPayload[IMAGE_CHUNK_PAYLOAD];
  uint16_t imgPayloadLen = 0;
  uint16_t chunkIndex = 0;
  uint32_t sentAnaglyphBytes = 0;

  while (received < totalLen) {
    uint8_t lenBuf[2];
    if (!readExact(lenBuf, 2, 4000)) {
      Serial.println("[DER] Timeout leyendo tamaño de bloque");
      return false;
    }

    uint16_t n = read_u16_le(lenBuf);
    if (n == 0 || n > CHUNK_SIZE || (received + n) > totalLen) {
      Serial.printf("[DER] Bloque inválido: %u\n", n);
      return false;
    }

    if (n % 2 != 0) {
      Serial.println("[DER] Bloque RGB565 impar");
      return false;
    }

    uint8_t buf[CHUNK_SIZE];
    if (!readExact(buf, n, 4000)) {
      Serial.println("[DER] Timeout leyendo bloque");
      return false;
    }

    for (uint16_t i = 0; i < n; i += 2) {
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t leftGray = rgb565ToGray(p);
      uint8_t rightG = rightGray[pixelIndex++];
      uint16_t anaPix = packAnaglyphRGB565(leftGray, rightG);

      if (imgPayloadLen > (IMAGE_CHUNK_PAYLOAD - 2)) {
        if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
          espOk = false;
        }
        sentAnaglyphBytes += imgPayloadLen;
        chunkIndex++;
        imgPayloadLen = 0;
      }

      imgPayload[imgPayloadLen++] = (uint8_t)((anaPix >> 8) & 0xFF);
      imgPayload[imgPayloadLen++] = (uint8_t)(anaPix & 0xFF);
    }

    received += n;
    Link.write(ACK_CHUNK_BYTE);
    Link.flush();
  }

  if (imgPayloadLen > 0) {
    if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
      espOk = false;
    }
    sentAnaglyphBytes += imgPayloadLen;
    chunkIndex++;
  }

  if (sentAnaglyphBytes != expectedLen) {
    Serial.printf("[DER] Mismatch bytes anaglifo: enviados=%lu esperados=%lu\n",
                  (unsigned long)sentAnaglyphBytes,
                  (unsigned long)expectedLen);
    return false;
  }

  if (espOk) {
    if (!sendImageEnd(imageId, chunkIndex, expectedLen)) {
      espOk = false;
    }
  }

  return espOk;
}

bool sendPhotoFrame(uint16_t imageId, camera_fb_t *fb) {
  if (!fb) return false;

  uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
  if (fb->len != expectedLen) {
    Serial.printf("[DER] len inesperado en foto: len=%u esperado=%u\n",
                  (unsigned)fb->len, (unsigned)expectedLen);
    return false;
  }

  uint32_t anaglyphLen = (uint32_t)fb->width * fb->height * 2;
  if (!sendImageBegin(imageId, fb->width, fb->height, anaglyphLen)) {
    Serial.println("[DER] No se pudo enviar PKT_BEGIN de anaglifo");
    return false;
  }

  uint8_t imgPayload[IMAGE_CHUNK_PAYLOAD];
  uint16_t imgPayloadLen = 0;
  uint16_t chunkIndex = 0;
  uint32_t sentAnaglyphBytes = 0;

  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];

    uint8_t gray = rgb565ToGray(p);
    uint16_t anaPix = packAnaglyphRGB565(gray, gray);
    uint8_t anaHi = (uint8_t)((anaPix >> 8) & 0xFF);
    uint8_t anaLo = (uint8_t)(anaPix & 0xFF);

    if (imgPayloadLen > (IMAGE_CHUNK_PAYLOAD - 2)) {
      if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
        Serial.println("[DER] No se pudo enviar PKT_CHUNK de anaglifo");
        return false;
      }
      sentAnaglyphBytes += imgPayloadLen;
      chunkIndex++;
      imgPayloadLen = 0;
    }

    imgPayload[imgPayloadLen++] = anaHi;
    imgPayload[imgPayloadLen++] = anaLo;
  }

  if (imgPayloadLen > 0) {
    if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
      Serial.println("[DER] No se pudo enviar ultimo PKT_CHUNK de anaglifo");
      return false;
    }
    sentAnaglyphBytes += imgPayloadLen;
    chunkIndex++;
  }

  if (sentAnaglyphBytes != anaglyphLen) {
    Serial.printf("[DER] Mismatch bytes anaglifo: enviados=%lu esperados=%lu\n",
                  (unsigned long)sentAnaglyphBytes,
                  (unsigned long)anaglyphLen);
    return false;
  }

  if (!sendImageEnd(imageId, chunkIndex, anaglyphLen)) {
    Serial.println("[DER] No se pudo enviar PKT_END de anaglifo");
    return false;
  }

  return true;
}

// ================= BOTÓN =================
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

#if 0
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

bool waitForChunkMarker(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t prev = 0;
  bool havePrev = false;
  static bool dumpedOnce = false;
  uint8_t dumpBuf[32];
  uint8_t dumpCount = 0;

  while ((millis() - t0) < timeoutMs) {
    if (Link.available()) {
      uint8_t b = (uint8_t)Link.read();
      if (!dumpedOnce && dumpCount < sizeof(dumpBuf)) {
        dumpBuf[dumpCount++] = b;
      }

      if (havePrev) {
        uint16_t v = (uint16_t)prev | ((uint16_t)b << 8);
        if (v == CHUNK_MARKER) {
          if (!dumpedOnce) {
            Serial.print("[DER] First bytes after header: ");
            for (uint8_t i = 0; i < dumpCount; i++) {
              Serial.printf("%02X ", dumpBuf[i]);
            }
            Serial.println();
            dumpedOnce = true;
          }
          return true;
        }
      }

      prev = b;
      havePrev = true;
    } else {
      delay(1);
    }
  }

  if (!dumpedOnce && dumpCount > 0) {
    Serial.print("[DER] First bytes after header (timeout): ");
    for (uint8_t i = 0; i < dumpCount; i++) {
      Serial.printf("%02X ", dumpBuf[i]);
    }
    Serial.println();
    dumpedOnce = true;
  }

  return false;
}

// ================= BUFFER GRIS =================
bool buildGrayBufferFromFrame(camera_fb_t *fb, uint8_t **outGray) {
  if (!fb || !outGray) return false;

  uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
  if (fb->len != expectedLen) {
    Serial.printf("[DER] len inesperado en derecha: len=%u esperado=%u\n",
                  (unsigned)fb->len, (unsigned)expectedLen);
    return false;
  }

  uint32_t totalPixels = (uint32_t)fb->width * fb->height;
  uint8_t *gray = (uint8_t*)malloc(totalPixels);
  if (!gray) {
    Serial.println("[DER] No se pudo reservar rightGray");
    return false;
  }

  uint32_t px = 0;
  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
    gray[px++] = rgb565ToGray(p);
  }

  *outGray = gray;
  return true;
}

// ================= RECEPCIÓN IZQUIERDA + ANAGLIFO =================
bool receiveLeftAndSendAnaglyph(uint16_t imageId,
                                uint16_t width,
                                uint16_t height,
                                uint32_t totalLen,
                                const uint8_t *rightGray) {
  uint32_t expectedLen = (uint32_t)width * height * 2;
  if (totalLen != expectedLen) {
    Serial.printf("[DER] len no coincide con width/height: len=%u esperado=%u\n",
                  (unsigned)totalLen, (unsigned)expectedLen);
    return false;
  }

  uint32_t anaglyphLen = (uint32_t)width * height * 3;
  if (!sendImageBegin(imageId, width, height, anaglyphLen)) {
    Serial.println("[DER] No se pudo enviar PKT_BEGIN");
    return false;
  }

  uint32_t received = 0;
  uint32_t pixelIndex = 0;
  uint8_t imgPayload[IMAGE_CHUNK_PAYLOAD];
  uint16_t imgPayloadLen = 0;
  uint16_t chunkIndex = 0;
  uint32_t sentAnaglyphBytes = 0;
  uint32_t chunkCount = 0;

  while (received < totalLen) {
    if (!waitForChunkMarker(4000)) {
      Serial.println("[DER] Timeout esperando marcador de bloque");
      return false;
    }

    uint8_t lenBuf[2];
    if (!readExact(lenBuf, 2, 4000)) {
      Serial.println("[DER] Timeout leyendo tamaño de bloque");
      return false;
    }

    uint16_t n = read_u16_le(lenBuf);

    if (n == 0 || n > CHUNK_SIZE || (received + n) > totalLen) {
      Serial.printf("[DER] Bloque inválido: %u\n", n);
      return false;
    }

    if (n % 2 != 0) {
      Serial.println("[DER] Bloque RGB565 impar");
      return false;
    }

    uint8_t buf[CHUNK_SIZE];
    if (!readExact(buf, n, 4000)) {
      Serial.println("[DER] Timeout leyendo bloque");
      return false;
    }

    for (uint16_t i = 0; i < n; i += 2) {
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t leftGray = rgb565ToGray(p);

      // Ensure room for a full RGB triplet. IMAGE_CHUNK_PAYLOAD (200) is not
      // divisible by 3, so flush when less than 3 bytes remain.
      if (imgPayloadLen > (IMAGE_CHUNK_PAYLOAD - 3)) {
        if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
          Serial.println("[DER] No se pudo enviar PKT_CHUNK");
          return false;
        }
        sentAnaglyphBytes += imgPayloadLen;
        chunkIndex++;
        imgPayloadLen = 0;
      }

      // Anaglifo:
      // R = izquierda
      // G = derecha
      // B = derecha
      imgPayload[imgPayloadLen++] = leftGray;
      imgPayload[imgPayloadLen++] = rightGray[pixelIndex];
      imgPayload[imgPayloadLen++] = rightGray[pixelIndex];

      pixelIndex++;
    }

    received += n;
    chunkCount++;
    if ((chunkCount % 100) == 0) {
      Serial.printf("[DER] Progreso chunks=%lu bytes=%lu/%lu\n",
                    (unsigned long)chunkCount,
                    (unsigned long)received,
                    (unsigned long)totalLen);
    }
  }

  if (imgPayloadLen > 0) {
    if (!sendImageChunk(imageId, chunkIndex, imgPayload, imgPayloadLen)) {
      Serial.println("[DER] No se pudo enviar último PKT_CHUNK");
      return false;
    }
    sentAnaglyphBytes += imgPayloadLen;
    chunkIndex++;
  }

  if (sentAnaglyphBytes != anaglyphLen) {
    Serial.printf("[DER] Mismatch bytes anaglifo: enviados=%lu esperados=%lu\n",
                  (unsigned long)sentAnaglyphBytes,
                  (unsigned long)anaglyphLen);
    return false;
  }

  if (!sendImageEnd(imageId, chunkIndex, anaglyphLen)) {
    Serial.println("[DER] No se pudo enviar PKT_END");
    return false;
  }

  return true;
}

// ================= SETUP =================
#endif
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.printf("[DER] PROTO %s\n", PROTO_VERSION);

  pinMode(BTN_PIN, INPUT_PULLUP);
  Link.setRxBufferSize(4096);
  Link.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  if (!initCamera()) {
    Serial.println("Error al iniciar cámara derecha");
    while (true) delay(1000);
  }

  if (!initEspNow()) {
    Serial.println("Error al iniciar ESP-NOW");
    while (true) delay(1000);
  }

  Serial.println("XIAO derecha lista");
}

// ================= LOOP =================
void loop() {
  if (g_busy) return;
  if (!buttonPressed()) return;

  g_busy = true;
  while (Link.available()) Link.read();

  uint32_t tStart = millis();
  uint16_t imageId = g_imageId++;

  Serial.println("\n[DER] Trigger detectado");

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[DER] Error capturando imagen derecha");
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] width=%u height=%u len=%u format=%u\n",
                fb->width, fb->height, fb->len, fb->format);

  uint8_t *rightGray = nullptr;
  if (!buildGrayBufferFromFrame(fb, &rightGray)) {
    Serial.println("[DER] Error construyendo rightGray");
    esp_camera_fb_return(fb);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t rightW = fb->width;
  uint16_t rightH = fb->height;

  esp_camera_fb_return(fb);

  Link.write(READY_BYTE);
  Link.flush();
  Serial.println("[DER] READY enviado, esperando header...");

  uint8_t hdr[10];
  if (!readExact(hdr, sizeof(hdr), 8000)) {
    Serial.println("[DER] Timeout leyendo header");
    free(rightGray);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t magic    = read_u16_le(&hdr[0]);
  uint16_t leftW    = read_u16_le(&hdr[2]);
  uint16_t leftH    = read_u16_le(&hdr[4]);
  uint32_t totalLen = read_u32_le(&hdr[6]);

  if (magic != MAGIC) {
    Serial.printf("[DER] BAD MAGIC 0x%04X\n", magic);
    free(rightGray);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] Header recibido: w=%u h=%u len=%u\n",
                leftW, leftH, (unsigned)totalLen);

  if (leftW != rightW || leftH != rightH) {
    Serial.println("[DER] Dimensiones izquierda/derecha no coinciden");
    free(rightGray);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  if (!receiveLeftAndSendAnaglyphUART(imageId, leftW, leftH, totalLen, rightGray)) {
    Serial.println("[DER] Error generando/enviando anaglifo por ESP-NOW");
    free(rightGray);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  free(rightGray);

  Link.write(ACK_FINAL_BYTE);
  Link.flush();

  Serial.printf("[DER] Anaglifo enviado por ESP-NOW, imageId=%u\n", imageId);

  uint32_t elapsed = millis() - tStart;
  Serial.printf("[DER] Tiempo total: %lu ms (%.2f s)\n",
              (unsigned long)elapsed,
              elapsed / 1000.0f);
  Serial.println("[DER] Recepción y envío completados");

  waitButtonRelease();
  g_busy = false;
}