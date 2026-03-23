#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

// ---------------- CONFIGURATION ----------------
#define WIFI_CHANNEL 6
#define SERIAL_BAUD 460800
#define DEBUG 0
#define IMG_DEBUG 0
#define STATUS_PRINT_ENABLE 0

// 1 = full-rate sensor CSV print; increase to throttle printing if needed at high packet rates.
#define SENSOR_PRINT_DIVIDER 1

// Image protocol
static const uint16_t PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;
static const uint8_t PKT_ACK = 4;
static const uint16_t IMG_MAX_CHUNK_PAYLOAD = 200;
static const uint32_t IMG_MAX_IMAGE_BYTES = 120000;   // safety cap
static const uint16_t IMG_SERIAL_CHUNK_BYTES = 64;    // smaller lines are more robust over USB CDC serial
static const uint8_t IMG_SERIAL_CHUNK_INTERVAL_MS = 6; // extra pacing reduces host-side parser overruns
static const uint8_t IMG_SERIAL_CHUNK_DUPLICATES = 2;  // transmit each chunk multiple times for redundancy
static const uint8_t IMG_SERIAL_BEGIN_REPEATS = 2;     // repeat begin frame to improve host lock-on
static const uint8_t SERIAL_IMG_SOF1 = 0xA5;
static const uint8_t SERIAL_IMG_SOF2 = 0x5A;
static const uint8_t SERIAL_IMG_FRAME_BEGIN = 1;
static const uint8_t SERIAL_IMG_FRAME_CHUNK = 2;
static const uint8_t SERIAL_IMG_FRAME_END = 3;

#if IMG_DEBUG
#define IMG_DBG(...) Serial.printf(__VA_ARGS__)
#else
#define IMG_DBG(...)
#endif

// ---------------- ERROR COUNTERS ----------------
uint32_t invalidSizePackets = 0;
uint32_t unknownPackets = 0;

// ============================================================================
// SENSOR PACKET STRUCT - Must match sender.ino
// ============================================================================
typedef struct {
  uint32_t packetNumber;
  uint32_t senderTimestamp;
  float temperature_bme280;
  float temperature_ds18b20;
  float pressure_hpa;
  float altitude_m;
  float velocity_x;
  float velocity_y;
  float velocity_z;
  float accel_x;
  float accel_y;
  float accel_z;
} sensor_packet;

// ============================================================================
// IMAGE PACKET STRUCTS - Must match anaglifo_sender.ino
// ============================================================================
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

// ============================================================================
// SENSOR STATE (queue to keep callback short)
// ============================================================================
struct SensorQueueItem {
  sensor_packet pkt;
  int16_t rssi;
};

static const uint16_t SENSOR_QUEUE_SIZE = 128;
SensorQueueItem g_sensorQueue[SENSOR_QUEUE_SIZE];
volatile uint16_t g_sensorHead = 0;
volatile uint16_t g_sensorTail = 0;
volatile uint32_t g_sensorQueueDrops = 0;

uint32_t received = 0, lost = 0, lastPacket = 0;
uint32_t bytesReceived = 0;
unsigned long lastThroughputCalc = 0;
float smoothedRSSI = -60;
const float alpha = 0.2;
uint32_t sensorPrintCounter = 0;

// ============================================================================
// IMAGE RX + EMIT STATE
// ============================================================================
volatile bool g_imgReceiving = false;
volatile uint16_t g_imgImageId = 0;
volatile uint16_t g_imgWidth = 0;
volatile uint16_t g_imgHeight = 0;
volatile uint32_t g_imgDataLen = 0;
volatile uint16_t g_imgChunkPayload = 0;
volatile uint16_t g_imgExpectedChunk = 0;
volatile uint32_t g_imgReceivedBytes = 0;

uint8_t *g_imgBuffer = nullptr;
volatile bool g_imgReadyToEmit = false;
volatile bool g_imgEmitActive = false;
volatile uint32_t g_imgEmitOffset = 0;
volatile uint16_t g_imgEmitChunkIndex = 0;
volatile uint16_t g_imgEmitImageId = 0;
volatile uint16_t g_imgEmitWidth = 0;
volatile uint16_t g_imgEmitHeight = 0;
volatile uint32_t g_imgEmitDataLen = 0;

volatile bool g_imgBeginPending = false;
volatile uint16_t g_imgBeginId = 0;
volatile uint16_t g_imgBeginW = 0;
volatile uint16_t g_imgBeginH = 0;
volatile uint32_t g_imgBeginLen = 0;

volatile bool g_imgEndPending = false;
volatile uint16_t g_imgEndId = 0;
volatile uint8_t g_imgEndOk = 0;
volatile uint16_t g_imgEndChunks = 0;
volatile uint32_t g_imgEndBytes = 0;

volatile uint32_t g_imgOkImages = 0;
volatile uint32_t g_imgBadPackets = 0;

// ============================================================================
// SOURCE MAC ROUTING (different XIAOs for sensor and image)
// ============================================================================
bool g_haveSensorMac = false;
bool g_haveImageMac = false;
volatile bool g_imagePeerReady = false;
uint8_t g_sensorMac[6] = {0};
uint8_t g_imageMac[6] = {0};

volatile bool g_imgAckPending = false;
volatile uint16_t g_imgAckId = 0;
volatile uint8_t g_imgAckOk = 0;
volatile uint16_t g_imgAckChunks = 0;
volatile uint32_t g_imgAckBytes = 0;

// ============================================================================
// CRITICAL SECTION
// ============================================================================
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// HELPERS
// ============================================================================
bool macEqual(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void macCopy(uint8_t *dst, const uint8_t *src) {
  for (int i = 0; i < 6; i++) dst[i] = src[i];
}

void printMac(const char *label, const uint8_t *mac) {
  Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n",
                label,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

float calculateDistance(float rssi) {
  float rssiAt1Meter = -56;
  float n = 2.0;
  return pow(10.0, ((rssiAt1Meter - rssi) / (10.0 * n)));
}

void signalImageBegin(uint16_t imageId, uint16_t width, uint16_t height, uint32_t dataLen) {
  g_imgBeginId = imageId;
  g_imgBeginW = width;
  g_imgBeginH = height;
  g_imgBeginLen = dataLen;
  g_imgBeginPending = true;
}

void signalImageEnd(uint16_t imageId, uint8_t ok, uint16_t chunks, uint32_t bytesCount) {
  g_imgEndId = imageId;
  g_imgEndOk = ok;
  g_imgEndChunks = chunks;
  g_imgEndBytes = bytesCount;
  g_imgEndPending = true;
}

void signalImageAck(uint16_t imageId, uint8_t ok, uint16_t chunks, uint32_t bytesCount) {
  if (imageId == 0) return;
  g_imgAckId = imageId;
  g_imgAckOk = ok;
  g_imgAckChunks = chunks;
  g_imgAckBytes = bytesCount;
  g_imgAckPending = true;
}

void resetImageRxStateNoFree() {
  g_imgReceiving = false;
  g_imgImageId = 0;
  g_imgWidth = 0;
  g_imgHeight = 0;
  g_imgDataLen = 0;
  g_imgChunkPayload = 0;
  g_imgExpectedChunk = 0;
  g_imgReceivedBytes = 0;
}

void freeImageBufferIfAny() {
  if (g_imgBuffer != nullptr) {
    free(g_imgBuffer);
    g_imgBuffer = nullptr;
  }
}

void abortImageRx(uint8_t okFlag) {
  signalImageAck(g_imgImageId, okFlag, g_imgExpectedChunk, g_imgReceivedBytes);
  signalImageEnd(g_imgImageId, okFlag, g_imgExpectedChunk, g_imgReceivedBytes);
  freeImageBufferIfAny();
  resetImageRxStateNoFree();
  g_imgReadyToEmit = false;
  g_imgEmitActive = false;
  g_imgEmitOffset = 0;
  g_imgEmitChunkIndex = 0;
  g_imgEmitImageId = 0;
  g_imgEmitWidth = 0;
  g_imgEmitHeight = 0;
  g_imgEmitDataLen = 0;
}

bool enqueueSensor(const sensor_packet &pkt, int16_t rssi) {
  uint16_t nextHead = (g_sensorHead + 1) % SENSOR_QUEUE_SIZE;
  if (nextHead == g_sensorTail) {
    g_sensorQueueDrops++;
    return false;
  }

  g_sensorQueue[g_sensorHead].pkt = pkt;
  g_sensorQueue[g_sensorHead].rssi = rssi;
  g_sensorHead = nextHead;
  return true;
}

bool dequeueSensor(SensorQueueItem &out) {
  if (g_sensorTail == g_sensorHead) return false;
  out = g_sensorQueue[g_sensorTail];
  g_sensorTail = (g_sensorTail + 1) % SENSOR_QUEUE_SIZE;
  return true;
}

void printHexPayload(const uint8_t *data, uint16_t n) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint16_t i = 0; i < n; i++) {
    uint8_t b = data[i];
    char out[2] = {HEX_DIGITS[(b >> 4) & 0x0F], HEX_DIGITS[b & 0x0F]};
    Serial.write((const uint8_t *)out, 2);
  }
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void emitImageFrame(uint8_t frameType, const uint8_t *payload, uint16_t payloadLen) {
  // Frame layout: SOF1 SOF2 type len_lo len_hi payload... crc_lo crc_hi
  uint8_t frame[300];
  size_t total = (size_t)7 + (size_t)payloadLen;
  if (total > sizeof(frame)) return;

  frame[0] = SERIAL_IMG_SOF1;
  frame[1] = SERIAL_IMG_SOF2;
  frame[2] = frameType;
  frame[3] = (uint8_t)(payloadLen & 0xFF);
  frame[4] = (uint8_t)((payloadLen >> 8) & 0xFF);

  if (payloadLen > 0 && payload != nullptr) {
    memcpy(&frame[5], payload, payloadLen);
  }

  uint16_t crc = crc16_ccitt(&frame[2], (size_t)3 + (size_t)payloadLen);
  frame[5 + payloadLen] = (uint8_t)(crc & 0xFF);
  frame[6 + payloadLen] = (uint8_t)((crc >> 8) & 0xFF);

  Serial.write(frame, total);
}

void emitBeginFrame(uint16_t imageId, uint16_t width, uint16_t height, uint32_t dataLen) {
  uint8_t payload[10];
  memcpy(&payload[0], &imageId, sizeof(imageId));
  memcpy(&payload[2], &width, sizeof(width));
  memcpy(&payload[4], &height, sizeof(height));
  memcpy(&payload[6], &dataLen, sizeof(dataLen));
  emitImageFrame(SERIAL_IMG_FRAME_BEGIN, payload, sizeof(payload));
}

void emitChunkFrame(uint16_t imageId, uint16_t chunkIndex, uint16_t n, const uint8_t *data) {
  uint8_t payload[6 + IMG_SERIAL_CHUNK_BYTES];
  memcpy(&payload[0], &imageId, sizeof(imageId));
  memcpy(&payload[2], &chunkIndex, sizeof(chunkIndex));
  memcpy(&payload[4], &n, sizeof(n));
  if (n > 0 && data != nullptr) {
    memcpy(&payload[6], data, n);
  }
  emitImageFrame(SERIAL_IMG_FRAME_CHUNK, payload, (uint16_t)(6 + n));
}

void emitEndFrame(uint16_t imageId, uint8_t ok, uint16_t chunks, uint32_t bytesCount) {
  uint8_t payload[9];
  memcpy(&payload[0], &imageId, sizeof(imageId));
  payload[2] = ok;
  memcpy(&payload[3], &chunks, sizeof(chunks));
  memcpy(&payload[5], &bytesCount, sizeof(bytesCount));
  emitImageFrame(SERIAL_IMG_FRAME_END, payload, sizeof(payload));
}

bool ensureImagePeerRegistered() {
  if (!g_haveImageMac) return false;
  if (g_imagePeerReady) return true;

  if (!esp_now_is_peer_exist(g_imageMac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, g_imageMac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
      return false;
    }
  }

  g_imagePeerReady = true;
  return true;
}

bool sendAckPacket(uint16_t imageId, uint8_t ok, uint16_t chunks, uint32_t bytesCount) {
  if (!ensureImagePeerRegistered()) return false;

  AckPacket ack{};
  ack.type = PKT_ACK;
  ack.imageId = imageId;
  ack.ok = ok;
  ack.chunks = chunks;
  ack.bytesCount = bytesCount;

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (esp_now_send(g_imageMac, (const uint8_t *)&ack, sizeof(ack)) == ESP_OK) {
      return true;
    }
    delay(2);
  }

  return false;
}

// ============================================================================
// IMAGE CALLBACK ROUTING
// ============================================================================
void handleImagePacket(const uint8_t *data, int len) {
  if (len <= 0 || data == nullptr) {
    g_imgBadPackets++;
    IMG_DBG("IMG_DBG bad_packet null_or_empty len=%d\n", len);
    return;
  }

  uint8_t type = data[0];

  if (type == PKT_BEGIN) {
    if ((size_t)len != sizeof(BeginPacket)) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG begin_bad_size len=%d expected=%u\n", len, (unsigned)sizeof(BeginPacket));
      if (g_imgReceiving) {
        abortImageRx(0);
      }
      return;
    }

    BeginPacket b;
    memcpy(&b, data, sizeof(BeginPacket));

    if (b.magic != PACKET_MAGIC ||
        b.dataLen == 0 ||
        b.dataLen > IMG_MAX_IMAGE_BYTES ||
        b.chunkPayload == 0 ||
        b.chunkPayload > IMG_MAX_CHUNK_PAYLOAD) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG begin_bad_fields magic=%u dataLen=%lu chunkPayload=%u\n",
              (unsigned)b.magic,
              (unsigned long)b.dataLen,
              (unsigned)b.chunkPayload);
      if (g_imgReceiving) {
        abortImageRx(0);
      }
      return;
    }

    // Do not interrupt emission of a completed image.
    if (g_imgEmitActive || g_imgReadyToEmit) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG begin_while_emitting id=%u\n", (unsigned)b.imageId);
      return;
    }

    if (g_imgReceiving) {
      abortImageRx(0);
    }

    freeImageBufferIfAny();
    g_imgBuffer = (uint8_t *)malloc(b.dataLen);
    if (!g_imgBuffer) {
      g_imgBadPackets++;
          IMG_DBG("IMG_DBG malloc_fail bytes=%lu free_heap=%u\n",
            (unsigned long)b.dataLen,
            (unsigned)ESP.getFreeHeap());
      return;
    }

        IMG_DBG("IMG_DBG begin_ok id=%u w=%u h=%u len=%lu chunk=%u\n",
          (unsigned)b.imageId,
          (unsigned)b.width,
          (unsigned)b.height,
          (unsigned long)b.dataLen,
          (unsigned)b.chunkPayload);

    g_imgReceiving = true;
    g_imgImageId = b.imageId;
    g_imgWidth = b.width;
    g_imgHeight = b.height;
    g_imgDataLen = b.dataLen;
    g_imgChunkPayload = b.chunkPayload;
    g_imgExpectedChunk = 0;
    g_imgReceivedBytes = 0;
    return;
  }

  if (type == PKT_CHUNK) {
    if (len < 7) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG chunk_too_short len=%d\n", len);
      if (g_imgReceiving) {
        abortImageRx(0);
      }
      return;
    }

    if (!g_imgReceiving || g_imgBuffer == nullptr) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG chunk_without_begin len=%d\n", len);
      return;
    }

    uint16_t imageId = 0;
    uint16_t chunkIndex = 0;
    uint16_t n = 0;
    memcpy(&imageId, &data[1], sizeof(imageId));
    memcpy(&chunkIndex, &data[3], sizeof(chunkIndex));
    memcpy(&n, &data[5], sizeof(n));

    if (imageId != g_imgImageId || chunkIndex != g_imgExpectedChunk) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG chunk_order_mismatch id=%u/%u chunk=%u expected=%u\n",
              (unsigned)imageId,
              (unsigned)g_imgImageId,
              (unsigned)chunkIndex,
              (unsigned)g_imgExpectedChunk);
      abortImageRx(0);
      return;
    }

    if (n == 0 || n > g_imgChunkPayload || n > IMG_MAX_CHUNK_PAYLOAD || (7 + (int)n) != len) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG chunk_size_mismatch n=%u chunkPayload=%u len=%d expected_len=%u\n",
              (unsigned)n,
              (unsigned)g_imgChunkPayload,
              len,
              (unsigned)(7 + n));
      abortImageRx(0);
      return;
    }

    if (g_imgReceivedBytes + n > g_imgDataLen) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG chunk_overflow recv=%lu n=%u total=%lu\n",
              (unsigned long)g_imgReceivedBytes,
              (unsigned)n,
              (unsigned long)g_imgDataLen);
      abortImageRx(0);
      return;
    }

    memcpy(g_imgBuffer + g_imgReceivedBytes, &data[7], n);
    g_imgReceivedBytes += n;
    g_imgExpectedChunk++;
    return;
  }

  if (type == PKT_END) {
    if ((size_t)len != sizeof(EndPacket)) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG end_bad_size len=%d expected=%u\n", len, (unsigned)sizeof(EndPacket));
      if (g_imgReceiving) {
        abortImageRx(0);
      }
      return;
    }

    if (!g_imgReceiving || g_imgBuffer == nullptr) {
      g_imgBadPackets++;
      IMG_DBG("IMG_DBG end_without_begin len=%d\n", len);
      return;
    }

    EndPacket e;
    memcpy(&e, data, sizeof(EndPacket));

    bool ok = (e.imageId == g_imgImageId) &&
              (e.dataLen == g_imgDataLen) &&
              (g_imgReceivedBytes == g_imgDataLen) &&
              (g_imgExpectedChunk == e.totalChunks);

    if (!ok) {
      g_imgBadPackets++;
          IMG_DBG("IMG_DBG end_validation_fail id=%u/%u dataLen=%lu/%lu recv=%lu chunks=%u/%u\n",
            (unsigned)e.imageId,
            (unsigned)g_imgImageId,
            (unsigned long)e.dataLen,
            (unsigned long)g_imgDataLen,
            (unsigned long)g_imgReceivedBytes,
            (unsigned)e.totalChunks,
            (unsigned)g_imgExpectedChunk);
      abortImageRx(0);
      return;
    }

        IMG_DBG("IMG_DBG end_ok id=%u chunks=%u bytes=%lu\n",
          (unsigned)e.imageId,
          (unsigned)e.totalChunks,
          (unsigned long)e.dataLen);

    // Hand over to loop for paced serial emission.
    g_imgEmitImageId = g_imgImageId;
    g_imgEmitWidth = g_imgWidth;
    g_imgEmitHeight = g_imgHeight;
    g_imgEmitDataLen = g_imgDataLen;
    g_imgReadyToEmit = true;
    g_imgEmitActive = false;
    g_imgEmitOffset = 0;
    g_imgEmitChunkIndex = 0;
    resetImageRxStateNoFree();
    return;
  }

  g_imgBadPackets++;
  IMG_DBG("IMG_DBG unknown_img_type type=%u len=%d\n", (unsigned)type, len);
}

// ============================================================================
// ESP-NOW CALLBACK
// ============================================================================
void OnReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || len <= 0) {
    unknownPackets++;
    return;
  }

  const uint8_t *src = info->src_addr;

  portENTER_CRITICAL_ISR(&g_mux);

  // Route known sensor source.
  if (g_haveSensorMac && macEqual(src, g_sensorMac)) {
    if (len == (int)sizeof(sensor_packet)) {
      sensor_packet pkt;
      memcpy(&pkt, incomingData, sizeof(pkt));
      enqueueSensor(pkt, info->rx_ctrl->rssi);
    } else {
      invalidSizePackets++;
    }
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  // Route known image source.
  if (g_haveImageMac && macEqual(src, g_imageMac)) {
    handleImagePacket(incomingData, len);
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  // First-time learn image source from valid begin packet.
  if (len == (int)sizeof(BeginPacket) && incomingData[0] == PKT_BEGIN) {
    BeginPacket b;
    memcpy(&b, incomingData, sizeof(BeginPacket));
    if (b.magic == PACKET_MAGIC) {
      if (!g_haveImageMac || !macEqual(g_imageMac, src)) {
        g_imagePeerReady = false;
      }
      g_haveImageMac = true;
      macCopy(g_imageMac, src);
      handleImagePacket(incomingData, len);
      portEXIT_CRITICAL_ISR(&g_mux);
      return;
    }
  }

  // First-time learn sensor source by packet size.
  if (len == (int)sizeof(sensor_packet)) {
    sensor_packet pkt;
    memcpy(&pkt, incomingData, sizeof(pkt));
    if (!g_haveSensorMac) {
      g_haveSensorMac = true;
      macCopy(g_sensorMac, src);
    }
    enqueueSensor(pkt, info->rx_ctrl->rssi);
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  // Fallback: likely image source sending chunk/end before learn.
  if (incomingData[0] == PKT_CHUNK || incomingData[0] == PKT_END) {
    if (!g_haveImageMac) {
      g_haveImageMac = true;
      macCopy(g_imageMac, src);
      g_imagePeerReady = false;
    }
    handleImagePacket(incomingData, len);
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  unknownPackets++;
  portEXIT_CRITICAL_ISR(&g_mux);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  WiFi.mode(WIFI_STA);

  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnReceive);

  Serial.println(
    "packetNum,"
    "distance_m,"
    "rssi_dbm,"
    "smoothedRSSI_dbm,"
    "lossRate_pct,"
    "throughput_kbps,"
    "temp_bme280_C,"
    "temp_ds18b20_C,"
    "pressure_hpa,"
    "altitude_m,"
    "velocity_x_ms,"
    "velocity_y_ms,"
    "velocity_z_ms,"
    "accel_x_ms2,"
    "accel_y_ms2,"
    "accel_z_ms2"
  );

  Serial.println("RECEIVER_READY");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  bool emitJustStarted = false;
  static uint32_t nextChunkEmitMs = 0;

  // Keep reverse path ready so ACKs can be sent when image handling completes.
  if (g_haveImageMac && !g_imagePeerReady) {
    ensureImagePeerRegistered();
  }

  // 1) Drain sensor queue first to keep telemetry continuous.
  SensorQueueItem sItem;
  uint16_t processedSensors = 0;
  while (processedSensors < 200) {
    portENTER_CRITICAL(&g_mux);
    bool hasSensor = dequeueSensor(sItem);
    portEXIT_CRITICAL(&g_mux);
    if (!hasSensor) break;
    processedSensors++;

    received++;
    bytesReceived += sizeof(sensor_packet);

    if (lastPacket != 0 && sItem.pkt.packetNumber > lastPacket + 1) {
      lost += (sItem.pkt.packetNumber - lastPacket - 1);
    }
    lastPacket = sItem.pkt.packetNumber;

    smoothedRSSI = alpha * sItem.rssi + (1.0f - alpha) * smoothedRSSI;

    float lossRate = 0;
    if ((received + lost) > 0) {
      lossRate = 100.0f * (float)lost / (float)(received + lost);
    }

    float distance = calculateDistance(smoothedRSSI);

    float throughput = 0;
    if (millis() - lastThroughputCalc > 1000) {
      throughput = (bytesReceived * 8) / 1000.0f;
      bytesReceived = 0;
      lastThroughputCalc = millis();
    }

    bool imageBusy;
    portENTER_CRITICAL(&g_mux);
    imageBusy = g_imgEmitActive || g_imgReadyToEmit || g_imgReceiving;
    portEXIT_CRITICAL(&g_mux);

    // During image RX/emit, avoid telemetry serial output entirely to keep
    // IMG_BEGIN/IMG_CHUNK/IMG_END framing stable on the host.
    if (imageBusy) {
      continue;
    }

    uint16_t activeDivider = SENSOR_PRINT_DIVIDER;
    if (activeDivider == 0) activeDivider = 1;

    sensorPrintCounter++;
    if ((sensorPrintCounter % activeDivider) == 0) {
      Serial.print(sItem.pkt.packetNumber); Serial.print(",");
      Serial.print(distance, 4); Serial.print(",");
      Serial.print(sItem.rssi); Serial.print(",");
      Serial.print(smoothedRSSI, 4); Serial.print(",");
      Serial.print(lossRate, 4); Serial.print(",");
      Serial.print(throughput, 4); Serial.print(",");
      Serial.print(sItem.pkt.temperature_bme280, 4); Serial.print(",");
      Serial.print(sItem.pkt.temperature_ds18b20, 4); Serial.print(",");
      Serial.print(sItem.pkt.pressure_hpa, 4); Serial.print(",");
      Serial.print(sItem.pkt.altitude_m, 4); Serial.print(",");
      Serial.print(sItem.pkt.velocity_x, 4); Serial.print(",");
      Serial.print(sItem.pkt.velocity_y, 4); Serial.print(",");
      Serial.print(sItem.pkt.velocity_z, 4); Serial.print(",");
      Serial.print(sItem.pkt.accel_x, 4); Serial.print(",");
      Serial.print(sItem.pkt.accel_y, 4); Serial.print(",");
      Serial.println(sItem.pkt.accel_z, 4);
    }
  }

  // 2) Emit pending image fail/success end notifications.
  if (g_imgBeginPending) {
    portENTER_CRITICAL(&g_mux);
    bool pending = g_imgBeginPending;
    uint16_t id = g_imgBeginId;
    uint16_t w = g_imgBeginW;
    uint16_t h = g_imgBeginH;
    uint32_t n = g_imgBeginLen;
    g_imgBeginPending = false;
    portEXIT_CRITICAL(&g_mux);

    if (pending) {
      for (uint8_t i = 0; i < IMG_SERIAL_BEGIN_REPEATS; i++) {
        emitBeginFrame(id, w, h, n);
        if (i + 1 < IMG_SERIAL_BEGIN_REPEATS) {
          delay(1);
        }
      }
    }
  }

  if (g_imgEndPending) {
    portENTER_CRITICAL(&g_mux);
    bool pending = g_imgEndPending;
    uint16_t id = g_imgEndId;
    uint8_t ok = g_imgEndOk;
    uint16_t chunks = g_imgEndChunks;
    uint32_t bytesCount = g_imgEndBytes;
    g_imgEndPending = false;
    portEXIT_CRITICAL(&g_mux);

    if (pending) {
      emitEndFrame(id, ok, chunks, bytesCount);
    }
  }

  if (g_imgAckPending) {
    portENTER_CRITICAL(&g_mux);
    bool pending = g_imgAckPending;
    uint16_t id = g_imgAckId;
    uint8_t ok = g_imgAckOk;
    uint16_t chunks = g_imgAckChunks;
    uint32_t bytesCount = g_imgAckBytes;
    g_imgAckPending = false;
    portEXIT_CRITICAL(&g_mux);

    if (pending) {
      sendAckPacket(id, ok, chunks, bytesCount);
    }
  }

  // 3) If full image is ready, emit one IMG_CHUNK line per loop iteration.
  if (g_imgReadyToEmit && !g_imgEmitActive) {
    portENTER_CRITICAL(&g_mux);
    g_imgEmitActive = true;
    g_imgEmitOffset = 0;
    g_imgEmitChunkIndex = 0;
    uint16_t id = g_imgEmitImageId;
    uint16_t w = g_imgEmitWidth;
    uint16_t h = g_imgEmitHeight;
    uint32_t n = g_imgEmitDataLen;
    portEXIT_CRITICAL(&g_mux);

    signalImageBegin(id, w, h, n);
    nextChunkEmitMs = millis() + IMG_SERIAL_CHUNK_INTERVAL_MS;
        IMG_DBG("IMG_DBG emit_begin id=%u w=%u h=%u len=%lu\n",
          (unsigned)id,
          (unsigned)w,
          (unsigned)h,
          (unsigned long)n);
    emitJustStarted = true;
  }

  // Avoid emitting first chunk in the same loop where IMG_BEGIN was scheduled,
  // so the serial stream order is always IMG_BEGIN -> IMG_CHUNK...
  if (g_imgEmitActive && g_imgBuffer != nullptr && !emitJustStarted) {
    uint32_t nowMs = millis();
    if (nowMs < nextChunkEmitMs) {
      delay(1);
      return;
    }

    uint32_t offset;
    uint32_t total;
    uint16_t id;
    uint16_t chunkIndex;

    portENTER_CRITICAL(&g_mux);
    offset = g_imgEmitOffset;
    total = g_imgEmitDataLen;
    id = g_imgEmitImageId;
    chunkIndex = g_imgEmitChunkIndex;
    portEXIT_CRITICAL(&g_mux);

    if (offset < total) {
      uint16_t n = IMG_SERIAL_CHUNK_BYTES;
      if (offset + n > total) {
        n = (uint16_t)(total - offset);
      }

      for (uint8_t dup = 0; dup < IMG_SERIAL_CHUNK_DUPLICATES; dup++) {
        emitChunkFrame(id, chunkIndex, n, g_imgBuffer + offset);
      }
      nextChunkEmitMs = millis() + IMG_SERIAL_CHUNK_INTERVAL_MS;

      portENTER_CRITICAL(&g_mux);
      g_imgEmitOffset += n;
      g_imgEmitChunkIndex++;
      portEXIT_CRITICAL(&g_mux);
    }

    bool done = false;
    uint16_t endChunks = 0;
    uint32_t endBytes = 0;
    portENTER_CRITICAL(&g_mux);
    if (g_imgEmitOffset >= g_imgEmitDataLen) {
      done = true;
      endChunks = g_imgEmitChunkIndex;
      endBytes = g_imgEmitDataLen;
      g_imgEmitActive = false;
      g_imgReadyToEmit = false;
      g_imgOkImages++;
    }
    portEXIT_CRITICAL(&g_mux);

    if (done) {
      signalImageEnd(id, 1, endChunks, endBytes);
      signalImageAck(id, 1, endChunks, endBytes);
      IMG_DBG("IMG_DBG emit_end id=%u chunks=%u bytes=%lu\n",
              (unsigned)id,
              (unsigned)endChunks,
              (unsigned long)endBytes);
      freeImageBufferIfAny();
      resetImageRxStateNoFree();
      portENTER_CRITICAL(&g_mux);
      g_imgEmitImageId = 0;
      g_imgEmitWidth = 0;
      g_imgEmitHeight = 0;
      g_imgEmitDataLen = 0;
      portEXIT_CRITICAL(&g_mux);
    }
  }

  // 4) Periodic status
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  bool serialImageBusy;
  portENTER_CRITICAL(&g_mux);
  serialImageBusy = g_imgEmitActive || g_imgReadyToEmit || g_imgReceiving;
  portEXIT_CRITICAL(&g_mux);

  if (STATUS_PRINT_ENABLE && !serialImageBusy && (now - lastStatus > 2000)) {
    lastStatus = now;

    if (g_haveSensorMac) printMac("SENSOR_MAC", g_sensorMac);
    if (g_haveImageMac) printMac("IMAGE_MAC", g_imageMac);

    Serial.printf("RECEIVER_STATUS sensor_rx=%lu sensor_drop=%lu img_ok=%lu img_bad=%lu unknown=%lu\n",
                  (unsigned long)received,
                  (unsigned long)g_sensorQueueDrops,
                  (unsigned long)g_imgOkImages,
                  (unsigned long)g_imgBadPackets,
                  (unsigned long)unknownPackets);
  }

  delay(1);
}
