#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ================= CONFIG =================
static const uint8_t WIFI_CHANNEL = 6;

static const uint16_t PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;

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

// ================= RECEPTION STATE =================
static volatile bool g_receiving = false;
static volatile uint16_t g_imageId = 0;
static volatile uint16_t g_width = 0;
static volatile uint16_t g_height = 0;
static volatile uint32_t g_dataLen = 0;
static volatile uint16_t g_chunkPayload = 0;
static volatile uint16_t g_expectedChunk = 0;
static volatile uint32_t g_receivedBytes = 0;

static volatile uint32_t g_okImages = 0;
static volatile uint32_t g_badPackets = 0;

// ================= SERIAL FRAMING =================
// Protocol to PC over USB serial:
// 1) ASCII line: IMG_BEGIN <id> <w> <h> <len>
// 2) Exactly <len> raw grayscale bytes
// 3) ASCII line: IMG_END <id> <ok:0|1> <chunks> <bytes>

void beginNewImage(uint16_t imageId,
                   uint16_t width,
                   uint16_t height,
                   uint32_t dataLen,
                   uint16_t chunkPayload) {
  g_receiving = true;
  g_imageId = imageId;
  g_width = width;
  g_height = height;
  g_dataLen = dataLen;
  g_chunkPayload = chunkPayload;
  g_expectedChunk = 0;
  g_receivedBytes = 0;

  Serial.printf("IMG_BEGIN %u %u %u %lu\n",
                (unsigned)g_imageId,
                (unsigned)g_width,
                (unsigned)g_height,
                (unsigned long)g_dataLen);
}

void abortImage() {
  if (g_receiving) {
    Serial.printf("IMG_END %u 0 %u %lu\n",
                  (unsigned)g_imageId,
                  (unsigned)g_expectedChunk,
                  (unsigned long)g_receivedBytes);
  }

  g_receiving = false;
  g_imageId = 0;
  g_width = 0;
  g_height = 0;
  g_dataLen = 0;
  g_chunkPayload = 0;
  g_expectedChunk = 0;
  g_receivedBytes = 0;
}

void completeImage(uint16_t totalChunks) {
  bool ok = (g_receivedBytes == g_dataLen) && (g_expectedChunk == totalChunks);
  Serial.printf("IMG_END %u %u %u %lu\n",
                (unsigned)g_imageId,
                ok ? 1u : 0u,
                (unsigned)g_expectedChunk,
                (unsigned long)g_receivedBytes);

  if (ok) {
    g_okImages++;
  }

  g_receiving = false;
  g_imageId = 0;
  g_width = 0;
  g_height = 0;
  g_dataLen = 0;
  g_chunkPayload = 0;
  g_expectedChunk = 0;
  g_receivedBytes = 0;
}

// ================= ESP-NOW CALLBACK =================
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;

  if (len <= 0 || data == nullptr) {
    g_badPackets++;
    return;
  }

  uint8_t type = data[0];

  if (type == PKT_BEGIN) {
    if ((size_t)len != sizeof(BeginPacket)) {
      g_badPackets++;
      abortImage();
      return;
    }

    BeginPacket b;
    memcpy(&b, data, sizeof(BeginPacket));

    if (b.magic != PACKET_MAGIC || b.dataLen == 0 || b.chunkPayload == 0) {
      g_badPackets++;
      abortImage();
      return;
    }

    if (g_receiving) {
      // New image arrived before finishing current one: abort old and start new.
      abortImage();
    }

    beginNewImage(b.imageId, b.width, b.height, b.dataLen, b.chunkPayload);
    return;
  }

  if (type == PKT_CHUNK) {
    // Packet layout: [type:1][imageId:2][chunkIndex:2][n:2][payload:n]
    if (len < 7) {
      g_badPackets++;
      abortImage();
      return;
    }

    if (!g_receiving) {
      g_badPackets++;
      return;
    }

    uint16_t imageId = 0;
    uint16_t chunkIndex = 0;
    uint16_t n = 0;

    memcpy(&imageId, &data[1], sizeof(imageId));
    memcpy(&chunkIndex, &data[3], sizeof(chunkIndex));
    memcpy(&n, &data[5], sizeof(n));

    if (imageId != g_imageId || chunkIndex != g_expectedChunk) {
      g_badPackets++;
      abortImage();
      return;
    }

    if (n == 0 || n > g_chunkPayload || (7 + (int)n) != len) {
      g_badPackets++;
      abortImage();
      return;
    }

    if (g_receivedBytes + n > g_dataLen) {
      g_badPackets++;
      abortImage();
      return;
    }

    // Forward only raw image bytes to host script.
    Serial.write(&data[7], n);

    g_receivedBytes += n;
    g_expectedChunk++;
    return;
  }

  if (type == PKT_END) {
    if ((size_t)len != sizeof(EndPacket)) {
      g_badPackets++;
      abortImage();
      return;
    }

    if (!g_receiving) {
      g_badPackets++;
      return;
    }

    EndPacket e;
    memcpy(&e, data, sizeof(EndPacket));

    if (e.imageId != g_imageId || e.dataLen != g_dataLen) {
      g_badPackets++;
      abortImage();
      return;
    }

    completeImage(e.totalChunks);
    return;
  }

  g_badPackets++;
}

bool initEspNowReceiver() {
  WiFi.mode(WIFI_STA);

  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  if (err != ESP_OK) return false;

  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  if (err != ESP_OK) return false;

  err = esp_wifi_set_max_tx_power(78);
  if (err != ESP_OK) return false;

  err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) return false;

  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onEspNowReceive);
  return true;
}

void setup() {
  Serial.begin(2000000);
  delay(300);

  if (!initEspNowReceiver()) {
    Serial.println("RECEIVER_INIT_FAIL");
    while (true) delay(1000);
  }

  Serial.println("RECEIVER_READY");
}

void loop() {
  static uint32_t lastStatus = 0;
  uint32_t now = millis();

  if (!g_receiving && (now - lastStatus > 2000)) {
    lastStatus = now;
    Serial.printf("RECEIVER_STATUS ok=%lu bad=%lu receiving=%u\n",
                  (unsigned long)g_okImages,
                  (unsigned long)g_badPackets,
                  g_receiving ? 1u : 0u);
  }

  delay(2);
}
