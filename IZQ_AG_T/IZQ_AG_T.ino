#include <Arduino.h>
#include "esp_camera.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int TX_PIN_PROFILE_A = D7;  // Right RX pin when boards are wired D7<->D7
static const int RX_PIN_PROFILE_A = D6;  // Right TX pin when boards are wired D6<->D6
static const int TX_PIN_PROFILE_B = D6;  // Right RX pin when boards are cross-wired TX->RX
static const int RX_PIN_PROFILE_B = D7;  // Right TX pin when boards are cross-wired TX->RX
static const bool START_WITH_PROFILE_A = true;
//static const uint32_t BAUD = 921600;
static const uint32_t BAUD = 115200;
static const uint32_t UART_PROFILE_SWITCH_MS = 1200;

// Protocolo
static const uint16_t MAGIC = 0xA55A;
static const uint8_t READY_BYTE     = 0x52; // 'R'
static const uint8_t ACK_HDR_BYTE   = 0x48; // 'H'
static const uint8_t ACK_CHUNK_BYTE = 0x43; // 'C'
static const uint8_t ACK_FINAL_BYTE = 0x06; // ACK
static const uint8_t ERR_BYTE       = 0x15; // NAK

static const uint16_t CHUNK_SIZE = 512;

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
static bool g_useProfileA = START_WITH_PROFILE_A;
static uint32_t g_lastUartActivityMs = 0;

void beginLink(bool useProfileA) {
  int rxPin = useProfileA ? RX_PIN_PROFILE_A : RX_PIN_PROFILE_B;
  int txPin = useProfileA ? TX_PIN_PROFILE_A : TX_PIN_PROFILE_B;

  Link.end();
  Link.begin(BAUD, SERIAL_8N1, rxPin, txPin);

  Serial.printf("[IZQ] UART profile %c active: TX=D%d RX=D%d BAUD=%lu\n",
                useProfileA ? 'A' : 'B',
                txPin,
                rxPin,
                BAUD);
}

void maybeSwitchUartProfile() {
  if (g_busy) return;
  if ((millis() - g_lastUartActivityMs) < UART_PROFILE_SWITCH_MS) return;

  g_useProfileA = !g_useProfileA;
  beginLink(g_useProfileA);
  g_lastUartActivityMs = millis();
}

bool waitForAnyByte(uint8_t *outByte, uint32_t timeoutMs) {
  if (!outByte) return false;
  uint32_t t0 = millis();
  uint32_t checkCount = 0;
  
  while (millis() - t0 < timeoutMs) {
    int avail = Link.available();
    checkCount++;
    
    if (avail > 0) {
      *outByte = (uint8_t)Link.read();
      g_lastUartActivityMs = millis();
      Serial.printf("[IZQ] waitForAnyByte: got 0x%02X after %lu checks\n", *outByte, checkCount);
      return true;
    }
    delay(1);
  }
  
  if (checkCount > 0) {
    Serial.printf("[IZQ] waitForAnyByte timeout after %lu checks\n", checkCount);
  }
  return false;
}

static void write_u16_le(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
}

static void write_u32_le(uint8_t *p, uint32_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

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
  //c.pixel_format = PIXFORMAT_JPEG;
  c.pixel_format = PIXFORMAT_RGB565;

  // Para depurar transferencia, VGA es más seguro que SVGA
  c.frame_size   = FRAMESIZE_QVGA;
  c.jpeg_quality = 10;
  c.fb_count     = 1;

  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location  = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
    c.frame_size  = FRAMESIZE_QVGA;
    c.fb_location = CAMERA_FB_IN_DRAM;
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

bool waitByte(uint8_t expected, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint32_t checkCount = 0;
  uint32_t bytesReceived = 0;
  Serial.printf("[IZQ] waitByte(0x%02X) starting, timeout=%lu ms\n", expected, timeoutMs);
  
  while (millis() - t0 < timeoutMs) {
    int avail = Link.available();
    checkCount++;
    
    if (avail > 0) {
      bytesReceived++;
      g_lastUartActivityMs = millis();
      Serial.printf("[IZQ] Check #%lu: Link.available()=%d\n", checkCount, avail);
      uint8_t b = Link.read();
      if (b == expected) {
        Serial.printf("[IZQ] Got expected byte 0x%02X\n", b);
        return true;
      }
      Serial.printf("[IZQ] Got 0x%02X (expected 0x%02X)\n", b, expected);
    } else {
      delay(1);
    }
  }
  
  Serial.printf("[IZQ] waitByte timeout: checked %lu times, received %lu bytes\n", checkCount, bytesReceived);
  return false;
}


bool sendHeader(camera_fb_t *fb) {
  uint8_t hdr[10];
  write_u16_le(&hdr[0], MAGIC);
  write_u16_le(&hdr[2], fb->width);
  write_u16_le(&hdr[4], fb->height);
  write_u32_le(&hdr[6], fb->len);
  return Link.write(hdr, sizeof(hdr)) == sizeof(hdr);
}

bool sendFrameChunked(camera_fb_t *fb) {
  if (!sendHeader(fb)) {
    Serial.println("[IZQ] Error enviando header");
    return false;
  }

  if (!waitByte(ACK_HDR_BYTE, 3000)) {
    Serial.println("[IZQ] No llegó ACK_HDR");
    return false;
  }

  uint32_t sent = 0;
  while (sent < fb->len) {
    uint16_t n = (uint16_t)min<uint32_t>(CHUNK_SIZE, fb->len - sent);

    uint8_t lenBuf[2];
    write_u16_le(lenBuf, n);

    if (Link.write(lenBuf, 2) != 2) return false;
    if (Link.write(fb->buf + sent, n) != n) return false;
    Link.flush();

    if (!waitByte(ACK_CHUNK_BYTE, 3000)) {
      Serial.printf("[IZQ] No llegó ACK_CHUNK en offset=%lu\n", (unsigned long)sent);
      return false;
    }

    sent += n;
  }

  return true;
}

void captureAndSend(bool waitReadyFirst) {
  if (g_busy) return;
  g_busy = true;

  if (waitReadyFirst) {
    Serial.println("\n[IZQ] READY detected, capturing...");
  } else {
    Serial.println("\n[IZQ] Trigger detected");
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[IZQ] Error capturing image");
    if (!waitReadyFirst) waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[IZQ] Capture ready, %u bytes\n", (unsigned)fb->len);

  if (!waitReadyFirst) {
    Serial.println("[IZQ] Waiting READY...");
    if (!waitByte(READY_BYTE, 15000)) {
      Serial.println("[IZQ] READY not received");
      esp_camera_fb_return(fb);
      waitButtonRelease();
      g_busy = false;
      return;
    }
  }

  Serial.println("[IZQ] Sending image chunks...");

  if (!sendFrameChunked(fb)) {
    Serial.println("[IZQ] Error sending image");
    esp_camera_fb_return(fb);
    if (!waitReadyFirst) waitButtonRelease();
    g_busy = false;
    return;
  }

  esp_camera_fb_return(fb);

  if (waitByte(ACK_FINAL_BYTE, 12000)) {
    Serial.println("[IZQ] Image sent and confirmed");
  } else if (waitByte(ERR_BYTE, 300)) {
    Serial.println("[IZQ] Right camera reported error");
  } else {
    Serial.println("[IZQ] Final ACK not received");
  }

  if (!waitReadyFirst) waitButtonRelease();
  g_busy = false;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Link.setRxBufferSize(2048);
  beginLink(g_useProfileA);
  g_lastUartActivityMs = millis();

  if (!initCamera()) {
    Serial.println("Error al iniciar cámara izquierda");
    while (true) delay(1000);
  }

  Serial.println("XIAO izquierda lista");
}

void loop() {
  if (g_busy) return;

  maybeSwitchUartProfile();

  if (buttonPressed()) {
    captureAndSend(false);
    return;
  }

  uint8_t b = 0;
  if (waitForAnyByte(&b, 5)) {
    if (b == READY_BYTE) {
      captureAndSend(true);
    } else {
      Serial.printf("[IZQ] UART byte=0x%02X\n", b);
    }
  }
}