#include <Arduino.h>
#include "esp_camera.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int TX_PIN  = D6;
static const int RX_PIN  = D7;
static const uint32_t BAUD = 921600;

// Capture mode:
// true  = keep capturing/sending continuously
// false = capture only on button press
static const bool CONTINUOUS_CAPTURE_MODE = true;

// Protocolo
static const uint16_t MAGIC = 0xA55A;
static const uint8_t READY_BYTE     = 0x52; // 'R'
static const uint8_t ACK_HDR_BYTE   = 0x48; // 'H'
static const uint8_t ACK_CHUNK_BYTE = 0x43; // 'C'
static const uint8_t ACK_FINAL_BYTE = 0x06; // ACK
static const uint8_t ERR_BYTE       = 0x15; // NAK

static const uint16_t CHUNK_SIZE = 256;

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

  // Keep left camera resolution aligned with DER to avoid header mismatch.
  c.frame_size   = FRAMESIZE_HQVGA;
  //c.jpeg_quality = 20;
  c.fb_count     = 1;

  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location  = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
  c.frame_size  = FRAMESIZE_HQVGA;
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
  while (millis() - t0 < timeoutMs) {
    if (Link.available()) {
      uint8_t b = Link.read();
      if (b == expected) return true;
    } else {
      delay(1);
    }
  }
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

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.printf("[IZQ] continuous_mode=%u\n", (unsigned)CONTINUOUS_CAPTURE_MODE);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Link.setRxBufferSize(2048);
  Link.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  if (!initCamera()) {
    Serial.println("Error al iniciar cámara izquierda");
    while (true) delay(1000);
  }

  Serial.println("XIAO izquierda lista");
}

void loop() {
  if (g_busy) return;
  if (!CONTINUOUS_CAPTURE_MODE && !buttonPressed()) return;

  g_busy = true;
  while (Link.available()) Link.read();

  Serial.println("\n[IZQ] Trigger detectado");

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[IZQ] Error capturando imagen");
    if (!CONTINUOUS_CAPTURE_MODE) waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[IZQ] Captura lista, %u bytes\n", (unsigned)fb->len);
  Serial.println("[IZQ] Esperando READY...");

  if (!waitByte(READY_BYTE, 8000)) {
    Serial.println("[IZQ] No llegó READY");
    esp_camera_fb_return(fb);
    if (!CONTINUOUS_CAPTURE_MODE) waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.println("[IZQ] READY recibido, enviando imagen por bloques...");

  if (!sendFrameChunked(fb)) {
    Serial.println("[IZQ] Error enviando imagen");
    esp_camera_fb_return(fb);
    if (!CONTINUOUS_CAPTURE_MODE) waitButtonRelease();
    g_busy = false;
    return;
  }

  esp_camera_fb_return(fb);

  if (waitByte(ACK_FINAL_BYTE, 8000)) {
    Serial.println("[IZQ] Imagen enviada y guardada correctamente");
  } else if (waitByte(ERR_BYTE, 300)) {
    Serial.println("[IZQ] La derecha reportó error");
  } else {
    Serial.println("[IZQ] No llegó ACK final");
  }

  if (!CONTINUOUS_CAPTURE_MODE) waitButtonRelease();
  g_busy = false;
}