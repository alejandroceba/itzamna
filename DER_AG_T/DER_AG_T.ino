#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int TX_PIN  = D6;
static const int RX_PIN  = D7;
//static const uint32_t BAUD = 921600;
static const uint32_t BAUD = 115200;
static const int SD_CS = 21;

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

static const uint16_t CHUNK_SIZE = 128;

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

// ================= CONVERSIÓN A GRIS =================
uint8_t rgb565ToGray(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = ( p        & 0x1F) * 255 / 31;

  return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

// ================= RUTAS =================
uint16_t nextIndex() {
  for (uint16_t i = 1; i < 10000; i++) {
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "/Right_%04u.pgm", i);
    if (!SD.exists(tmp)) return i;
  }
  return 9999;
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

// ================= SD =================
bool initSD() {
  SPI.begin(7, 8, 9, SD_CS);
  if (!SD.begin(SD_CS, SPI)) return false;
  if (SD.cardType() == CARD_NONE) return false;
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

bool saveGrayPGM(const char *path, const uint8_t *gray, uint16_t width, uint16_t height) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  f.printf("P5\n%u %u\n255\n", width, height);
  size_t total = (size_t)width * height;
  size_t w = f.write(gray, total);
  f.close();

  return w == total;
}

// ================= RECEPCIÓN IZQUIERDA + ANAGLIFO =================
bool receiveLeftAndBuildOutputs(const char *leftPath,
                                const char *anaglyphPath,
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

  File fLeft = SD.open(leftPath, FILE_WRITE);
  if (!fLeft) {
    Serial.println("[DER] No se pudo abrir archivo izquierdo");
    return false;
  }

  File fAna = SD.open(anaglyphPath, FILE_WRITE);
  if (!fAna) {
    Serial.println("[DER] No se pudo abrir archivo anaglifo");
    fLeft.close();
    return false;
  }

  // Cabeceras
  fLeft.printf("P5\n%u %u\n255\n", width, height);
  fAna.printf("P6\n%u %u\n255\n", width, height);

  uint32_t received = 0;
  uint32_t pixelIndex = 0;
  uint32_t chunkCount = 0;

  while (received < totalLen) {
    if (!waitForChunkMarker(10000)) {
      Serial.println("[DER] Timeout esperando marcador de bloque");
      fLeft.close();
      fAna.close();
      return false;
    }

    uint8_t lenBuf[2];
    if (!readExact(lenBuf, 2, 10000)) {
      Serial.println("[DER] Timeout leyendo tamaño de bloque");
      fLeft.close();
      fAna.close();
      return false;
    }

    uint16_t n = read_u16_le(lenBuf);

    if (n == 0 || n > CHUNK_SIZE || (received + n) > totalLen) {
      Serial.printf("[DER] Bloque inválido: %u\n", n);
      fLeft.close();
      fAna.close();
      return false;
    }

    if (n % 2 != 0) {
      Serial.println("[DER] Bloque RGB565 impar");
      fLeft.close();
      fAna.close();
      return false;
    }

    uint8_t buf[CHUNK_SIZE];
    if (!readExact(buf, n, 10000)) {
      Serial.println("[DER] Timeout leyendo bloque");
      fLeft.close();
      fAna.close();
      return false;
    }

    size_t pixelCount = n / 2;
    uint8_t leftChunk[CHUNK_SIZE / 2];
    uint8_t anaChunk[(CHUNK_SIZE / 2) * 3];

    for (uint16_t i = 0; i < n; i += 2) {
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t leftGray = rgb565ToGray(p);
      size_t outIndex = i / 2;

      leftChunk[outIndex] = leftGray;

      // Anaglifo:
      // R = izquierda
      // G = derecha
      // B = derecha
      uint8_t rightValue = rightGray[pixelIndex];
      size_t rgbIndex = outIndex * 3;
      anaChunk[rgbIndex + 0] = leftGray;
      anaChunk[rgbIndex + 1] = rightValue;
      anaChunk[rgbIndex + 2] = rightValue;

      pixelIndex++;
    }

    if (fLeft.write(leftChunk, pixelCount) != pixelCount) {
      Serial.println("[DER] Error escribiendo bloque izquierda");
      fLeft.close();
      fAna.close();
      return false;
    }

    if (fAna.write(anaChunk, pixelCount * 3) != pixelCount * 3) {
      Serial.println("[DER] Error escribiendo bloque anaglifo");
      fLeft.close();
      fAna.close();
      return false;
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

  fLeft.close();
  fAna.close();
  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.printf("[DER] UART Link config TX=%d RX=%d BAUD=%lu\n", TX_PIN, RX_PIN, (unsigned long)BAUD);
  Serial.printf("[DER] PROTO %s\n", PROTO_VERSION);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Link.setRxBufferSize(16384);
  Link.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  if (!initCamera()) {
    Serial.println("Error al iniciar cámara derecha");
    while (true) delay(1000);
  }

  if (!initSD()) {
    Serial.println("Error al iniciar SD");
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

  uint16_t idx = nextIndex();

  char rightName[40];
  char leftName[40];
  char anaName[40];

  snprintf(rightName, sizeof(rightName), "/Right_%04u.pgm", idx);
  snprintf(leftName,  sizeof(leftName),  "/Left_%04u.pgm", idx);
  snprintf(anaName,   sizeof(anaName),   "/Anaglyph_%04u.ppm", idx);

  String rightPath = String(rightName);
  String leftPath  = String(leftName);
  String anaPath   = String(anaName);

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

  if (!saveGrayPGM(rightPath.c_str(), rightGray, fb->width, fb->height)) {
    Serial.println("[DER] Error guardando imagen derecha");
    free(rightGray);
    esp_camera_fb_return(fb);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  uint16_t rightW = fb->width;
  uint16_t rightH = fb->height;

  esp_camera_fb_return(fb);

  Serial.printf("[DER] Derecha guardada en %s\n", rightPath.c_str());

  Serial.println("[DER] Esperando header de IZQ...");

  uint8_t hdr[10];
  if (!readExact(hdr, sizeof(hdr), 15000)) {
    Serial.println("[DER] Timeout esperando header de IZQ");
    free(rightGray);
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
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] Header recibido: w=%u h=%u len=%u\n",
                leftW, leftH, (unsigned)totalLen);

  if (leftW != rightW || leftH != rightH) {
    Serial.println("[DER] Dimensiones izquierda/derecha no coinciden");
    free(rightGray);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  if (!receiveLeftAndBuildOutputs(leftPath.c_str(),
                                  anaPath.c_str(),
                                  leftW, leftH, totalLen,
                                  rightGray)) {
    Serial.println("[DER] Error guardando imagen izquierda / anaglifo");
    free(rightGray);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  free(rightGray);

  Serial.printf("[DER] Izquierda guardada en %s\n", leftPath.c_str());
  Serial.printf("[DER] Anaglifo guardado en %s\n", anaPath.c_str());

  uint32_t elapsed = millis() - tStart;
  Serial.printf("[DER] Tiempo total: %lu ms (%.2f s)\n",
              (unsigned long)elapsed,
              elapsed / 1000.0f);
  Serial.println("[DER] Recepción y guardado completados");

  waitButtonRelease();
  g_busy = false;
}