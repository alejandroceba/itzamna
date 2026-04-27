#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int TX_PIN  = D6;
static const int RX_PIN  = D7;
static const uint32_t BAUD = 921600;
static const int SD_CS = 21;

// Protocolo
// Header: [MAGIC(2)][WIDTH(2)][HEIGHT(2)][LEN(4)]
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

// ================= CONVERSIÓN A GRIS =================
// OJO: aquí se interpreta RGB565 como BYTE ALTO primero.
// Esto sigue la misma lógica del código viejo que sí te funcionó.
static inline uint8_t rgb565ToGray(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = ( p        & 0x1F) * 255 / 31;

  return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

// Empaca anaglifo "gris izquierdo + gris derecho" a RGB565:
// R = leftGray
// G = rightGray
// B = rightGray
static inline uint16_t packAnaglyphRGB565(uint8_t leftGray, uint8_t rightGray) {
  uint16_t r = ((uint16_t)(leftGray  >> 3) & 0x1F) << 11;
  uint16_t g = ((uint16_t)(rightGray >> 2) & 0x3F) << 5;
  uint16_t b = ((uint16_t)(rightGray >> 3) & 0x1F);
  return (uint16_t)(r | g | b);
}

// ================= RUTAS =================
uint16_t nextIndex() {
  for (uint16_t i = 1; i < 10000; i++) {
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "/Right_%04u.rgb565", i);
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

  c.frame_size   = FRAMESIZE_HQVGA;
  c.fb_count     = 1;

  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
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

// ================= GUARDAR RGB565 RAW =================
bool saveLocalFrame(const char *path, camera_fb_t *fb) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  size_t w = f.write(fb->buf, fb->len);
  f.close();
  return w == fb->len;
}

// ================= BUFFER GRIS DESDE FRAME =================
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
    // MISMA interpretación que el código viejo:
    uint16_t p = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
    gray[px++] = rgb565ToGray(p);
  }

  *outGray = gray;
  return true;
}

// ================= RECEPCIÓN IZQUIERDA + ANAGLIFO RGB565 =================
bool receiveLeftAndBuildOutputs(const char *leftPath,
                                const char *anaglyphRGB565Path,
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

  File fAna = SD.open(anaglyphRGB565Path, FILE_WRITE);
  if (!fAna) {
    Serial.println("[DER] No se pudo abrir archivo anaglifo RGB565");
    fLeft.close();
    return false;
  }

  Link.write(ACK_HDR_BYTE);
  Link.flush();

  uint32_t received = 0;
  uint32_t pixelIndex = 0;

  while (received < totalLen) {
    uint8_t lenBuf[2];
    if (!readExact(lenBuf, 2, 4000)) {
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
    if (!readExact(buf, n, 4000)) {
      Serial.println("[DER] Timeout leyendo bloque");
      fLeft.close();
      fAna.close();
      return false;
    }

    // Guardar izquierda RAW tal cual llegó
    if (fLeft.write(buf, n) != n) {
      Serial.println("[DER] Error escribiendo Left.raw");
      fLeft.close();
      fAna.close();
      return false;
    }

    // Construir anaglifo usando MISMA lógica visual del código viejo:
    // R = leftGray, G = rightGray, B = rightGray
    for (uint16_t i = 0; i < n; i += 2) {
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t leftGray = rgb565ToGray(p);
      uint8_t rightG = rightGray[pixelIndex];

      uint16_t anaPix = packAnaglyphRGB565(leftGray, rightG);

      // Guardar anaglifo RGB565 en el mismo orden "alto, bajo"
      uint8_t anaBytes[2];
      anaBytes[0] = (anaPix >> 8) & 0xFF;
      anaBytes[1] = anaPix & 0xFF;

      if (fAna.write(anaBytes, 2) != 2) {
        Serial.println("[DER] Error escribiendo anaglifo RGB565");
        fLeft.close();
        fAna.close();
        return false;
      }

      pixelIndex++;
    }

    received += n;

    Link.write(ACK_CHUNK_BYTE);
    Link.flush();
  }

  fLeft.close();
  fAna.close();
  return true;
}

// ================= BMP =================
bool writeBMPHeader(File &f, uint16_t width, uint16_t height) {
  uint32_t rowSize = ((uint32_t)width * 3 + 3) & ~3UL;
  uint32_t pixelDataSize = rowSize * (uint32_t)height;
  uint32_t fileSize = 54 + pixelDataSize;

  uint8_t hdr[54] = {0};

  hdr[0] = 'B';
  hdr[1] = 'M';
  write_u32_le(&hdr[2], fileSize);
  write_u32_le(&hdr[10], 54);
  write_u32_le(&hdr[14], 40);
  write_u32_le(&hdr[18], width);
  write_u32_le(&hdr[22], height);   // bottom-up
  write_u16_le(&hdr[26], 1);
  write_u16_le(&hdr[28], 24);
  write_u32_le(&hdr[34], pixelDataSize);

  return f.write(hdr, sizeof(hdr)) == sizeof(hdr);
}

bool convertRGB565FileToBMP(const char *rgbPath,
                            const char *bmpPath,
                            uint16_t width,
                            uint16_t height) {
  File fIn = SD.open(rgbPath, FILE_READ);
  if (!fIn) {
    Serial.println("[DER] No se pudo abrir Ana.rgb565");
    return false;
  }

  File fOut = SD.open(bmpPath, FILE_WRITE);
  if (!fOut) {
    Serial.println("[DER] No se pudo abrir Ana.bmp");
    fIn.close();
    return false;
  }

  if (!writeBMPHeader(fOut, width, height)) {
    Serial.println("[DER] Error escribiendo header BMP");
    fIn.close();
    fOut.close();
    return false;
  }

  const uint32_t rgbRowBytes = (uint32_t)width * 2;
  const uint32_t bmpRowBytes = ((uint32_t)width * 3 + 3) & ~3UL;

  uint8_t *rgbRow = (uint8_t *)malloc(rgbRowBytes);
  uint8_t *bmpRow = (uint8_t *)malloc(bmpRowBytes);

  if (!rgbRow || !bmpRow) {
    Serial.println("[DER] Sin RAM para convertir BMP");
    if (rgbRow) free(rgbRow);
    if (bmpRow) free(bmpRow);
    fIn.close();
    fOut.close();
    return false;
  }

  for (int row = height - 1; row >= 0; row--) {
    uint32_t offset = (uint32_t)row * rgbRowBytes;
    if (!fIn.seek(offset)) {
      Serial.println("[DER] Error haciendo seek en Ana.rgb565");
      free(rgbRow);
      free(bmpRow);
      fIn.close();
      fOut.close();
      return false;
    }

    if (fIn.read(rgbRow, rgbRowBytes) != (int)rgbRowBytes) {
      Serial.println("[DER] Error leyendo fila de Ana.rgb565");
      free(rgbRow);
      free(bmpRow);
      fIn.close();
      fOut.close();
      return false;
    }

    memset(bmpRow, 0, bmpRowBytes);

    for (uint16_t x = 0; x < width; x++) {
      // MISMA interpretación "alto, bajo"
      uint16_t pix = ((uint16_t)rgbRow[2 * x] << 8) | rgbRow[2 * x + 1];

      uint8_t r5 = (pix >> 11) & 0x1F;
      uint8_t g6 = (pix >> 5)  & 0x3F;
      uint8_t b5 = pix & 0x1F;

      uint8_t r8 = (r5 * 255) / 31;
      uint8_t g8 = (g6 * 255) / 63;
      uint8_t b8 = (b5 * 255) / 31;

      uint32_t j = (uint32_t)x * 3;
      bmpRow[j + 0] = b8;
      bmpRow[j + 1] = g8;
      bmpRow[j + 2] = r8;
    }

    if (fOut.write(bmpRow, bmpRowBytes) != bmpRowBytes) {
      Serial.println("[DER] Error escribiendo fila BMP");
      free(rgbRow);
      free(bmpRow);
      fIn.close();
      fOut.close();
      return false;
    }
  }

  free(rgbRow);
  free(bmpRow);
  fIn.close();
  fOut.close();
  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Link.setRxBufferSize(4096);
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
  char bmpName[40];

  snprintf(rightName, sizeof(rightName), "/Right_%04u.rgb565", idx);
  snprintf(leftName,  sizeof(leftName),  "/Left_%04u.rgb565", idx);
  snprintf(anaName,   sizeof(anaName),   "/Anaglyph_%04u.rgb565", idx);
  snprintf(bmpName,   sizeof(bmpName),   "/Anaglyph_%04u.bmp", idx);

  String rightPath = String(rightName);
  String leftPath  = String(leftName);
  String anaPath   = String(anaName);
  String bmpPath   = String(bmpName);

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

  // Guardar derecha RAW
  if (!saveLocalFrame(rightPath.c_str(), fb)) {
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

  if (!receiveLeftAndBuildOutputs(leftPath.c_str(),
                                  anaPath.c_str(),
                                  leftW, leftH, totalLen,
                                  rightGray)) {
    Serial.println("[DER] Error guardando imagen izquierda / anaglifo");
    free(rightGray);
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  free(rightGray);

  Serial.printf("[DER] Izquierda guardada en %s\n", leftPath.c_str());
  Serial.printf("[DER] Anaglifo RGB565 guardado en %s\n", anaPath.c_str());

  if (!convertRGB565FileToBMP(anaPath.c_str(), bmpPath.c_str(), leftW, leftH)) {
    Serial.println("[DER] Error convirtiendo anaglifo a BMP");
    Link.write(ERR_BYTE);
    waitButtonRelease();
    g_busy = false;
    return;
  }

  Serial.printf("[DER] BMP guardado en %s\n", bmpPath.c_str());

  uint32_t elapsed = millis() - tStart;
  Serial.printf("[DER] Tiempo total: %lu ms (%.2f s)\n",
                (unsigned long)elapsed,
                elapsed / 1000.0f);

  Link.write(ACK_FINAL_BYTE);
  Link.flush();
  Serial.println("[DER] ACK final enviado");

  waitButtonRelease();
  g_busy = false;
}