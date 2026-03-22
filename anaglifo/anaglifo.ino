#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ================= CONFIG =================
static const int BTN_PIN = D2;
static const int SD_CS = 21;

/*
// UART (kept for future ESP-NOW/UART workflow)
static const int TX_PIN = D6;
static const int RX_PIN = D7;
static const uint32_t BAUD = 2000000;

// Protocolo (kept for future left-image receive flow)
// Header: [MAGIC(2)][WIDTH(2)][HEIGHT(2)][LEN(4)]
static const uint16_t MAGIC = 0xA55A;
static const uint8_t READY_BYTE     = 0x52; // 'R'
static const uint8_t ACK_HDR_BYTE   = 0x48; // 'H'
static const uint8_t ACK_CHUNK_BYTE = 0x43; // 'C'
static const uint8_t ACK_FINAL_BYTE = 0x06; // ACK
static const uint8_t ERR_BYTE       = 0x15; // NAK
static const uint16_t CHUNK_SIZE = 512;
*/

// ================= EXPOSURE / GAIN =================
static const int AEC_VALUE = 450;
static const int AGC_GAIN = 10;

/*
HardwareSerial Link(1);
*/

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

/*
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
*/

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
	SPI.begin(7, 8, 9, SD_CS);
	if (!SD.begin(SD_CS, SPI)) return false;
	if (SD.cardType() == CARD_NONE) return false;
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

/*
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
*/

// ================= GRAYSCALE BUFFER =================
bool buildGrayBufferFromFrame(camera_fb_t *fb, uint8_t **outGray) {
	if (!fb || !outGray) return false;

	uint32_t expectedLen = (uint32_t)fb->width * fb->height * 2;
	if (fb->len != expectedLen) {
		Serial.printf("[DER] Unexpected length in right frame: len=%u expected=%u\n",
									(unsigned)fb->len, (unsigned)expectedLen);
		return false;
	}

	uint32_t totalPixels = (uint32_t)fb->width * fb->height;
	uint8_t *gray = (uint8_t*)malloc(totalPixels);
	if (!gray) {
		Serial.println("[DER] Could not allocate rightGray");
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

/*
// ================= RECEIVE LEFT + BUILD ANAGLYPH =================
bool receiveLeftAndBuildOutputs(const char *leftPath,
																const char *anaglyphPath,
																uint16_t width,
																uint16_t height,
																uint32_t totalLen,
																const uint8_t *rightGray) {
	// Original dual-camera/anaglyph flow kept here for next steps.
	// Intentionally commented for single-camera save test.
	return false;
}
*/

// ================= SETUP =================
void setup() {
	Serial.begin(115200);
	delay(800);

	pinMode(BTN_PIN, INPUT_PULLUP);

	/*
	Link.setRxBufferSize(4096);
	Link.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
	*/

	if (!initCamera()) {
		Serial.println("Error initializing camera");
		while (true) delay(1000);
	}

	if (!initSD()) {
		Serial.println("Error initializing SD");
		while (true) delay(1000);
	}

	Serial.println("Single-camera capture+save mode ready");
}

// ================= LOOP =================
void loop() {
	if (g_busy) return;
	if (!buttonPressed()) return;

	g_busy = true;

	uint32_t tStart = millis();
	uint16_t idx = nextIndex();

	char rightName[40];
	snprintf(rightName, sizeof(rightName), "/Right_%04u.pgm", idx);
	String rightPath = String(rightName);

	/*
	char leftName[40];
	char anaName[40];
	snprintf(leftName, sizeof(leftName), "/Left_%04u.pgm", idx);
	snprintf(anaName, sizeof(anaName), "/Anaglyph_%04u.ppm", idx);
	*/

	Serial.println("\n[DER] Trigger detected");

	camera_fb_t *fb = esp_camera_fb_get();
	if (!fb) {
		Serial.println("[DER] Error capturing right image");
		waitButtonRelease();
		g_busy = false;
		return;
	}

	Serial.printf("[DER] width=%u height=%u len=%u format=%u\n",
								fb->width, fb->height, fb->len, fb->format);

	uint8_t *rightGray = nullptr;
	if (!buildGrayBufferFromFrame(fb, &rightGray)) {
		Serial.println("[DER] Error building rightGray");
		esp_camera_fb_return(fb);
		waitButtonRelease();
		g_busy = false;
		return;
	}

	if (!saveGrayPGM(rightPath.c_str(), rightGray, fb->width, fb->height)) {
		Serial.println("[DER] Error saving right image");
		free(rightGray);
		esp_camera_fb_return(fb);
		waitButtonRelease();
		g_busy = false;
		return;
	}

	esp_camera_fb_return(fb);
	free(rightGray);

	Serial.printf("[DER] Right image saved to %s\n", rightPath.c_str());

	/*
	// Original flow kept for next step (do not execute yet):
	// 1) Link.write(READY_BYTE)
	// 2) Receive header from other XIAO
	// 3) Receive left image chunks over UART
	// 4) Build anaglyph and save left/anaglyph files
	// 5) Send ACK_FINAL_BYTE
	*/

	uint32_t elapsed = millis() - tStart;
	Serial.printf("[DER] Capture+save time: %lu ms (%.2f s)\n",
								(unsigned long)elapsed,
								elapsed / 1000.0f);

	waitButtonRelease();
	g_busy = false;
}
