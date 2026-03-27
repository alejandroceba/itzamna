#include <Arduino.h>

// Temporary UART link receiver test for XIAO ESP32S3 Sense.
// Upload this sketch to one board, and uart_link_tx_test.ino to the other.

static const int TX_PIN = D6;
static const int RX_PIN = D7;
static const uint32_t LINK_BAUD = 115200;

HardwareSerial Link(1);

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("[UART-RX] Starting receiver test");
  Serial.printf("[UART-RX] TX=%d RX=%d LINK_BAUD=%lu\n", TX_PIN, RX_PIN, (unsigned long)LINK_BAUD);

  Link.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Link.setRxBufferSize(2048);
}

void loop() {
  static uint32_t total = 0;
  static uint32_t line = 0;

  while (Link.available()) {
    uint8_t b = (uint8_t)Link.read();
    total++;
    line++;

    Serial.printf("%02X ", b);

    if (line >= 16) {
      Serial.printf(" | total=%lu\n", (unsigned long)total);
      line = 0;
    }
  }

  delay(5);
}
