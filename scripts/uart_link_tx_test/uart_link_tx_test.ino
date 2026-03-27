#include <Arduino.h>

// Temporary UART link transmitter test for XIAO ESP32S3 Sense.
// Upload this sketch to one board, and uart_link_rx_test.ino to the other.

static const int TX_PIN = D6;
static const int RX_PIN = D7;
static const uint32_t LINK_BAUD = 115200;

HardwareSerial Link(1);

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("[UART-TX] Starting transmitter test");
  Serial.printf("[UART-TX] TX=%d RX=%d LINK_BAUD=%lu\n", TX_PIN, RX_PIN, (unsigned long)LINK_BAUD);

  Link.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Link.setRxBufferSize(1024);
}

void loop() {
  static uint32_t seq = 0;

  // Sync header + sequence makes dropped/corrupted traffic obvious.
  uint8_t pkt[8];
  pkt[0] = 0x55;
  pkt[1] = 0xAA;
  pkt[2] = (uint8_t)(seq & 0xFF);
  pkt[3] = (uint8_t)((seq >> 8) & 0xFF);
  pkt[4] = (uint8_t)((seq >> 16) & 0xFF);
  pkt[5] = (uint8_t)((seq >> 24) & 0xFF);
  pkt[6] = 0x0D;
  pkt[7] = 0x0A;

  size_t n = Link.write(pkt, sizeof(pkt));
  Link.flush();

  Serial.printf("[UART-TX] Sent seq=%lu bytes=%u\n", (unsigned long)seq, (unsigned)n);
  seq++;
  delay(200);
}
