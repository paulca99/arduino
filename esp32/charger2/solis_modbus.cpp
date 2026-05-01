/*
 * solis_modbus.cpp — Solis RS485/Modbus polling implementation
 */
#include "solis_modbus.h"
#include <HardwareSerial.h>

// ── Internals ──────────────────────────────────────────────────────
static HardwareSerial RS485(2);  // UART2
static SolisData _solis = {};    // shared struct
SemaphoreHandle_t solisMutex;

#define SOLIS_ID         1        // Modbus device address
#define POLL_DELAY_MS    500      // pause between full poll cycles
#define REPLY_TIMEOUT_MS 200      // max ms to wait for each reply

// ── Modbus CRC-16 ──────────────────────────────────────────────────
static uint16_t modbusCRC(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// ── Modbus request frame ───────────────────────────────────────────
static void sendRequest(uint16_t startReg, uint16_t count) {
#if RS485_DE_PIN >= 0
  digitalWrite(RS485_DE_PIN, HIGH);  // drive enable
  delayMicroseconds(100);
#endif

  uint8_t frame[8];
  frame[0] = SOLIS_ID;
  frame[1] = 0x03;  // Read Holding Registers
  frame[2] = startReg >> 8;
  frame[3] = startReg & 0xFF;
  frame[4] = count >> 8;
  frame[5] = count & 0xFF;
  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
  RS485.write(frame, 8);
  RS485.flush();  // wait for TX to complete

#if RS485_DE_PIN >= 0
  delayMicroseconds(100);
  digitalWrite(RS485_DE_PIN, LOW);   // back to receive
#endif
}

// ── Non-blocking reply reader ──────────────────────────────────────
static bool readReply(uint8_t* buf, int expected) {
  int len = 0;
  unsigned long start = millis();
  while (millis() - start < REPLY_TIMEOUT_MS) {
    while (RS485.available() && len < expected) {
      buf[len++] = (uint8_t)RS485.read();
    }
    if (len >= expected) return true;
    vTaskDelay(1);  // yield — keeps RTOS responsive
  }
  return (len >= expected);
}

// ── Decode Modbus block 1 (registers 0x3100–0x3127) ───────────────
// Response layout (85 bytes for 40 regs):
//   [0]   = device addr
//   [1]   = function code (0x03)
//   [2]   = byte count (0x50 = 80)
//   [3..82] = register data (40 × 2 bytes, high byte first)
//   [83..84] = CRC
static void decodeBlock1(const uint8_t* b) {
  _solis.pv1Voltage    = (b[3]  << 8 | b[4])  / 10.0f;
  _solis.pv1Current    = (b[5]  << 8 | b[6])  / 10.0f;
  _solis.pv2Voltage    = (b[7]  << 8 | b[8])  / 10.0f;
  _solis.pv2Current    = (b[9]  << 8 | b[10]) / 10.0f;
  _solis.batteryVoltage = (b[15] << 8 | b[16]) / 10.0f;
  _solis.batteryCurrent = (int16_t)(b[17] << 8 | b[18]) / 10.0f;
  _solis.batteryPower   = (int16_t)(b[19] << 8 | b[20]);
  _solis.inverterTemp   = (b[35] << 8 | b[36]) / 10.0f;
}

// ── Decode Modbus block 2 (registers 0x3200–0x3227) ───────────────
static void decodeBlock2(const uint8_t* b) {
  _solis.gridVoltage  = (b[3]  << 8 | b[4])  / 10.0f;
  _solis.gridCurrent  = (b[5]  << 8 | b[6])  / 10.0f;
  _solis.gridPower    = (int16_t)(b[7]  << 8 | b[8]);
  _solis.frequency    = (b[9]  << 8 | b[10]) / 100.0f;
  _solis.powerFactor  = (int16_t)(b[11] << 8 | b[12]) / 1000.0f;
  _solis.totalPower   = (int16_t)(b[13] << 8 | b[14]);
  _solis.dailyEnergy  = (b[23] << 8 | b[24]) / 100.0f;
  _solis.totalEnergy  = (b[25] << 8 | b[26]) / 10.0f;
}

// ── Public functions ───────────────────────────────────────────────

void solisModbusSetup() {
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  solisMutex = xSemaphoreCreateMutex();

#if RS485_DE_PIN >= 0
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
#endif

  _solis = {};  // zero-initialise
}

void solisModbusTask(void* pv) {
  uint8_t buf[100];
  bool b1ok, b2ok;

  for (;;) {
    // ── Block 1: 0x3100, 40 registers ──
    while (RS485.available()) RS485.read();  // flush stale bytes
    sendRequest(0x3100, 40);
    b1ok = readReply(buf, 85);
    if (b1ok) {
      if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        decodeBlock1(buf);
        xSemaphoreGive(solisMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // ── Block 2: 0x3200, 40 registers ──
    while (RS485.available()) RS485.read();
    sendRequest(0x3200, 40);
    b2ok = readReply(buf, 85);
    if (b2ok) {
      if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        decodeBlock2(buf);
        _solis.lastUpdateMs = millis();
        _solis.valid        = true;
        xSemaphoreGive(solisMutex);
      }
    }

    if (!b1ok || !b2ok) {
      Serial.printf("[Solis] Poll partial: b1=%d b2=%d\n", b1ok, b2ok);
    }

    vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
  }
}

SolisData getSolisSnapshot() {
  SolisData snap = {};
  if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = _solis;
    xSemaphoreGive(solisMutex);
  }
  return snap;
}

unsigned long solisDataAgeMs() {
  SolisData snap = getSolisSnapshot();
  if (!snap.valid) return 0;
  return millis() - snap.lastUpdateMs;
}
