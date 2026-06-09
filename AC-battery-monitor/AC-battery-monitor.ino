#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <WiFi.h>
#include <WebServer.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets_template.h"
#endif

static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define STARTUP_SCAN_TIMEOUT_MS   15000
#define RECONNECT_SCAN_TIMEOUT_MS  8000
#define SCAN_SLICE_MS              1200
#define MIN_SCAN_SLICE_MS           200
#define BLE_SCAN_INTERVAL_UNITS    1349
#define BLE_SCAN_WINDOW_UNITS       449
#define MS_PER_SECOND              1000
#define CONNECT_DELAY_MS            100
#define DISCONNECT_CLEANUP_DELAY_MS  50
#define REQUEST_DELAY_MS            250
#define RESPONSE_TIMEOUT_MS        2500
#define RESPONSE_POLL_MS             20
#define BETWEEN_BATTERIES_MS        20
#define BETWEEN_CYCLES_MS          400
#define RECONNECT_INTERVAL_MS      8000
#define MAX_PACKET_LEN             128
#define MAX_CELLS                   24
#define PACKET03_SOC_INDEX          23
#define PACKET03_FET_INDEX          24
#define PACKET03_TEMP_COUNT_IDX_A   26
#define PACKET03_TEMP_COUNT_IDX_B   25
#define CELL_PACKET_EVERY_N_CYCLES   5

struct BatteryState {
  const char* name;
  const char* mac;
  bool enabled;

  BLEAdvertisedDevice* advertisedDevice = nullptr;
  BLEClient* client = nullptr;
  BLERemoteService* service = nullptr;
  BLERemoteCharacteristic* rx = nullptr;
  BLERemoteCharacteristic* tx = nullptr;

  bool seen = false;
  bool connected = false;
  bool hasData = false;
  bool hasTemperature = false;
  bool chargeMos = false;
  bool dischargeMos = false;

  unsigned long connectedAtMs = 0;
  unsigned long lastGoodDataMs = 0;
  unsigned long lastRequestMs = 0;
  unsigned long lastDisconnectMs = 0;
  unsigned long nextReconnectMs = 0;
  unsigned long okReads = 0;
  unsigned long failedReads = 0;
  unsigned long disconnectCount = 0;

  float voltage = 0.0f;
  float current = 0.0f;
  float temperature = 0.0f;
  uint8_t soc = 0;

  uint16_t cellMv[MAX_CELLS] = {0};
  uint8_t cellCount = 0;
  bool hasCellData = false;
  unsigned long lastCellDataMs = 0;

  uint8_t packetBuf[MAX_PACKET_LEN] = {0};
  int packetLen = 0;
  int expectedLen = 0;
  bool packetError = false;
  bool gotPacket03 = false;
  bool gotPacket04 = false;
};

static BatteryState batteries[] = {
  {"AC_Solax", "a4:c1:37:20:4e:3b", true},
  {"AC_Growatt", "a5:c2:37:49:c7:a2", true},
};

static const int BATTERY_COUNT = sizeof(batteries) / sizeof(batteries[0]);
static BLEScan* pBLEScan = nullptr;
static WebServer server(80);
static unsigned long loopCount = 0;

static int findBatteryByClient(BLEClient* client);
static int findBatteryByRx(BLERemoteCharacteristic* characteristic);
static bool hasDeadlinePassed(unsigned long deadlineMs);
static uint32_t scanDurationSeconds(unsigned long durationMs);
static int enabledBatteryCount();
static int seenBatteryCount();
static bool isBatteryConnected(const BatteryState& battery);
static void resetPacketAssembly(BatteryState& battery);
static uint16_t calcChecksum(const uint8_t* data, int length);
static bool checksumValid(const uint8_t* data, int length);
static bool parsePacket03(BatteryState& battery);
static bool parsePacket04(BatteryState& battery);
static bool scanForAllEnabledBatteries(unsigned long timeoutMs);
static bool scanForBattery(BatteryState& battery, unsigned long timeoutMs);
static void cleanupBatteryClient(BatteryState& battery);
static bool connectBattery(BatteryState& battery);
static bool requestBatterySnapshot(BatteryState& battery);
static bool requestCellVoltages(BatteryState& battery);
static bool reconnectBattery(BatteryState& battery);
static String buildJsonResponse();
static String boolJson(bool v);
static float computePowerW(const BatteryState& battery);
static uint16_t minCellMv(const BatteryState& battery);
static uint16_t maxCellMv(const BatteryState& battery);

static int findBatteryByClient(BLEClient* client) {
  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (batteries[i].client == client) return i;
  }
  return -1;
}

static int findBatteryByRx(BLERemoteCharacteristic* characteristic) {
  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (batteries[i].rx == characteristic) return i;
  }
  return -1;
}

static bool hasDeadlinePassed(unsigned long deadlineMs) {
  return (int32_t)(millis() - deadlineMs) >= 0;
}

static uint32_t scanDurationSeconds(unsigned long durationMs) {
  return (uint32_t)((durationMs + (MS_PER_SECOND - 1)) / MS_PER_SECOND);
}

static int enabledBatteryCount() {
  int count = 0;
  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (batteries[i].enabled) count++;
  }
  return count;
}

static int seenBatteryCount() {
  int count = 0;
  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (batteries[i].enabled && batteries[i].seen) count++;
  }
  return count;
}

static bool isBatteryConnected(const BatteryState& battery) {
  return battery.enabled && battery.client != nullptr && battery.connected &&
         battery.client->isConnected() && battery.rx != nullptr && battery.tx != nullptr;
}

static void resetPacketAssembly(BatteryState& battery) {
  battery.packetLen = 0;
  battery.expectedLen = 0;
  battery.packetError = false;
  battery.gotPacket03 = false;
  battery.gotPacket04 = false;
}

static uint16_t calcChecksum(const uint8_t* data, int length) {
  if (data == nullptr || length < 7) return 0;

  int checksum = 0x10000;
  int dataLength = data[3];
  if (length < dataLength + 7) return 0;

  for (int i = 0; i < dataLength + 1; i++) checksum -= data[i + 3];
  return (uint16_t)checksum;
}

static bool checksumValid(const uint8_t* data, int length) {
  if (data == nullptr) return false;
  if (length < 7) return false;
  int checksumIndex = data[3] + 4;
  if (checksumIndex + 1 >= length) return false;
  uint16_t got = ((uint16_t)data[checksumIndex] << 8) | data[checksumIndex + 1];
  return calcChecksum(data, length) == got;
}

static bool parsePacket03(BatteryState& battery) {
  if (battery.packetLen <= PACKET03_SOC_INDEX) {
    battery.packetError = true;
    return false;
  }

  uint16_t rawVolts = ((uint16_t)battery.packetBuf[4] << 8) | battery.packetBuf[5];
  int16_t rawCurrent = (int16_t)(((uint16_t)battery.packetBuf[6] << 8) | battery.packetBuf[7]);

  battery.voltage = rawVolts / 100.0f;
  battery.current = rawCurrent / 100.0f;
  battery.soc = battery.packetBuf[PACKET03_SOC_INDEX];

  uint8_t mosStatus = battery.packetLen > PACKET03_FET_INDEX ? battery.packetBuf[PACKET03_FET_INDEX] : 0;
  battery.chargeMos = (mosStatus & 0x01U) != 0;
  battery.dischargeMos = (mosStatus & 0x02U) != 0;

  battery.hasTemperature = false;
  battery.temperature = 0.0f;

  int tempCountIndex = -1;
  if (battery.packetLen > PACKET03_TEMP_COUNT_IDX_A) {
    tempCountIndex = PACKET03_TEMP_COUNT_IDX_A;
  } else if (battery.packetLen > PACKET03_TEMP_COUNT_IDX_B) {
    tempCountIndex = PACKET03_TEMP_COUNT_IDX_B;
  }

  if (tempCountIndex >= 0) {
    uint8_t tempCount = battery.packetBuf[tempCountIndex];
    int tempStart = tempCountIndex + 1;
    if (tempCount > 0 && tempStart + 1 < battery.packetLen) {
      uint16_t rawTemp = ((uint16_t)battery.packetBuf[tempStart] << 8) | battery.packetBuf[tempStart + 1];
      battery.temperature = (rawTemp / 10.0f) - 273.15f;
      battery.hasTemperature = true;
    }
  }

  battery.hasData = true;
  battery.gotPacket03 = true;
  battery.lastGoodDataMs = millis();
  return true;
}

static bool parsePacket04(BatteryState& battery) {
  if (battery.packetLen < 7) {
    battery.packetError = true;
    return false;
  }

  int payloadLen = battery.packetBuf[3];
  if (payloadLen <= 0 || (payloadLen % 2) != 0) {
    battery.packetError = true;
    return false;
  }

  int count = payloadLen / 2;
  if (count > MAX_CELLS) count = MAX_CELLS;

  battery.cellCount = count;
  for (int i = 0; i < count; i++) {
    int offset = 4 + (i * 2);
    battery.cellMv[i] = ((uint16_t)battery.packetBuf[offset] << 8) | battery.packetBuf[offset + 1];
  }

  battery.hasCellData = true;
  battery.lastCellDataMs = millis();
  battery.gotPacket04 = true;
  return true;
}

static void notifyCallback(BLERemoteCharacteristic* characteristic, uint8_t* pData, size_t length, bool isNotify) {
  (void)isNotify;

  int index = findBatteryByRx(characteristic);
  if (index < 0) return;

  BatteryState& battery = batteries[index];
  if (battery.packetError) return;

  if (battery.packetLen == 0) {
    if (length == 0 || pData[0] != 0xDD) return;
    battery.packetError = (pData[2] != 0x00);
    battery.expectedLen = pData[3] + 7;
  }

  if (battery.packetLen + (int)length > MAX_PACKET_LEN) {
    battery.packetError = true;
    return;
  }

  memcpy(battery.packetBuf + battery.packetLen, pData, length);
  battery.packetLen += (int)length;

  if (battery.packetError || battery.expectedLen <= 0 || battery.packetLen != battery.expectedLen) {
    return;
  }

  if (!checksumValid(battery.packetBuf, battery.packetLen)) {
    battery.packetError = true;
    return;
  }

  uint8_t packetType = battery.packetBuf[1];
  if (packetType == 0x03) {
    if (!parsePacket03(battery)) battery.packetError = true;
    return;
  }

  if (packetType == 0x04) {
    if (!parsePacket04(battery)) battery.packetError = true;
    return;
  }

  battery.packetError = true;
}

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) override {
    int index = findBatteryByClient(client);
    if (index < 0) return;

    batteries[index].connected = true;
    batteries[index].connectedAtMs = millis();
    batteries[index].nextReconnectMs = 0;
    Serial.printf("[%s] connected\n", batteries[index].name);
  }

  void onDisconnect(BLEClient* client) override {
    int index = findBatteryByClient(client);
    if (index < 0) return;

    BatteryState& battery = batteries[index];
    battery.connected = false;
    battery.service = nullptr;
    battery.rx = nullptr;
    battery.tx = nullptr;
    battery.lastDisconnectMs = millis();
    battery.nextReconnectMs = battery.lastDisconnectMs + RECONNECT_INTERVAL_MS;
    battery.disconnectCount++;

    Serial.printf("[%s] disconnected (drops=%lu)\n", battery.name, battery.disconnectCount);
  }
};

class DiscoveryCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveServiceUUID() || !advertisedDevice.isAdvertisingService(serviceUUID)) return;

    String seenMac = advertisedDevice.getAddress().toString().c_str();
    seenMac.toLowerCase();

    for (int i = 0; i < BATTERY_COUNT; i++) {
      if (!batteries[i].enabled) continue;

      String targetMac = batteries[i].mac;
      targetMac.toLowerCase();
      if (seenMac != targetMac) continue;

      BLEAdvertisedDevice* discoveredDevice = new BLEAdvertisedDevice(advertisedDevice);
      if (discoveredDevice == nullptr) return;

      batteries[i].seen = true;
      if (batteries[i].advertisedDevice != nullptr) delete batteries[i].advertisedDevice;
      batteries[i].advertisedDevice = discoveredDevice;

      Serial.printf("[%s] discovered at %s\n", batteries[i].name, batteries[i].mac);

      if (seenBatteryCount() >= enabledBatteryCount()) {
        BLEDevice::getScan()->stop();
      }
      return;
    }
  }
};

static ClientCallbacks clientCallbacks;
static DiscoveryCallbacks discoveryCallbacks;

static void cleanupBatteryClient(BatteryState& battery) {
  if (battery.client != nullptr) {
    if (battery.client->isConnected()) {
      battery.client->disconnect();
      delay(DISCONNECT_CLEANUP_DELAY_MS);
    }
    delete battery.client;
    battery.client = nullptr;
  }

  battery.connected = false;
  battery.service = nullptr;
  battery.rx = nullptr;
  battery.tx = nullptr;
  battery.hasCellData = false;
  battery.cellCount = 0;
  resetPacketAssembly(battery);
}

static bool scanForAllEnabledBatteries(unsigned long timeoutMs) {
  for (int i = 0; i < BATTERY_COUNT; i++) {
    batteries[i].seen = false;
    if (batteries[i].advertisedDevice != nullptr) {
      delete batteries[i].advertisedDevice;
      batteries[i].advertisedDevice = nullptr;
    }
  }

  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
  pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

  unsigned long deadlineMs = millis() + timeoutMs;
  while (!hasDeadlinePassed(deadlineMs) && seenBatteryCount() < enabledBatteryCount()) {
    int32_t remainingMs = (int32_t)(deadlineMs - millis());
    if (remainingMs < 0) remainingMs = 0;
    unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS) ? (unsigned long)remainingMs : SCAN_SLICE_MS;
    if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

    pBLEScan->start(scanDurationSeconds(sliceMs), false);
    pBLEScan->clearResults();
  }

  return seenBatteryCount() == enabledBatteryCount();
}

static bool scanForBattery(BatteryState& battery, unsigned long timeoutMs) {
  battery.seen = false;
  if (battery.advertisedDevice != nullptr) {
    delete battery.advertisedDevice;
    battery.advertisedDevice = nullptr;
  }

  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
  pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

  unsigned long deadlineMs = millis() + timeoutMs;
  while (!hasDeadlinePassed(deadlineMs) && !battery.seen) {
    int32_t remainingMs = (int32_t)(deadlineMs - millis());
    if (remainingMs < 0) remainingMs = 0;
    unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS) ? (unsigned long)remainingMs : SCAN_SLICE_MS;
    if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

    pBLEScan->start(scanDurationSeconds(sliceMs), false);
    pBLEScan->clearResults();
  }

  return battery.seen;
}

static bool connectBattery(BatteryState& battery) {
  if (!battery.enabled) return false;
  if (isBatteryConnected(battery)) return true;
  if (battery.advertisedDevice == nullptr) return false;

  cleanupBatteryClient(battery);

  battery.client = BLEDevice::createClient();
  if (battery.client == nullptr) {
    Serial.printf("[%s] failed to create BLE client\n", battery.name);
    return false;
  }

  battery.client->setClientCallbacks(&clientCallbacks);

  delay(CONNECT_DELAY_MS);
  if (!battery.client->connect(battery.advertisedDevice)) {
    Serial.printf("[%s] connect() failed\n", battery.name);
    cleanupBatteryClient(battery);
    return false;
  }

  battery.service = battery.client->getService(serviceUUID);
  if (battery.service == nullptr) {
    Serial.printf("[%s] FF00 service not found\n", battery.name);
    cleanupBatteryClient(battery);
    return false;
  }

  battery.rx = battery.service->getCharacteristic(charUUID_rx);
  if (battery.rx == nullptr || !battery.rx->canNotify()) {
    Serial.printf("[%s] FF01 notify characteristic not found\n", battery.name);
    cleanupBatteryClient(battery);
    return false;
  }
  battery.rx->registerForNotify(notifyCallback);

  battery.tx = battery.service->getCharacteristic(charUUID_tx);
  if (battery.tx == nullptr || (!battery.tx->canWrite() && !battery.tx->canWriteNoResponse())) {
    Serial.printf("[%s] FF02 write characteristic not found\n", battery.name);
    cleanupBatteryClient(battery);
    return false;
  }

  delay(REQUEST_DELAY_MS);
  battery.connected = battery.client->isConnected();
  battery.connectedAtMs = millis();

  Serial.printf("[%s] ready: connected + notifications registered\n", battery.name);
  return battery.connected;
}

static bool requestBatterySnapshot(BatteryState& battery) {
  static uint8_t cmd3[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

  if (!isBatteryConnected(battery)) return false;

  resetPacketAssembly(battery);
  battery.lastRequestMs = millis();
  battery.tx->writeValue(cmd3, sizeof(cmd3), false);

  unsigned long deadlineMs = millis() + RESPONSE_TIMEOUT_MS;
  while (!hasDeadlinePassed(deadlineMs)) {
    if (!isBatteryConnected(battery)) break;
    if (battery.gotPacket03) return true;
    delay(RESPONSE_POLL_MS);
  }

  return false;
}

static bool requestCellVoltages(BatteryState& battery) {
  static uint8_t cmd4[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

  if (!isBatteryConnected(battery)) return false;

  resetPacketAssembly(battery);
  battery.lastRequestMs = millis();
  battery.tx->writeValue(cmd4, sizeof(cmd4), false);

  unsigned long deadlineMs = millis() + RESPONSE_TIMEOUT_MS;
  while (!hasDeadlinePassed(deadlineMs)) {
    if (!isBatteryConnected(battery)) break;
    if (battery.gotPacket04) return true;
    delay(RESPONSE_POLL_MS);
  }

  return false;
}

static bool reconnectBattery(BatteryState& battery) {
  if (isBatteryConnected(battery)) return true;

  Serial.printf("[%s] reconnect attempt\n", battery.name);

  if (battery.advertisedDevice == nullptr) {
    if (!scanForBattery(battery, RECONNECT_SCAN_TIMEOUT_MS)) {
      Serial.printf("[%s] not seen during reconnect scan\n", battery.name);
      battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
      return false;
    }
  }

  if (connectBattery(battery)) return true;

  if (!scanForBattery(battery, RECONNECT_SCAN_TIMEOUT_MS)) {
    Serial.printf("[%s] rediscovery failed\n", battery.name);
    battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
    return false;
  }

  bool ok = connectBattery(battery);
  if (!ok) battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
  return ok;
}

static String boolJson(bool v) {
  return v ? F("true") : F("false");
}

static float computePowerW(const BatteryState& battery) {
  return battery.voltage * battery.current;
}

static uint16_t minCellMv(const BatteryState& battery) {
  if (!battery.hasCellData || battery.cellCount == 0) return 0;
  uint16_t v = battery.cellMv[0];
  for (uint8_t i = 1; i < battery.cellCount; i++) {
    if (battery.cellMv[i] < v) v = battery.cellMv[i];
  }
  return v;
}

static uint16_t maxCellMv(const BatteryState& battery) {
  if (!battery.hasCellData || battery.cellCount == 0) return 0;
  uint16_t v = battery.cellMv[0];
  for (uint8_t i = 1; i < battery.cellCount; i++) {
    if (battery.cellMv[i] > v) v = battery.cellMv[i];
  }
  return v;
}

static String buildJsonResponse() {
  unsigned long now = millis();
  int connectedCount = 0;

  bool aggValid = false;
  float aggVoltage = 0.0f;
  float aggCurrent = 0.0f;
  float aggPower = 0.0f;
  float aggTemp = 0.0f;
  int aggTempCount = 0;

  for (int i = 0; i < BATTERY_COUNT; i++) {
    BatteryState& b = batteries[i];
    if (!b.enabled) continue;
    if (isBatteryConnected(b)) connectedCount++;
    if (!b.hasData) continue;

    aggValid = true;
    aggVoltage += b.voltage;
    aggCurrent += b.current;
    aggPower += computePowerW(b);
    if (b.hasTemperature) {
      aggTemp += b.temperature;
      aggTempCount++;
    }
  }

  float chargePower = aggPower > 0.0f ? aggPower : 0.0f;
  float dischargePower = aggPower < 0.0f ? -aggPower : 0.0f;

  String json;
  json.reserve(2200);
  json += F("{");
  json += F("\"device\":\"AC_BMS_MONITOR\",");
  json += F("\"uptime_ms\":");
  json += String(now);
  json += F(",\"connected_count\":");
  json += String(connectedCount);
  json += F(",\"battery_count\":");
  json += String(BATTERY_COUNT);
  json += F(",\"aggregate\":{");
  json += F("\"valid\":");
  json += boolJson(aggValid);
  json += F(",\"voltage_v\":");
  json += String(aggVoltage, 2);
  json += F(",\"current_a\":");
  json += String(aggCurrent, 2);
  json += F(",\"power_w\":");
  json += String(aggPower, 1);
  json += F(",\"charge_power_w\":");
  json += String(chargePower, 1);
  json += F(",\"discharge_power_w\":");
  json += String(dischargePower, 1);
  json += F(",\"temperature_c\":");
  json += String(aggTempCount > 0 ? (aggTemp / aggTempCount) : 0.0f, 1);
  json += F("},\"batteries\":[");

  for (int i = 0; i < BATTERY_COUNT; i++) {
    BatteryState& b = batteries[i];
    uint16_t minMv = minCellMv(b);
    uint16_t maxMv = maxCellMv(b);
    uint16_t deltaMv = (maxMv >= minMv) ? (maxMv - minMv) : 0;
    unsigned long lastGoodMsAgo = b.lastGoodDataMs == 0 ? 0 : (now - b.lastGoodDataMs);

    if (i > 0) json += ',';
    json += F("{");
    json += F("\"name\":\"");
    json += b.name;
    json += F("\",\"mac\":\"");
    json += b.mac;
    json += F("\",\"connected\":");
    json += boolJson(isBatteryConnected(b));
    json += F(",\"has_data\":");
    json += boolJson(b.hasData);
    json += F(",\"voltage_v\":");
    json += String(b.voltage, 2);
    json += F(",\"current_a\":");
    json += String(b.current, 2);
    json += F(",\"power_w\":");
    json += String(computePowerW(b), 1);
    json += F(",\"soc\":");
    json += String(b.soc);
    json += F(",\"temperature_c\":");
    json += String(b.hasTemperature ? b.temperature : 0.0f, 1);
    json += F(",\"charge_mos\":");
    json += boolJson(b.chargeMos);
    json += F(",\"discharge_mos\":");
    json += boolJson(b.dischargeMos);
    json += F(",\"cell_count\":");
    json += String(b.cellCount);
    json += F(",\"min_cell_mv\":");
    json += String(minMv);
    json += F(",\"max_cell_mv\":");
    json += String(maxMv);
    json += F(",\"cell_delta_mv\":");
    json += String(deltaMv);
    json += F(",\"ok_reads\":");
    json += String(b.okReads);
    json += F(",\"failed_reads\":");
    json += String(b.failedReads);
    json += F(",\"disconnect_count\":");
    json += String(b.disconnectCount);
    json += F(",\"last_good_ms_ago\":");
    json += String(lastGoodMsAgo);
    json += F("}");
  }

  json += F("]}");
  return json;
}

static void printSummary() {
  int connectedCount = 0;
  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (isBatteryConnected(batteries[i])) connectedCount++;
  }

  Serial.printf("\n=== AC_BMS_MONITOR summary @ %lus ===\n", millis() / 1000UL);
  for (int i = 0; i < BATTERY_COUNT; i++) {
    BatteryState& b = batteries[i];
    Serial.printf("[%s] connected=%s seen=%s ok=%lu fail=%lu drops=%lu",
                  b.name,
                  isBatteryConnected(b) ? "yes" : "no",
                  b.seen ? "yes" : "no",
                  b.okReads,
                  b.failedReads,
                  b.disconnectCount);
    if (b.hasData) {
      Serial.printf("  %.2fV %.2fA %.1fW SOC=%u%%", b.voltage, b.current, computePowerW(b), b.soc);
    }
    Serial.println();
  }
  Serial.printf("Connected %d/%d enabled batteries\n", connectedCount, enabledBatteryCount());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("AC_BMS_MONITOR startup");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting WiFi SSID=%s\n", WIFI_SSID);
  unsigned long wifiDeadline = millis() + 30000;
  while (WiFi.status() != WL_CONNECTED && !hasDeadlinePassed(wifiDeadline)) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi not connected yet; HTTP server still started.");
  }

  server.on("/", []() {
    String body = F("AC_BMS_MONITOR OK\nTry /api/bms\n");
    body += F("Connected WiFi: ");
    body += (WiFi.status() == WL_CONNECTED) ? F("yes") : F("no");
    body += '\n';
    body += F("IP: ");
    body += WiFi.localIP().toString();
    body += '\n';
    server.send(200, "text/plain", body);
  });

  server.on("/status", []() {
    server.send(200, "text/plain", "AC_BMS_MONITOR running. Use /api/bms");
  });

  server.on("/api/bms", []() {
    server.send(200, "application/json", buildJsonResponse());
  });

  server.begin();

  BLEDevice::init("AC_BMS_MONITOR");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(&discoveryCallbacks);

  Serial.printf("Configured entries: %d\n", BATTERY_COUNT);
  for (int i = 0; i < BATTERY_COUNT; i++) {
    Serial.printf("  [%d] %s  %s  enabled=%s\n",
                  i,
                  batteries[i].name,
                  batteries[i].mac,
                  batteries[i].enabled ? "true" : "false");
  }

  bool discoverySuccessful = scanForAllEnabledBatteries(STARTUP_SCAN_TIMEOUT_MS);
  Serial.printf("Startup discovery: seen=%d/%d\n", seenBatteryCount(), enabledBatteryCount());
  if (!discoverySuccessful) {
    Serial.println("Some enabled batteries were not found during startup scan.");
  }

  for (int i = 0; i < BATTERY_COUNT; i++) {
    if (!batteries[i].enabled) continue;

    if (!batteries[i].seen) {
      Serial.printf("[%s] startup connect skipped (not discovered)\n", batteries[i].name);
      batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
      continue;
    }

    bool ok = connectBattery(batteries[i]);
    Serial.printf("[%s] startup connect %s\n", batteries[i].name, ok ? "OK" : "FAIL");
    if (!ok) batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
  }

  printSummary();
}

void loop() {
  loopCount++;

  for (int i = 0; i < BATTERY_COUNT; i++) {
    BatteryState& battery = batteries[i];
    if (!battery.enabled) continue;

    if (isBatteryConnected(battery)) {
      bool ok03 = requestBatterySnapshot(battery);
      if (ok03) {
        battery.okReads++;
      } else {
        battery.failedReads++;
      }

      if ((loopCount % CELL_PACKET_EVERY_N_CYCLES) == 0 && isBatteryConnected(battery)) {
        bool ok04 = requestCellVoltages(battery);
        if (!ok04) battery.failedReads++;
      }
    } else if (battery.nextReconnectMs != 0 && hasDeadlinePassed(battery.nextReconnectMs)) {
      reconnectBattery(battery);
    }

    delay(BETWEEN_BATTERIES_MS);
  }

  if ((loopCount % 10) == 0) {
    printSummary();
  }

  server.handleClient();
  delay(BETWEEN_CYCLES_MS);
}
