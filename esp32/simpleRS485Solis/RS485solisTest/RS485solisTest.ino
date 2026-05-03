/************************************************************
   SOLIS S5‑EH1P‑3.6K‑L POLLER + WEB UI + JSON API
   ------------------------------------------------
   - Core 1: FreeRTOS task polls Solis via RS485 (UART2)
   - Core 0: WiFi + AsyncWebServer on 80 (HTML UI)
             AsyncWebServer on 8080 (/api/solis JSON)
   - Shared SolisData protected by a mutex

   DEBUG NOTES (added):
   - Validates Modbus reply: slave id, function, byte count, CRC
   - Hex-dumps first bytes of frames periodically
   - Flushes UART RX before each request (to prevent misaligned frames)
   - Logs JSON response periodically + heap

   IMPORTANT:
   - At 9600 baud, an 85-byte Modbus response takes ~90ms on the wire.
     The original 40ms read window will frequently timeout.
************************************************************/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <math.h>

HardwareSerial RS485(2);   // UART2

/********************  WIFI CONFIG  ********************/
const char* WIFI_SSID     = "TP-LINK_73F3";
const char* WIFI_PASSWORD = "DEADBEEF";

/********************  WEB SERVERS  ********************/
AsyncWebServer serverHTTP(80);     // Pretty UI
AsyncWebServer serverAPI(8080);    // JSON API

/********************  SHARED DATA  ********************/
struct SolisData {
  float pv1Voltage, pv1Current;
  float pv2Voltage, pv2Current;
  float batteryVoltage, batteryCurrent, batteryPower;
  float gridVoltage, gridCurrent, gridPower;
  float frequency, powerFactor;
  float inverterTemp;
  float totalPower;
  float dailyEnergy, totalEnergy;
};

SolisData solis;                 // Shared instance
SemaphoreHandle_t solisMutex;    // Protects 'solis'

/********************  DEBUG CONFIG  ********************/
static const bool  DEBUG_MODBUS = true;
static const bool  DEBUG_JSON   = true;
static const uint32_t DEBUG_FRAME_DUMP_MS = 2000;
static const uint32_t DEBUG_JSON_DUMP_MS  = 2000;

/********************  MODBUS TIMING  ********************/
// 85 bytes at 9600 baud is ~90ms on the wire, plus device processing time.
static const uint32_t MODBUS_REPLY_TIMEOUT_MS = 250;

/********************  CRC  ********************/
uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

/********************  DEBUG HELPERS  ********************/
static void dumpFrameHead(const char* tag, const uint8_t* b, int len) {
  Serial.print("[");
  Serial.print(tag);
  Serial.print("] len=");
  Serial.print(len);
  Serial.print(" head=");
  int n = len < 16 ? len : 16;
  for (int i = 0; i < n; i++) {
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static bool validateModbusReply(const uint8_t* b, int len, uint8_t expectedId, uint8_t expectedFunc, uint8_t expectedByteCount) {
  if (len < 5) return false;

  if (b[0] != expectedId) return false;
  if (b[1] != expectedFunc) return false;
  if (b[2] != expectedByteCount) return false;

  // CRC is last 2 bytes, little-endian
  uint16_t rxCrc = (uint16_t)b[len - 2] | ((uint16_t)b[len - 1] << 8);
  uint16_t calc  = modbusCRC((uint8_t*)b, len - 2);
  return rxCrc == calc;
}

static void flushRS485Rx() {
  int flushed = 0;
  while (RS485.available()) {
    RS485.read();
    flushed++;
  }
  if (DEBUG_MODBUS && flushed) {
    Serial.print("[RS485] flushed bytes: ");
    Serial.println(flushed);
  }
}

/********************  MODBUS TX  ********************/
void sendRequest(uint16_t startReg, uint16_t count) {
  uint8_t frame[8];
  frame[0] = 1; // Solis inverter ID
  frame[1] = 3; // Read Holding Registers
  frame[2] = startReg >> 8;
  frame[3] = startReg & 0xFF;
  frame[4] = count >> 8;
  frame[5] = count & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  RS485.write(frame, 8);
}

/********************  MODBUS RX (NON-BLOCKING)  ********************/
// Reads a Modbus RTU reply.
// - If expectedLen > 0, reads exactly that many bytes.
// - If expectedLen == 0, dynamically determines length after reading first 3 bytes:
//     total = 3 + byteCount + 2
// Returns number of bytes read (0 on timeout).
int readReply(uint8_t *buf, int bufSize, int expectedLen) {
  int len = 0;
  int targetLen = expectedLen;
  unsigned long start = millis();

  while (millis() - start < MODBUS_REPLY_TIMEOUT_MS) {
    while (RS485.available()) {
      int c = RS485.read();
      if (len < bufSize) {
        buf[len++] = (uint8_t)c;
      }

      if (expectedLen == 0 && len >= 3) {
        uint8_t byteCount = buf[2];
        targetLen = 3 + byteCount + 2;
        if (targetLen > bufSize) {
          if (DEBUG_MODBUS) {
            Serial.print("[RS485] reply too big for buffer, byteCount=");
            Serial.println(byteCount);
          }
          return 0;
        }
      }

      if (targetLen > 0 && len >= targetLen) {
        return len;
      }
    }
    vTaskDelay(1);  // yield to other tasks
  }

  return 0;
}

/********************  DECODE BLOCKS  ********************/
void decodeBlock1(uint8_t *b) {
  solis.pv1Voltage = (b[3] << 8 | b[4]) / 10.0;
  solis.pv1Current = (b[5] << 8 | b[6]) / 10.0;

  solis.pv2Voltage = (b[7] << 8 | b[8]) / 10.0;
  solis.pv2Current = (b[9] << 8 | b[10]) / 10.0;

  solis.batteryVoltage = (b[15] << 8 | b[16]) / 10.0;
  solis.batteryCurrent = (int16_t)(b[17] << 8 | b[18]) / 10.0;
  solis.batteryPower   = (int16_t)(b[19] << 8 | b[20]);

  solis.inverterTemp = (b[35] << 8 | b[36]) / 10.0;
}

void decodeBlock2(uint8_t *b) {
  solis.gridVoltage = (b[3] << 8 | b[4]) / 10.0;
  solis.gridCurrent = (b[5] << 8 | b[6]) / 10.0;
  solis.gridPower   = (int16_t)(b[7] << 8 | b[8]);

  solis.frequency   = (b[9] << 8 | b[10]) / 100.0;
  solis.powerFactor = (int16_t)(b[11] << 8 | b[12]) / 1000.0;

  solis.totalPower  = (int16_t)(b[13] << 8 | b[14]);

  solis.dailyEnergy = (b[23] << 8 | b[24]) / 100.0;
  solis.totalEnergy = (b[25] << 8 | b[26]) / 10.0;
}

/********************  SOLIS POLLING TASK (CORE 1)  ********************/
void solisTask(void *pv) {
  uint8_t buf[128];
  uint32_t lastDump = 0;

  for (;;) {
    // Block 1: 0x3100 (40 registers => expect byteCount=80 => 85-byte frame)
    flushRS485Rx();
    sendRequest(0x3100, 40);
    int n1 = readReply(buf, sizeof(buf), 0);
    if (n1 > 0) {
      bool ok = (n1 == 85) && validateModbusReply(buf, n1, 1, 3, 80);

      uint32_t now = millis();
      if (DEBUG_MODBUS && (now - lastDump) > DEBUG_FRAME_DUMP_MS) {
        lastDump = now;
        dumpFrameHead("3100", buf, n1);
        if (!ok) {
          uint16_t rxCrc = (n1 >= 2) ? ((uint16_t)buf[n1 - 2] | ((uint16_t)buf[n1 - 1] << 8)) : 0;
          uint16_t calc  = (n1 >= 2) ? modbusCRC(buf, n1 - 2) : 0;
          Serial.print("[3100] INVALID frame n="); Serial.print(n1);
          Serial.print(" id="); Serial.print(buf[0]);
          Serial.print(" func="); Serial.print(buf[1]);
          Serial.print(" bytes="); Serial.print(buf[2]);
          Serial.print(" crcRx=0x"); Serial.print(rxCrc, HEX);
          Serial.print(" crcCalc=0x"); Serial.println(calc, HEX);
        }
      }

      if (ok) {
        xSemaphoreTake(solisMutex, portMAX_DELAY);
        decodeBlock1(buf);
        xSemaphoreGive(solisMutex);
      }
    } else if (DEBUG_MODBUS) {
      Serial.println("[3100] timeout/no full frame");
    }

    // Block 2: 0x3200
    flushRS485Rx();
    sendRequest(0x3200, 40);
    int n2 = readReply(buf, sizeof(buf), 0);
    if (n2 > 0) {
      bool ok = (n2 == 85) && validateModbusReply(buf, n2, 1, 3, 80);

      uint32_t now = millis();
      if (DEBUG_MODBUS && (now - lastDump) > DEBUG_FRAME_DUMP_MS) {
        lastDump = now;
        dumpFrameHead("3200", buf, n2);
        if (!ok) {
          uint16_t rxCrc = (n2 >= 2) ? ((uint16_t)buf[n2 - 2] | ((uint16_t)buf[n2 - 1] << 8)) : 0;
          uint16_t calc  = (n2 >= 2) ? modbusCRC(buf, n2 - 2) : 0;
          Serial.print("[3200] INVALID frame n="); Serial.print(n2);
          Serial.print(" id="); Serial.print(buf[0]);
          Serial.print(" func="); Serial.print(buf[1]);
          Serial.print(" bytes="); Serial.print(buf[2]);
          Serial.print(" crcRx=0x"); Serial.print(rxCrc, HEX);
          Serial.print(" crcCalc=0x"); Serial.println(calc, HEX);
        }
      }

      if (ok) {
        xSemaphoreTake(solisMutex, portMAX_DELAY);
        decodeBlock2(buf);
        xSemaphoreGive(solisMutex);
      }
    } else if (DEBUG_MODBUS) {
      Serial.println("[3200] timeout/no full frame");
    }

    vTaskDelay(pdMS_TO_TICKS(300)); // 300ms between polls
  }
}

/********************  HTML PAGE (PORT 80)  ********************/
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Solis Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:0; padding:20px; }
    h1 { text-align:center; }
    .grid { display:grid; grid-template-columns: repeat(auto-fit,minmax(220px,1fr)); gap:15px; margin-top:20px; }
    .card { background:#1e1e1e; padding:15px; border-radius:8px; box-shadow:0 0 10px #000; }
    .label { font-size:0.9em; color:#aaa; }
    .value { font-size:1.4em; margin-top:5px; }
  </style>
</head>
<body>
  <h1>Solis S5‑EH1P‑3.6K‑L</h1>
  <div class="grid">
    <div class="card">
      <div class="label">Grid Power</div>
      <div class="value" id="gridPower">-- W</div>
    </div>
    <div class="card">
      <div class="label">PV1</div>
      <div class="value" id="pv1">-- V / -- A</div>
    </div>
    <div class="card">
      <div class="label">PV2</div>
      <div class="value" id="pv2">-- V / -- A</div>
    </div>
    <div class="card">
      <div class="label">Battery</div>
      <div class="value" id="battery">-- V / -- A / -- W</div>
    </div>
    <div class="card">
      <div class="label">Grid</div>
      <div class="value" id="grid">-- V / -- A</div>
    </div>
    <div class="card">
      <div class="label">Freq / PF</div>
      <div class="value" id="freqpf">-- Hz / --</div>
    </div>
    <div class="card">
      <div class="label">Temp</div>
      <div class="value" id="temp">-- °C</div>
    </div>
    <div class="card">
      <div class="label">Energy</div>
      <div class="value" id="energy">Day: -- kWh / Total: -- kWh</div>
    </div>
  </div>

  <script>
    async function fetchData() {
      try {
        const url = "http://" + location.hostname + ":8080/api/solis";
        const res = await fetch(url);
        const d = await res.json();

        document.getElementById('gridPower').textContent =
          d.gridPower.toFixed(0) + " W";

        document.getElementById('pv1').textContent =
          d.pv1Voltage.toFixed(1) + " V / " + d.pv1Current.toFixed(1) + " A";

        document.getElementById('pv2').textContent =
          d.pv2Voltage.toFixed(1) + " V / " + d.pv2Current.toFixed(1) + " A";

        document.getElementById('battery').textContent =
          d.batteryVoltage.toFixed(1) + " V / " +
          d.batteryCurrent.toFixed(1) + " A / " +
          d.batteryPower.toFixed(0) + " W";

        document.getElementById('grid').textContent =
          d.gridVoltage.toFixed(1) + " V / " +
          d.gridCurrent.toFixed(1) + " A";

        document.getElementById('freqpf').textContent =
          d.frequency.toFixed(2) + " Hz / " +
          d.powerFactor.toFixed(3);

        document.getElementById('temp').textContent =
          d.inverterTemp.toFixed(1) + " °C";

        document.getElementById('energy').textContent =
          "Day: " + d.dailyEnergy.toFixed(2) + " kWh / Total: " +
          d.totalEnergy.toFixed(1) + " kWh";

      } catch (e) {
        console.log("Fetch error", e);
      }
    }

    setInterval(fetchData, 1000);
    fetchData();
  </script>
</body>
</html>
)HTML";

/********************  JSON API HANDLER (PORT 8080)  ********************/
String buildJson() {
  // Take a snapshot of the shared struct
  SolisData local;
  xSemaphoreTake(solisMutex, portMAX_DELAY);
  local = solis;
  xSemaphoreGive(solisMutex);

  if (DEBUG_JSON) {
    // Non-finite values will mess up formatting / indicate decode issues
    if (!isfinite(local.batteryCurrent) || !isfinite(local.gridVoltage) || !isfinite(local.frequency)) {
      Serial.println("[API] Non-finite value detected in SolisData snapshot");
    }
  }

  // Build JSON manually to avoid extra libs
  String json = "{";
  json += "\"pv1Voltage\":"    + String(local.pv1Voltage, 1) + ",";
  json += "\"pv1Current\":"    + String(local.pv1Current, 1) + ",";
  json += "\"pv2Voltage\":"    + String(local.pv2Voltage, 1) + ",";
  json += "\"pv2Current\":"    + String(local.pv2Current, 1) + ",";
  json += "\"batteryVoltage\":"+ String(local.batteryVoltage, 1) + ",";
  json += "\"batteryCurrent\":"+ String(local.batteryCurrent, 1) + ",";
  json += "\"batteryPower\":"  + String(local.batteryPower, 0) + ",";
  json += "\"gridVoltage\":"   + String(local.gridVoltage, 1) + ",";
  json += "\"gridCurrent\":"   + String(local.gridCurrent, 1) + ",";
  json += "\"gridPower\":"     + String(local.gridPower, 0) + ",";
  json += "\"frequency\":"     + String(local.frequency, 2) + ",";
  json += "\"powerFactor\":"   + String(local.powerFactor, 3) + ",";
  json += "\"inverterTemp\":"  + String(local.inverterTemp, 1) + ",";
  json += "\"totalPower\":"    + String(local.totalPower, 0) + ",";
  json += "\"dailyEnergy\":"   + String(local.dailyEnergy, 2) + ",";
  json += "\"totalEnergy\":"   + String(local.totalEnergy, 1);
  json += "}";
  return json;
}

/********************  WIFI + WEB SETUP  ********************/
void setupWeb() {
  // HTTP UI on port 80
  serverHTTP.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });
  serverHTTP.begin();

  // JSON API on port 8080
  serverAPI.on("/api/solis", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = buildJson();

    if (DEBUG_JSON) {
      static uint32_t last = 0;
      uint32_t now = millis();
      if (now - last > DEBUG_JSON_DUMP_MS) {
        last = now;
        Serial.print("[API] freeHeap=");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" jsonLen=");
        Serial.println(json.length());
        Serial.println(json);
      }
    }

    request->send(200, "application/json", json);
  });
  serverAPI.begin();
}

/********************  SETUP  ********************/
void setup() {
  Serial.begin(115200);
  RS485.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  solisMutex = xSemaphoreCreateMutex();

  // WiFi connect
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  setupWeb();

  // Start Solis polling task on core 1
  xTaskCreatePinnedToCore(
    solisTask,
    "SolisPoller",
    4096,
    NULL,
    1,
    NULL,
    1   // core 1
  );

  Serial.println("Solis poller + web servers started");
}

/********************  LOOP (CORE 0)  ********************/
void loop() {
  // You can add charger logic here if you like.
  // Web servers and WiFi run in background on core 0.
  delay(500);
}
