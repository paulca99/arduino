/************************************************************
   SOLIS S5‑EH1P‑3.6K‑L POLLER + WEB UI + JSON API
   ------------------------------------------------
   - Core 1: FreeRTOS task polls Solis via RS485 (UART2)
   - Core 0: WiFi + AsyncWebServer on 80 (HTML UI)
             AsyncWebServer on 8080 (/api/solis JSON)
   - Shared SolisData protected by a mutex

   DEBUG NOTES:
   - Validates Modbus reply: slave id, function, byte count, CRC
   - Hex-dumps first bytes of frames periodically
   - Logs JSON response periodically + heap

   IMPORTANT (shared bus):
   - You're tapped onto an existing RS485 bus (Eastron meter <-> Solis).
     There will be other traffic.

   SNIFFER MODE:
   - Set SNIFFER_MODE=true to disable transmitting and just passively sniff.
   - It will print observed Modbus request/response frames (id, func, addr, count)
     and keep a small "seen slave IDs" bitset.
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

/********************  MODE  ********************/
// Passive sniffer mode: do NOT transmit. Just observe existing bus traffic.
static const bool SNIFFER_MODE = true;

/********************  BUS / MODBUS CONFIG  ********************/
// Only used when SNIFFER_MODE=false.
static const uint8_t SOLIS_SLAVE_ID = 1;

/********************  DEBUG CONFIG  ********************/
static const bool  DEBUG_MODBUS = true;
static const bool  DEBUG_JSON   = true;
static const uint32_t DEBUG_FRAME_DUMP_MS = 2000;
static const uint32_t DEBUG_JSON_DUMP_MS  = 2000;
static const uint32_t DEBUG_STATS_MS      = 5000;

/********************  MODBUS TIMING  ********************/
static const uint32_t MODBUS_REPLY_TIMEOUT_MS = 350; // shared bus + device response latency
static const uint32_t MODBUS_SNIFF_TIMEOUT_MS = 200; // window for assembling a frame while sniffing

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
static void dumpBytes(const uint8_t* b, int len, int maxBytes = 16) {
  int n = len < maxBytes ? len : maxBytes;
  for (int i = 0; i < n; i++) {
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(' ');
  }
}

static void dumpFrameHead(const char* tag, const uint8_t* b, int len) {
  Serial.print("[");
  Serial.print(tag);
  Serial.print("] len=");
  Serial.print(len);
  Serial.print(" head=");
  dumpBytes(b, len, 16);
  Serial.println();
}

static bool validateModbusCrc(const uint8_t* b, int len) {
  if (len < 4) return false;
  uint16_t rxCrc = (uint16_t)b[len - 2] | ((uint16_t)b[len - 1] << 8);
  uint16_t calc  = modbusCRC((uint8_t*)b, len - 2);
  return rxCrc == calc;
}

static bool validateModbusReply(const uint8_t* b, int len, uint8_t expectedId, uint8_t expectedFunc, uint8_t expectedByteCount) {
  if (len < 5) return false;

  if (b[0] != expectedId) return false;
  if (b[1] != expectedFunc) return false;
  if (b[2] != expectedByteCount) return false;

  return validateModbusCrc(b, len);
}

/********************  MODBUS TX  ********************/
void sendRequest(uint8_t slaveId, uint16_t startReg, uint16_t count) {
  uint8_t frame[8];
  frame[0] = slaveId;
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

/********************  MODBUS RX (SYNC STATE MACHINE)  ********************/
int readReplySynced(uint8_t* buf, int bufSize, uint8_t slaveId, uint8_t func, uint8_t expectedByteCount) {
  enum { S_SYNC_ID, S_SYNC_FUNC, S_READ_BYTECOUNT, S_READ_REST } state = S_SYNC_ID;
  int len = 0;
  int targetLen = 0;
  uint32_t start = millis();

  while (millis() - start < MODBUS_REPLY_TIMEOUT_MS) {
    while (RS485.available()) {
      uint8_t c = (uint8_t)RS485.read();

      switch (state) {
        case S_SYNC_ID:
          if (c == slaveId) {
            buf[0] = c;
            len = 1;
            state = S_SYNC_FUNC;
          }
          break;

        case S_SYNC_FUNC:
          if (c == func || c == (uint8_t)(func | 0x80)) {
            buf[1] = c;
            len = 2;
            state = S_READ_BYTECOUNT;
          } else {
            state = S_SYNC_ID;
          }
          break;

        case S_READ_BYTECOUNT:
          buf[2] = c;
          len = 3;

          // Exception reply is always 5 bytes total.
          if (buf[1] == (uint8_t)(func | 0x80)) {
            targetLen = 5;
            state = S_READ_REST;
            break;
          }

          // Enforce expected byteCount for THIS request; otherwise treat as other traffic/desync.
          if (c != expectedByteCount) {
            state = S_SYNC_ID;
            break;
          }

          targetLen = 3 + c + 2;
          if (targetLen > bufSize) {
            state = S_SYNC_ID;
            break;
          }
          state = S_READ_REST;
          break;

        case S_READ_REST:
          if (len < bufSize) {
            buf[len++] = c;
          }
          if (targetLen > 0 && len >= targetLen) {
            return len;
          }
          break;
      }
    }
    vTaskDelay(1);
  }

  return 0;
}

/********************  MODBUS RX (SNIFFER)  ********************/
// Attempts to assemble a Modbus RTU frame by scanning for plausible headers.
// Supports:
//  - Requests: 8 bytes: [id][func][addrHi][addrLo][cntHi][cntLo][crcLo][crcHi]
//  - Responses: [id][func][byteCount][...data...][crcLo][crcHi]
//  - Exceptions: 5 bytes: [id][func|0x80][excCode][crcLo][crcHi]
//
// Returns bytes in buf (0 if no complete frame in timeout window).
int sniffFrame(uint8_t* buf, int bufSize) {
  enum { S_SYNC_ID, S_SYNC_FUNC, S_GUESS_LEN, S_READ_REST } state = S_SYNC_ID;
  int len = 0;
  int targetLen = 0;
  uint32_t start = millis();

  while (millis() - start < MODBUS_SNIFF_TIMEOUT_MS) {
    while (RS485.available()) {
      uint8_t c = (uint8_t)RS485.read();

      switch (state) {
        case S_SYNC_ID:
          // Accept any non-zero slave id
          if (c != 0) {
            buf[0] = c;
            len = 1;
            state = S_SYNC_FUNC;
          }
          break;

        case S_SYNC_FUNC:
          // Accept common Modbus funcs 1..6, 15, 16 and exception variants.
          // We keep it permissive to learn the bus.
          buf[1] = c;
          len = 2;
          state = S_GUESS_LEN;
          break;

        case S_GUESS_LEN:
          buf[2] = c;
          len = 3;

          // Exception frame is always 5 bytes total
          if ((buf[1] & 0x80) != 0) {
            targetLen = 5;
            state = S_READ_REST;
            break;
          }

          // If third byte looks like a byteCount for a response (reasonable 1..252)
          // we assume it's a response.
          if (buf[2] >= 1 && buf[2] <= 252) {
            // Could still be a request (addrHi) coincidentally in range.
            // Heuristic: if we can parse a valid 8-byte request later, great.
            targetLen = 3 + buf[2] + 2;
            if (targetLen > bufSize) {
              // Too big -> probably not a response bytecount; restart
              state = S_SYNC_ID;
              break;
            }
            state = S_READ_REST;
            break;
          }

          // Otherwise assume it's a request and read fixed 8 bytes.
          targetLen = 8;
          if (targetLen > bufSize) {
            state = S_SYNC_ID;
            break;
          }
          state = S_READ_REST;
          break;

        case S_READ_REST:
          if (len < bufSize) buf[len++] = c;
          if (targetLen > 0 && len >= targetLen) {
            // Validate CRC; if it fails, restart sync and keep scanning
            if (validateModbusCrc(buf, len)) {
              return len;
            }
            state = S_SYNC_ID;
            len = 0;
            targetLen = 0;
          }
          break;
      }
    }
    vTaskDelay(1);
  }
  return 0;
}

static void printSniffedFrame(const uint8_t* b, int len) {
  uint8_t id = b[0];
  uint8_t func = b[1];

  if ((func & 0x80) && len == 5) {
    Serial.print("[SNIFF] EXC id="); Serial.print(id);
    Serial.print(" func=0x"); Serial.print((uint8_t)(func & 0x7F), HEX);
    Serial.print(" code="); Serial.println(b[2]);
    return;
  }

  if (len == 8) {
    uint16_t addr = ((uint16_t)b[2] << 8) | b[3];
    uint16_t cnt  = ((uint16_t)b[4] << 8) | b[5];
    Serial.print("[SNIFF] REQ id="); Serial.print(id);
    Serial.print(" func=0x"); Serial.print(func, HEX);
    Serial.print(" addr=0x"); Serial.print(addr, HEX);
    Serial.print(" count="); Serial.println(cnt);
    return;
  }

  if (len >= 5) {
    uint8_t byteCount = b[2];
    Serial.print("[SNIFF] RESP id="); Serial.print(id);
    Serial.print(" func=0x"); Serial.print(func, HEX);
    Serial.print(" bytes="); Serial.print(byteCount);
    Serial.print(" head=");
    dumpBytes(b, len, 12);
    Serial.println();
    return;
  }
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
  uint8_t buf[256];
  uint32_t lastDump = 0;
  uint32_t lastStats = 0;

  // Stats
  uint32_t okFrames = 0;
  uint32_t crcFails = 0;
  uint32_t timeouts = 0;
  uint32_t exceptions = 0;

  // Sniffer: bitset of seen slave IDs
  static uint8_t seenIds[32]; // 256 bits
  auto markSeen = [&](uint8_t id) {
    seenIds[id >> 3] |= (1u << (id & 7));
  };

  for (;;) {
    if (SNIFFER_MODE) {
      int n = sniffFrame(buf, sizeof(buf));
      if (n > 0) {
        markSeen(buf[0]);
        if (DEBUG_MODBUS) {
          printSniffedFrame(buf, n);
        }
      }

      uint32_t now = millis();
      if (DEBUG_MODBUS && (now - lastStats) > DEBUG_STATS_MS) {
        lastStats = now;
        Serial.print("[SNIFF-STATS] seenIds=");
        for (int id = 1; id <= 247; id++) {
          if (seenIds[id >> 3] & (1u << (id & 7))) {
            Serial.print(id);
            Serial.print(' ');
          }
        }
        Serial.println();
      }

      // Yield a bit
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // Active poller mode (not used in sniff mode)
    vTaskDelay(pdMS_TO_TICKS(20));

    // Block 1: 0x3100 (40 registers => expect byteCount=80 => 85-byte frame)
    sendRequest(SOLIS_SLAVE_ID, 0x3100, 40);
    int n1 = readReplySynced(buf, sizeof(buf), SOLIS_SLAVE_ID, 3, 80);
    if (n1 > 0) {
      uint32_t now = millis();
      if (DEBUG_MODBUS && (now - lastDump) > DEBUG_FRAME_DUMP_MS) {
        lastDump = now;
        dumpFrameHead("3100", buf, n1);
      }

      if (n1 == 5 && buf[1] == (uint8_t)(3 | 0x80)) {
        exceptions++;
        if (DEBUG_MODBUS) {
          Serial.print("[3100] Modbus exception code: ");
          Serial.println(buf[2]);
        }
      } else if (validateModbusReply(buf, n1, SOLIS_SLAVE_ID, 3, 80)) {
        okFrames++;
        xSemaphoreTake(solisMutex, portMAX_DELAY);
        decodeBlock1(buf);
        xSemaphoreGive(solisMutex);
      } else {
        crcFails++;
        if (DEBUG_MODBUS) {
          uint16_t rxCrc = (uint16_t)buf[n1 - 2] | ((uint16_t)buf[n1 - 1] << 8);
          uint16_t calc  = modbusCRC(buf, n1 - 2);
          Serial.print("[3100] CRC/header fail n="); Serial.print(n1);
          Serial.print(" crcRx=0x"); Serial.print(rxCrc, HEX);
          Serial.print(" crcCalc=0x"); Serial.println(calc, HEX);
        }
      }
    } else {
      timeouts++;
      if (DEBUG_MODBUS) Serial.println("[3100] timeout/no frame for this slave");
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    // Block 2: 0x3200
    sendRequest(SOLIS_SLAVE_ID, 0x3200, 40);
    int n2 = readReplySynced(buf, sizeof(buf), SOLIS_SLAVE_ID, 3, 80);
    if (n2 > 0) {
      uint32_t now = millis();
      if (DEBUG_MODBUS && (now - lastDump) > DEBUG_FRAME_DUMP_MS) {
        lastDump = now;
        dumpFrameHead("3200", buf, n2);
      }

      if (n2 == 5 && buf[1] == (uint8_t)(3 | 0x80)) {
        exceptions++;
        if (DEBUG_MODBUS) {
          Serial.print("[3200] Modbus exception code: ");
          Serial.println(buf[2]);
        }
      } else if (validateModbusReply(buf, n2, SOLIS_SLAVE_ID, 3, 80)) {
        okFrames++;
        xSemaphoreTake(solisMutex, portMAX_DELAY);
        decodeBlock2(buf);
        xSemaphoreGive(solisMutex);
      } else {
        crcFails++;
        if (DEBUG_MODBUS) {
          uint16_t rxCrc = (uint16_t)buf[n2 - 2] | ((uint16_t)buf[n2 - 1] << 8);
          uint16_t calc  = modbusCRC(buf, n2 - 2);
          Serial.print("[3200] CRC/header fail n="); Serial.print(n2);
          Serial.print(" crcRx=0x"); Serial.print(rxCrc, HEX);
          Serial.print(" crcCalc=0x"); Serial.println(calc, HEX);
        }
      }
    } else {
      timeouts++;
      if (DEBUG_MODBUS) Serial.println("[3200] timeout/no frame for this slave");
    }

    uint32_t now = millis();
    if (DEBUG_MODBUS && (now - lastStats) > DEBUG_STATS_MS) {
      lastStats = now;
      Serial.print("[STATS] ok="); Serial.print(okFrames);
      Serial.print(" crcFail="); Serial.print(crcFails);
      Serial.print(" exc="); Serial.print(exceptions);
      Serial.print(" timeout="); Serial.println(timeouts);
    }

    vTaskDelay(pdMS_TO_TICKS(400));
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
    if (!isfinite(local.batteryCurrent) || !isfinite(local.gridVoltage) || !isfinite(local.frequency)) {
      Serial.println("[API] Non-finite value detected in SolisData snapshot");
    }
  }

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
  serverHTTP.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });
  serverHTTP.begin();

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

  xTaskCreatePinnedToCore(
    solisTask,
    "SolisPoller",
    6144,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Solis poller + web servers started");
  if (SNIFFER_MODE) {
    Serial.println("*** SNIFFER_MODE enabled: transmit disabled, passive listen only ***");
  }
}

/********************  LOOP (CORE 0)  ********************/
void loop() {
  delay(500);
}
