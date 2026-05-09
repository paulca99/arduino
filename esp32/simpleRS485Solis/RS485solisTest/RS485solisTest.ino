#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>

#if __has_include("secrets.h")
#include "secrets.h"
#elif __has_include("secrets_template.h")
#include "secrets_template.h"
#else
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";
const uint8_t SOLIS_SLAVE_ID = 1;
#endif

HardwareSerial RS485(2);
AsyncWebServer server(80);

static const uint32_t MODBUS_TIMEOUT_MS = 180;
static const uint32_t POLL_INTERVAL_MS = 1000;
static const uint32_t INTER_REGISTER_DELAY_MS = 15;
static const uint32_t SERIAL_SETTLE_DELAY_MS = 1200;
static const uint32_t MONITOR_MUTEX_TIMEOUT_MS = 100;

struct RegisterSpec {
  uint16_t reg;
};

static const RegisterSpec REGISTER_SPECS[] = {
  {33050},
  {33051},
  {33052},
  {33053},
  {33059},
  {33072},
  {33074},
  {33080},
  {33081},
  {33085},
  {33095},
  {33129},
  {33130},
  {33131},
  {33132},
  {33134},
  {33135},
  {33136},
  {33137},
  {33140},
  {33142},
};

static const size_t REGISTER_COUNT = sizeof(REGISTER_SPECS) / sizeof(REGISTER_SPECS[0]);
static const size_t JSON_BASE_OVERHEAD_BYTES = 160;
static const size_t JSON_BYTES_PER_REGISTER = 48;
static const size_t JSON_RESERVE_BYTES =
  JSON_BASE_OVERHEAD_BYTES + (REGISTER_COUNT * JSON_BYTES_PER_REGISTER);

struct RegisterValue {
  uint16_t raw;
  bool valid;
};

struct MonitorState {
  RegisterValue values[REGISTER_COUNT];
  uint32_t lastPollMs;
  uint32_t lastSuccessMs;
  uint32_t pollCount;
  uint32_t readErrors;
};

MonitorState monitorState = {};
SemaphoreHandle_t monitorMutex = nullptr;

static uint16_t modbusCRC(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

static void flush485() {
  while (RS485.available()) {
    RS485.read();
  }
}

static int readReply(uint8_t* buf, int maxLen, uint32_t timeoutMs) {
  int len = 0;
  uint32_t last = millis();
  while (millis() - last < timeoutMs) {
    while (RS485.available()) {
      uint8_t b = RS485.read();
      if (len < maxLen) {
        buf[len++] = b;
      }
      last = millis();
    }
    delay(1);
  }
  return len;
}

static bool validCRC(const uint8_t* buf, int len) {
  if (len < 4) {
    return false;
  }
  uint16_t rxCRC = buf[len - 2] | (uint16_t(buf[len - 1]) << 8);
  return rxCRC == modbusCRC(buf, len - 2);
}

static void sendReadInput(uint8_t slave, uint16_t rawAddr, uint16_t count) {
  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x04;
  frame[2] = rawAddr >> 8;
  frame[3] = rawAddr & 0xFF;
  frame[4] = count >> 8;
  frame[5] = count & 0xFF;
  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
  RS485.write(frame, 8);
  RS485.flush();
}

static bool readDocRegU16(uint8_t slave, uint16_t docReg, uint16_t& value) {
  uint8_t buf[32];
  const uint16_t rawAddr = docReg - 1;

  flush485();
  sendReadInput(slave, rawAddr, 1);

  const int len = readReply(buf, sizeof(buf), MODBUS_TIMEOUT_MS);
  if (len < 7) {
    return false;
  }
  if (!validCRC(buf, len)) {
    return false;
  }
  if (buf[0] != slave || buf[1] != 0x04 || buf[2] != 2) {
    return false;
  }

  value = (uint16_t(buf[3]) << 8) | buf[4];
  return true;
}

static String buildJson() {
  MonitorState snapshot = {};
  if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    snapshot = monitorState;
    xSemaphoreGive(monitorMutex);
  }

  String json;
  json.reserve(JSON_RESERVE_BYTES);
  json += "{";
  json += "\"uptimeMs\":";
  json += String(millis());
  json += ",\"lastPollMs\":";
  json += String(snapshot.lastPollMs);
  json += ",\"lastSuccessMs\":";
  json += String(snapshot.lastSuccessMs);
  json += ",\"pollCount\":";
  json += String(snapshot.pollCount);
  json += ",\"readErrors\":";
  json += String(snapshot.readErrors);

  for (size_t i = 0; i < REGISTER_COUNT; i++) {
    json += ",\"";
    json += String(REGISTER_SPECS[i].reg);
    json += "\":{";
    json += "\"valid\":";
    json += snapshot.values[i].valid ? "true" : "false";
    json += ",\"raw\":";
    json += String(snapshot.values[i].raw);
    json += ",\"signed\":";
    json += String((int16_t)snapshot.values[i].raw);
    json += "}";
  }

  json += "}";
  return json;
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Solis RS485 Monitor</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; font-family: Arial, sans-serif; background: #0f141a; color: #edf2f7; }
    main { max-width: 1100px; margin: 0 auto; padding: 16px; }
    h1 { margin: 0 0 6px; font-size: 1.7rem; }
    h2 { margin: 24px 0 10px; font-size: 1.1rem; color: #9fb3c8; }
    p { margin: 0 0 12px; color: #b8c4d0; }
    .status { display: flex; flex-wrap: wrap; gap: 12px; margin-bottom: 16px; color: #9fb3c8; font-size: 0.95rem; }
    .cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }
    .card, .panel { background: #18212b; border: 1px solid #22303d; border-radius: 10px; box-shadow: 0 4px 16px rgba(0, 0, 0, 0.25); }
    .card { padding: 14px; }
    .card .label { color: #8ea3b8; font-size: 0.85rem; margin-bottom: 6px; }
    .card .value { font-size: 1.45rem; font-weight: 700; }
    .subvalue { margin-top: 6px; font-size: 0.9rem; color: #aab8c5; }
    .stale { color: #ffb86b; }
    table { width: 100%; border-collapse: collapse; }
    th, td { text-align: left; padding: 10px 12px; border-bottom: 1px solid #22303d; vertical-align: top; }
    th { color: #8ea3b8; font-size: 0.82rem; text-transform: uppercase; letter-spacing: 0.04em; }
    td.note { color: #9fb3c8; font-size: 0.9rem; }
    .panel { overflow: hidden; }
    .panel-header { padding: 14px 14px 0; }
    .muted { color: #90a4b8; }
    code { color: #9ad1ff; }
  </style>
</head>
<body>
<main>
  <h1>Solis RS485 live monitor</h1>
  <p>ESP32-hosted page for watching the current Solis register guesses while you confirm the map.</p>

  <div class="status">
    <div id="age">Waiting for first poll…</div>
    <div id="polls">Polls: --</div>
    <div id="errors">Read errors: --</div>
  </div>

  <div class="cards">
    <div class="card">
      <div class="label">Grid power</div>
      <div class="value" id="card33132">--</div>
      <div class="subvalue">Reg 33132, signed 16-bit watts</div>
    </div>
    <div class="card">
      <div class="label">Grid voltage</div>
      <div class="value" id="card33074">--</div>
      <div class="subvalue">Reg 33074</div>
    </div>
    <div class="card">
      <div class="label">Grid frequency</div>
      <div class="value" id="card33095">--</div>
      <div class="subvalue">Reg 33095</div>
    </div>
    <div class="card">
      <div class="label">PV string 1 voltage</div>
      <div class="value" id="card33050">--</div>
      <div class="subvalue">Reg 33050</div>
    </div>
    <div class="card">
      <div class="label">PV string 2 voltage</div>
      <div class="value" id="card33052">--</div>
      <div class="subvalue">Reg 33052</div>
    </div>
    <div class="card">
      <div class="label">Battery SOC</div>
      <div class="value" id="card33140">--</div>
      <div class="subvalue">Reg 33140</div>
    </div>
    <div class="card">
      <div class="label">Battery voltage</div>
      <div class="value" id="card33142">--</div>
      <div class="subvalue">Reg 33142</div>
    </div>
  </div>

  <section class="panel">
    <div class="panel-header">
      <h2>Main watch list</h2>
      <p class="muted">Better-understood values first, with the still-uncertain current / direction candidates left visible for tonight's confirmation.</p>
    </div>
    <table>
      <thead>
        <tr>
          <th>Label</th>
          <th>Register</th>
          <th>Display</th>
          <th>Raw</th>
          <th>Signed</th>
          <th>Notes</th>
        </tr>
      </thead>
      <tbody id="mainRows"></tbody>
    </table>
  </section>

  <section class="panel" style="margin-top:16px">
    <div class="panel-header">
      <h2>Tentative / unknown registers</h2>
      <p class="muted">These are shown lower down on purpose so you can watch the raw values and keep refining the map.</p>
    </div>
    <table>
      <thead>
        <tr>
          <th>Label</th>
          <th>Register</th>
          <th>Display</th>
          <th>Raw</th>
          <th>Signed</th>
          <th>Notes</th>
        </tr>
      </thead>
      <tbody id="tentativeRows"></tbody>
    </table>
  </section>

  <p style="margin-top:16px" class="muted">JSON endpoint: <code>/api/solis</code> (same host, same port).</p>
</main>

<script>
  const mainRegs = [
    { reg: '33050', label: 'PV string 1 voltage', divisor: 10, decimals: 1, unit: 'V', note: 'Strong candidate from testing.' },
    { reg: '33051', label: 'PV string 1 current / power-related', divisor: 10, decimals: 1, unit: 'TBD', note: 'Exact meaning still unknown.' },
    { reg: '33052', label: 'PV string 2 voltage', divisor: 10, decimals: 1, unit: 'V', note: 'Strong candidate from testing.' },
    { reg: '33053', label: 'PV string 2 current / power-related', divisor: 10, decimals: 1, unit: 'TBD', note: 'Exact meaning still unknown.' },
    { reg: '33074', label: 'AC / grid voltage', divisor: 10, decimals: 1, unit: 'V', note: 'Known 0.1 V scaling.' },
    { reg: '33095', label: 'Grid frequency', divisor: 100, decimals: 2, unit: 'Hz', note: 'Known 0.01 Hz scaling.' },
    { reg: '33129', label: 'AC voltage 2', divisor: 10, decimals: 1, unit: 'V', note: 'Second AC-voltage-looking register.' },
    { reg: '33130', label: 'Grid / load current candidate', divisor: 10, decimals: 1, unit: 'A', note: 'Likely current, scaling still tentative.' },
    { reg: '33131', label: 'Import / export direction flag candidate', rawOnly: true, note: 'Watch for mode flips during load changes.' },
    { reg: '33132', label: 'Grid power', divisor: 1, decimals: 0, unit: 'W', signed: true, note: 'Strong candidate; signed 16-bit watts.' },
    { reg: '33140', label: 'Battery SOC', divisor: 1, decimals: 0, unit: '%', note: 'SOC percentage.' },
    { reg: '33142', label: 'Battery voltage', divisor: 100, decimals: 2, unit: 'V', note: 'Likely 0.01 V scaling.' }
  ];

  const tentativeRegs = [
    { reg: '33059', label: 'Register 33059', rawOnly: true, note: 'Unknown; show raw for comparison.' },
    { reg: '33072', label: 'Register 33072', rawOnly: true, note: 'Unknown; may be AC-side related.' },
    { reg: '33080', label: 'Register 33080', rawOnly: true, note: 'Unknown candidate from recent change logging.' },
    { reg: '33081', label: 'Register 33081', rawOnly: true, note: 'Moved strongly during earlier tests.' },
    { reg: '33085', label: 'Register 33085', rawOnly: true, note: 'Moved strongly during earlier tests.' },
    { reg: '33134', label: 'Register 33134', rawOnly: true, note: 'Tentative battery / power-related candidate.' },
    { reg: '33135', label: 'Register 33135', rawOnly: true, note: 'Tentative grid / net-power-related candidate.' },
    { reg: '33136', label: 'Register 33136', rawOnly: true, note: 'Unknown; grouped with 33134/33135.' },
    { reg: '33137', label: 'Register 33137', rawOnly: true, note: 'Tentative PV-related candidate.' }
  ];
  const regMeta = Object.fromEntries([...mainRegs, ...tentativeRegs].map(meta => [meta.reg, meta]));

  function rowHtml(meta) {
    return `<tr>
      <td>${meta.label}</td>
      <td>${meta.reg}</td>
      <td id="display-${meta.reg}">--</td>
      <td id="raw-${meta.reg}">--</td>
      <td id="signed-${meta.reg}">--</td>
      <td class="note">${meta.note}</td>
    </tr>`;
  }

  function installRows() {
    document.getElementById('mainRows').innerHTML = mainRegs.map(rowHtml).join('');
    document.getElementById('tentativeRows').innerHTML = tentativeRegs.map(rowHtml).join('');
  }

  function formatValue(meta, entry) {
    if (!entry || !entry.valid) {
      return '--';
    }
    if (meta.rawOnly) {
      return String(entry.raw);
    }
    const source = meta.signed ? entry.signed : entry.raw;
    const divisor = meta.divisor || 1;
    const decimals = meta.decimals || 0;
    const scaled = source / divisor;
    return scaled.toFixed(decimals) + (meta.unit ? ` ${meta.unit}` : '');
  }

  function updateRow(meta, data) {
    const entry = data[meta.reg];
    document.getElementById(`display-${meta.reg}`).textContent = formatValue(meta, entry);
    document.getElementById(`raw-${meta.reg}`).textContent = entry && entry.valid ? entry.raw : '--';
    document.getElementById(`signed-${meta.reg}`).textContent = entry && entry.valid ? entry.signed : '--';
  }

  function updateCards(data) {
    ['33132', '33074', '33095', '33050', '33052', '33140', '33142'].forEach(reg => {
      document.getElementById(`card${reg}`).textContent = formatValue(regMeta[reg], data[reg]);
    });
  }

  function updateStatus(data) {
    const ageMs = data.lastSuccessMs ? Math.max(0, data.uptimeMs - data.lastSuccessMs) : 0;
    const ageText = data.lastSuccessMs ? `Last good poll ${ageMs} ms ago` : 'Waiting for first successful poll…';
    const ageNode = document.getElementById('age');
    ageNode.textContent = ageText;
    ageNode.className = ageMs > 5000 ? 'stale' : '';
    document.getElementById('polls').textContent = `Polls: ${data.pollCount}`;
    document.getElementById('errors').textContent = `Read errors: ${data.readErrors}`;
  }

  async function fetchData() {
    try {
      const res = await fetch('/api/solis', { cache: 'no-store' });
      const data = await res.json();
      updateStatus(data);
      updateCards(data);
      mainRegs.forEach(meta => updateRow(meta, data));
      tentativeRegs.forEach(meta => updateRow(meta, data));
    } catch (error) {
      console.log('Fetch error', error);
    }
  }

  installRows();
  fetchData();
  setInterval(fetchData, 1000);
</script>
</body>
</html>
)HTML";

static void setupWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/solis", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildJson());
  });

  server.begin();
}

static void pollTask(void* pv) {
  (void)pv;

  for (;;) {
    uint32_t errorsThisPass = 0;
    bool anySuccess = false;

    for (size_t i = 0; i < REGISTER_COUNT; i++) {
      uint16_t raw = 0;
      if (readDocRegU16(SOLIS_SLAVE_ID, REGISTER_SPECS[i].reg, raw)) {
        const uint32_t now = millis();
        if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
          monitorState.values[i].raw = raw;
          monitorState.values[i].valid = true;
          monitorState.lastSuccessMs = now;
          xSemaphoreGive(monitorMutex);
          anySuccess = true;
        } else {
          errorsThisPass++;
        }
      } else {
        errorsThisPass++;
      }
      vTaskDelay(pdMS_TO_TICKS(INTER_REGISTER_DELAY_MS));
    }

    if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
      monitorState.lastPollMs = millis();
      monitorState.pollCount++;
      monitorState.readErrors += errorsThisPass;
      xSemaphoreGive(monitorMutex);
    }

    if (!anySuccess) {
      Serial.println("Poll completed with no successful register reads");
    }

    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(SERIAL_SETTLE_DELAY_MS);  // Let Serial and ESP32 startup messages settle before Wi-Fi output.
  Serial.println();
  Serial.println("Solis RS485 web monitor starting");

  monitorMutex = xSemaphoreCreateMutex();
  RS485.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Web monitor: http://");
  Serial.println(WiFi.localIP());

  setupWeb();

  xTaskCreatePinnedToCore(
    pollTask,
    "SolisPoll",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );
}

void loop() {
  delay(500);
}
