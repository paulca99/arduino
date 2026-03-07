#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <time.h>

// --- CONFIGURATION ---
const char* passwords[] = {"SecretPass123", "Workshop99", "Todmorden789"};
int passCount = 3;
WebServer server(80);

// --- DATA STRUCTURES (Must match Slaves) ---
typedef struct { 
    uint32_t unixTime; 
    int staggerMinutes; 
    float manualAngle; 
    float calibOffset; 
} struct_sync;

typedef struct { 
    float angle; 
    bool moving; 
    uint8_t mac[6]; 
} struct_status;

// --- REGISTRY ---
struct_status registry[6];
int slaveCount = 0;
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    if (len == sizeof(struct_status)) {
        struct_status incoming;
        memcpy(&incoming, data, sizeof(incoming));
        int idx = -1;
        for(int i=0; i<slaveCount; i++) {
            if(memcmp(registry[i].mac, mac, 6) == 0) idx = i;
        }
        if(idx == -1 && slaveCount < 6) {
            idx = slaveCount++;
            memcpy(registry[idx].mac, mac, 6);
            esp_now_peer_info_t peer = {};
            memcpy(peer.peer_addr, mac, 6);
            esp_now_add_peer(&peer);
        }
        if(idx != -1) {
            registry[idx].angle = incoming.angle;
            registry[idx].moving = incoming.moving;
        }
    }
}

void handleRoot() {
    String h = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><style>body{font-family:sans-serif;background:#121212;color:#e0e0e0;padding:20px;} h1{color:#fbc02d;} .card{background:#1e1e1e;padding:15px;margin-bottom:10px;border-left:5px solid #fbc02d;border-radius:8px;} .btn{padding:8px 12px;background:#fbc02d;color:#121212;text-decoration:none;border-radius:4px;font-weight:bold;display:inline-block;margin:5px 2px;} .btn-sec{background:#444;color:white;}</style></head><body>";
    h += "<h1>Solar Fence Dashboard</h1>";
    for(int i=0; i<slaveCount; i++) {
        h += "<div class='card'><b>Panel " + String(i+1) + "</b><br>Angle: " + String(registry[i].angle) + "&deg; (" + String(registry[i].moving ? "MOVING" : "IDLE") + ")<br>";
        h += "<a href='/move?id=" + String(i) + "&ang=10' class='btn'>Set 10&deg;</a>";
        h += "<a href='/move?id=" + String(i) + "&ang=65' class='btn'>Set 65&deg;</a><br>";
        h += "Calibrate: <a href='/cal?id=" + String(i) + "&off=0.5' class='btn btn-sec'>+0.5&deg;</a>";
        h += "<a href='/cal?id=" + String(i) + "&off=-0.5' class='btn btn-sec'>-0.5&deg;</a></div>";
    }
    h += "</body></html>";
    server.send(200, "text/html", h);
}

void handleMove() {
    int id = server.arg("id").toInt();
    float ang = server.arg("ang").toFloat();
    struct_sync cmd = {0, 0, ang, 0.0}; 
    esp_now_send(registry[id].mac, (uint8_t *) &cmd, sizeof(cmd));
    server.sendHeader("Location", "/"); server.send(303);
}

void handleCal() {
    int id = server.arg("id").toInt();
    float off = server.arg("off").toFloat();
    struct_sync cmd = {0, 0, -1.0, off}; 
    esp_now_send(registry[id].mac, (uint8_t *) &cmd, sizeof(cmd));
    server.sendHeader("Location", "/"); server.send(303);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Solar_Fence_Master", "todmorden");
    int n = WiFi.scanNetworks();
    for (int i=0; i<n; i++) {
        for (int j=0; j<passCount; j++) {
            WiFi.begin(WiFi.SSID(i).c_str(), passwords[j]);
            int r=0; while(WiFi.status()!=WL_CONNECTED && r++<15) delay(400);
            if(WiFi.status()==WL_CONNECTED) break;
        }
        if(WiFi.status()==WL_CONNECTED) break;
    }
    if(WiFi.status()==WL_CONNECTED) {
        configTime(0, 0, "uk.pool.ntp.org");
        server.on("/", handleRoot);
        server.on("/move", handleMove);
        server.on("/cal", handleCal);
        server.begin();
    }
    esp_now_init();
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    esp_now_peer_info_t bcast = {}; memcpy(bcast.peer_addr, broadcastAddr, 6); esp_now_add_peer(&bcast);
}

void loop() {
    server.handleClient();
    static unsigned long lastBeacon = 0;
    if (millis() - lastBeacon > 5000) {
        lastBeacon = millis();
        time_t now; time(&now);
        struct_sync sync = {(uint32_t)now, 0, -1.0, 0.0}; 
        for(int i=0; i<slaveCount; i++) {
            sync.staggerMinutes = i + 1;
            esp_now_send(registry[i].mac, (uint8_t *) &sync, sizeof(sync));
        }
        struct_sync ping = {0, 0, -1.0, 0.0}; 
        esp_now_send(broadcastAddr, (uint8_t *) &ping, sizeof(ping));
    }
}
