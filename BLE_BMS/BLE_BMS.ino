#include <NimBLEDevice.h>

// Target: Solax (Strongest Signal)
const std::string target_mac = "A4:C1:37:20:4E:3B";

static NimBLEUUID serviceUUID("FF00");
static NimBLEUUID charUUID_RX("FF01"); 
static NimBLEUUID charUUID_TX("FF02"); 

NimBLEClient* pClient = nullptr;
bool doConnect = false;
bool connected = false;
uint8_t readInfoCmd[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

// --- Callback: This is where your Volts/Amps appear ---
void notifyCallback(NimBLERemoteCharacteristic* pRemoteChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length > 20) {
        // JBD Protocol Parsing
        uint16_t v_raw = (uint16_t)pData[4] << 8 | pData[5];
        int16_t i_raw = (int16_t)pData[6] << 8 | pData[7];
        int soc = pData[23];

        Serial.println("\n***************************");
        Serial.printf("🏠 SOLAX STATUS REPORT\n");
        Serial.printf("⚡ Voltage: %.2f V\n", v_raw / 100.0);
        Serial.printf("🔌 Current: %.2f A\n", i_raw / 100.0);
        Serial.printf("🔋 SoC:     %d %%\n", soc);
        Serial.println("***************************");
    }
}

class MyCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
        std::string addr = advertisedDevice->getAddress().toString();
        for (auto & c : addr) c = toupper(c); 

        if (addr == target_mac) {
            Serial.println("\n🎯 Solax spotted! Attempting long-range link...");
            NimBLEDevice::getScan()->stop(); 
            doConnect = true;
        }
    }
};

void connectToBMS() {
    Serial.println("🔗 Linking to Solax...");
    if (!pClient) pClient = NimBLEDevice::createClient();

    // Give it 10 seconds to handshake because of the distance
    pClient->setConnectTimeout(10); 

    if (pClient->connect(NimBLEAddress(target_mac, 0), false)) { 
        Serial.println("✅ SUCCESS: Solax Link Established!");
        NimBLERemoteService* pSvc = pClient->getService(serviceUUID);
        if (pSvc) {
            NimBLERemoteCharacteristic* pChrRX = pSvc->getCharacteristic(charUUID_RX);
            if (pChrRX) {
                pChrRX->subscribe(true, notifyCallback);
                connected = true;
                return;
            }
        }
    }
    
    Serial.println("❌ Link failed. Signal might be too weak upstairs. Rescanning...");
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    connected = false;
    NimBLEDevice::getScan()->start(0, false);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== SOLAX MONITORING MODE ===");
    
    NimBLEDevice::init("");
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new MyCallbacks(), false);
    pScan->setActiveScan(true); 
    pScan->setInterval(250); // Slightly slower for better reliability
    pScan->setWindow(200);
    pScan->start(0, false);
}

void loop() {
    if (doConnect) {
        doConnect = false;
        connectToBMS();
    }

    if (connected && pClient && pClient->isConnected()) {
        NimBLERemoteService* pSvc = pClient->getService(serviceUUID);
        if (pSvc) {
            NimBLERemoteCharacteristic* pChrTX = pSvc->getCharacteristic(charUUID_TX);
            if (pChrTX) {
                pChrTX->writeValue((const uint8_t*)readInfoCmd, sizeof(readInfoCmd), false);
            }
        }
        delay(5000); // Update every 5 seconds
    } else if (connected) {
        connected = false;
        Serial.println("⚠️ Lost Solax signal. Searching...");
        NimBLEDevice::getScan()->start(0, false);
    }
    
    Serial.print("."); 
    delay(1000);
}
