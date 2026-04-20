#define DEBUG
#include "BleSerialClient.h"
using namespace std;

ByteRingBuffer<RX_BUFFER_SIZE> BleSerialClient::receiveBuffer;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

    log_i("Notify callback for characteristic ");
    log_i("%s",pBLERemoteCharacteristic->getUUID().toString().c_str());
    log_i(" of data length %d", length);

    for (int i = 0; i < length; i++){
       log_i("adding data to buffer %#04x", pData[i]);
       BleSerialClient::receiveBuffer.add(pData[i]); 
    }  
#ifdef DEBUG
    Serial.println("Reading:");
    char strBuf[50];
    for (int i = 0; i < length; i++){ 
        sprintf(strBuf, " - 0x%02x",pData[i]);
        Serial.print(strBuf);
    }
    Serial.println();
#endif

}  // notify callback

static void scanCompleteCB(BLEScanResults scanResults) {
#ifdef DEBUG
    printf("Scan complete!\n");
    printf("We found %d devices\n", scanResults.getCount());
    scanResults.dump();
    Serial.println("Begining Scan Again.");
#endif
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->clearResults();
    pBLEScan->start(5, scanCompleteCB, false);
} // scanCompleteCB

void BleSerialClient::onConnect(BLEClient *pClient)
{
  log_i("In onConnect()");
  bleConnected = true;
  if (enableLed)
    digitalWrite(ledPin, HIGH);
}  // onConnect

void BleSerialClient::onDisconnect(BLEClient *pClient)
{
  log_i("In onDisconnect()");
  bleConnected = false;
  if (enableLed)
    digitalWrite(ledPin, LOW);
  delay(5000);
  this->receiveBuffer.clear();
} // onDisconnect

bool BleSerialClient::connectToServer() {
    Serial.print("connectToServer() called: ");
    Serial.println(myDevice->getAddress().toString().c_str());

    pClient->setMTU(517);

    Serial.println("Calling pClient->connect()...");
    pClient->connect(myDevice);
    Serial.println("connect() call done");

    // ✅ Wait up to 5 seconds for connection to actually be ready
    int timeout = 50;
    while (!pClient->isConnected() && timeout > 0) {
        delay(100);
        timeout--;
        Serial.print(".");
    }
    Serial.println();

    if (!pClient->isConnected()) {
        Serial.println("❌ Timed out waiting for connection");
        pClient->disconnect();
        return false;
    }
    Serial.println("✅ isConnected() true!");

    Serial.println("Calling getService()...");
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    Serial.printf("getService returned: %s\n", pRemoteService ? "found" : "null");

    if (pRemoteService == nullptr) {
      Serial.println("❌ Service not found");
      pClient->disconnect();
      return false;
    }

    RxCharacteristic = pRemoteService->getCharacteristic(charRxUUID);
    Serial.printf("RX char: %s\n", RxCharacteristic ? "found" : "null");
    if (RxCharacteristic == nullptr) {
      pClient->disconnect();
      return false;
    }
    bleConnected = true;

    if(RxCharacteristic->canNotify())
      RxCharacteristic->registerForNotify(notifyCallback);

    TxCharacteristic = pRemoteService->getCharacteristic(charTxUUID);
    Serial.printf("TX char: %s\n", TxCharacteristic ? "found" : "null");
    if (TxCharacteristic == nullptr) {
      pClient->disconnect();
      return false;
    }

    if(TxCharacteristic->canWrite())
      Serial.println("✅ TX can write — fully connected!");

    return true;
}
void BleSerialClient::onResult(BLEAdvertisedDevice advertisedDevice) {
#ifdef DEBUG
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
#endif
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      pBLEScan->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
} // onResult

void BleSerialClient::begin(const char *name, bool enable_led, int led_pin)
{
  log_i("In begin()");
  enableLed = enable_led;
  ledPin = led_pin;
  serviceUUID = BLEUUID().fromString(BLE_SERIAL_SERVICE_UUID);
  charRxUUID  = BLEUUID().fromString(BLE_RX_UUID);
  charTxUUID  = BLEUUID().fromString(BLE_TX_UUID);
  if (enableLed) {
    pinMode(ledPin, OUTPUT);
  }
#ifdef DEBUG
  Serial.println("Starting BLE Client application...");
#endif
  pBLEDevice = new BLEDevice();
  pBLEDevice->init("");
  pClient = pBLEDevice->createClient();
  pClient->setClientCallbacks(this);

  pBLEScan = pBLEDevice->getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(this);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, scanCompleteCB, false);
  log_i("begin exit");
}

void BleSerialClient::bleLoop() {
  if (millis() - flush_100ms >= flush_time) flush();

  if (doConnect == true) {
    if (connectToServer()) {
#ifdef DEBUG
      Serial.println("We are now connected to the BLE Server.");
#endif
      this->receiveBuffer.clear();
    }
#ifdef DEBUG
    else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
#endif
    doConnect = false;
  }

  if (bleConnected == true) {
    log_i("we are connected");
  } else if (doScan) {
    log_i("ble !connected starting scanner");
    BLEDevice::getScan()->start(5, false);
  }
}

bool BleSerialClient::connected()
{
  return bleConnected;
}

int BleSerialClient::read()
{
  log_i("In read()");
  uint8_t result = this->receiveBuffer.pop();
  if (result == (uint8_t)'\n') {
    this->numAvailableLines--;
  }
  return result;
}

size_t BleSerialClient::readBytes(uint8_t *buffer, size_t bufferSize)
{
  log_i("readBytes(uint8_t *buffer, size_t bufferSize)");
  int i = 0;
  while (i < bufferSize && available()) {
    buffer[i] = this->receiveBuffer.pop();
    i++;
  }
  return i;
}

int BleSerialClient::peek()
{
  log_i("In peek()");
  if (this->receiveBuffer.getLength() == 0)
    return -1;
  return this->receiveBuffer.get(0);
}

int BleSerialClient::available()
{
  return receiveBuffer.getLength();
}

size_t BleSerialClient::print(const char *str)
{
  if (pClient->isConnected() == 0) {
    return 0;
  }
  size_t written = 0;
  for (size_t i = 0; str[i] != '\0'; i++) {
    written += this->write(str[i]);
  }
  if ((this->transmitBufferLength == maxTransferSize) || ((millis() - flush_100ms >= flush_time))) {
    flush();
  }
  return written;
}

size_t BleSerialClient::write(const uint8_t *buffer, size_t bufferSize)
{
  log_i("In write(const uint8_t *buffer, size_t bufferSize)");
  if (pClient->isConnected() == 0) {
    log_i("in write pClient is disconnected");
    return 0;
  }
  if (maxTransferSize < MIN_MTU) {
    int oldTransferSize = maxTransferSize;
    MTU = pClient->getMTU() - 5;
    maxTransferSize = MTU > BLE_BUFFER_SIZE ? BLE_BUFFER_SIZE : MTU;
    if (maxTransferSize != oldTransferSize) {
      log_i("Max BLE transfer size set to %u", maxTransferSize);
    }
  }
  if (maxTransferSize < MIN_MTU) {
    return 0;
  }
  size_t written = 0;
  for (int i = 0; i < bufferSize; i++) {
    log_i("about to write");
    written += this->write(buffer[i]);
  }
  if ((this->transmitBufferLength == maxTransferSize) || ((millis() - flush_100ms >= flush_time))) {
    flush();
  }
  return written;
}

size_t BleSerialClient::write(uint8_t byte)
{
  log_i("In write(uint8_t byte)");
  if (pClient->isConnected() == 0) {
    log_e("Not Connected");
    return 0;
  }
  log_i("%#04x", byte);
  this->transmitBuffer[this->transmitBufferLength] = byte;
  this->transmitBufferLength++;
  if ((this->transmitBufferLength == maxTransferSize) || ((millis() - flush_100ms >= flush_time))) {
    // flush();
  }
  return 1;
}

void BleSerialClient::flush()
{
  if (true) {
    if (this->transmitBufferLength > 0) {
      log_i("flushing %d bytes", this->transmitBufferLength);
      TxCharacteristic->writeValue(this->transmitBuffer, this->transmitBufferLength);
      this->transmitBufferLength = 0;
    }
    flush_100ms = millis();
  }
}

void BleSerialClient::end()
{
  pBLEDevice->deinit();
}

BleSerialClient::BleSerialClient()
{
}