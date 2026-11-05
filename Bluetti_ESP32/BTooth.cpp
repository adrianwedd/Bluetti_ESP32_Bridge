#include "BluettiConfig.h"
#include "BTooth.h"
#include "utils.h"
#include "PayloadParser.h"
#include "BWifi.h"
#include "display.h"


int pollTick = 0;

struct command_handle {
  uint8_t page;
  uint8_t offset;
  int length;
};

QueueHandle_t commandHandleQueue;
QueueHandle_t sendQueue;

unsigned long lastBTMessage = 0;

// --- BLE link management ---------------------------------------------------
// Upstream scanned once at boot and only re-scanned after a *successful* match,
// so one missed 5 s window meant the bridge never connected again and the
// watchdog deep-slept it in a loop. Scanning is now periodic and the watchdog
// is left as the last-resort path it was meant to be.
static BLEClient*     pClient = nullptr;   // one reused client, not one per attempt
static BLEAddress     bluettiAddress;      // copied at match time: no dangling pointer into scan results
static bool           scanInProgress = false;
static bool           scanDue = true;      // set on disconnect and on failed connect
static unsigned long  lastScanStart = 0;
static unsigned long  lastConnectAttempt = 0;
static bool           everAttemptedConnect = false;
static uint8_t        connectFailures = 0;

/**
 * Space out connection attempts after a failure: 10 s, 20 s, 40 s, 80 s, 160 s,
 * then 300 s. Scanning stays frequent, but a station that is refusing the link
 * must not be hammered every 10 s indefinitely - it is already advertising
 * while ignoring CONNECT_IND, and thousands of retries cannot help it.
 */
static unsigned long connectBackoffMs() {
  uint8_t shift = connectFailures > 5 ? 5 : connectFailures;
  unsigned long s = 10UL << shift;
  if (s > 300UL) { s = 300UL; }
  return s * 1000UL;
}

static void onScanComplete(BLEScanResults results) {
  scanInProgress = false;
  Serial.printf("[BT] scan complete: %d advertiser(s)\n", (int) results.getCount());
}

/**
 * Identity of the power station.
 *
 *  (A) the configured BLE name, exactly as before, for units that advertise one;
 *  (B) the shape this AC200L actually advertises: no name, service 0xff00, a
 *      "BLBLUETTI" manufacturer payload, and its known BLE address.
 *
 * The address is part of (B) deliberately: service UUID and manufacturer prefix
 * alone would attach to any Bluetti in radio range.
 */
static bool isBluettiDevice(BLEAdvertisedDevice* adv, const char* deviceId) {
  if (!adv->haveServiceUUID() || !adv->isAdvertisingService(serviceUUID)) {
    return false;
  }
  if (strlen(deviceId) > 0 && strcmp(adv->getName().c_str(), deviceId) == 0) {
    return true;
  }
#ifdef BLUETTI_BLE_ADDRESS
  if (strcasecmp(adv->getAddress().toString().c_str(), BLUETTI_BLE_ADDRESS) == 0
      && adv->haveManufacturerData()
      && adv->getManufacturerData().find("BLBLUETTI") != std::string::npos) {
    return true;
  }
#endif
  return false;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println(F("BLE - onConnect"));
     #ifdef DISPLAYSSD1306
      disp_setBlueTooth(true);
     #endif
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    // Stale handles must not survive the link: polling an invalidated
    // characteristic is how a dropped link turns into a crash instead of a retry.
    pRemoteWriteCharacteristic  = nullptr;
    pRemoteNotifyCharacteristic = nullptr;
    // Drop queued frames: a notification from the next link must not be parsed
    // against a page/offset left over from the dropped one.
    if (commandHandleQueue != nullptr) { xQueueReset(commandHandleQueue); }
    if (sendQueue != nullptr)          { xQueueReset(sendQueue); }
    scanDue = true;
    Serial.println(F("BLE - onDisconnect"));
    #ifdef DISPLAYSSD1306
      disp_setBlueTooth(false);
    #endif
    #ifdef RELAISMODE
      #ifdef DEBUG
        Serial.println(F("deactivate relais contact"));
      #endif
      digitalWrite(RELAIS_PIN, RELAIS_LOW);
    #endif
  }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class BluettiAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice *advertisedDevice) {
    Serial.print(F("[BLE] Advertised Device found: "));
    Serial.println(advertisedDevice->toString().c_str());

    ESPBluettiSettings settings = get_esp32_bluetti_settings();
    // We have found a device, let us now see if it is our power station.
    if (isBluettiDevice(advertisedDevice, settings.bluetti_device_id)) {
      if (doConnect) {
        return;   // already matched in this scan; do not queue a second attempt
      }
      Serial.printf("[BT] matched power station at %s (advType=%d connectable=%d addrType=%d)\n",
                    advertisedDevice->getAddress().toString().c_str(),
                    (int) advertisedDevice->getAdvType(),
                    advertisedDevice->isConnectable() ? 1 : 0,
                    (int) advertisedDevice->getAddress().getType());
      BLEDevice::getScan()->stop();
      // Copy the address: the advertised-device object belongs to the scan
      // results and is invalidated by the next clearResults().
      bluettiAddress = advertisedDevice->getAddress();
      doConnect = true;
    }
  } 
};

void initBluetooth(){
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new BluettiAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  
  commandHandleQueue = xQueueCreate( 5, sizeof(bt_command_t ) );
  sendQueue = xQueueCreate( 5, sizeof(bt_command_t) );
}


static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

#ifdef DEBUG
    Serial.println("[BT] F01 - Write Response");
    /* pData Debug... */
    for (int i=1; i<=length; i++){
       
      Serial.printf("%02x", pData[i-1]);
      
      if(i % 2 == 0){
        Serial.print(" "); 
      } 

      if(i % 16 == 0){
        Serial.println();  
      }
    }
    Serial.println();
#endif

    bt_command_t command_handle;
    if(xQueueReceive(commandHandleQueue, &command_handle, 500)){
      parse_bluetooth_data(command_handle.page, command_handle.offset, pData, length);
    }
   
}

bool connectToServer() {
    Serial.print(F("[BT] Forming a connection to "));
    Serial.println(bluettiAddress.toString().c_str());

    // Reset any handles left from a previous attempt before touching the link.
    connected = false;
    pRemoteWriteCharacteristic  = nullptr;
    pRemoteNotifyCharacteristic = nullptr;

    BLEDevice::setMTU(517); // set client to request maximum MTU from server (default is 23 otherwise)

    // One client for the life of the process. Creating a client per attempt
    // leaks a peer for every failed reconnect, which is how a flaky link turns
    // into a device that needs a power cycle.
    if (pClient == nullptr) {
      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new MyClientCallback());
      Serial.println(F("[BT] - Created client"));
    } else {
      pClient->disconnect();   // drop any stale session from the previous attempt
    }

    // Connect to the remote BLE Server.
    if (!pClient->connect(bluettiAddress)) {
      int rc = pClient->getLastError();
      // rc=574 (0x23E) is HCI 0x3E BLE_ERR_CONN_ESTABLISHMENT: our CONNECT_IND
      // went unanswered, i.e. the power station is not accepting the link.
      // Logged with the advertised address type so the two causes (wrong
      // address type vs. the unit refusing) can be told apart afterwards.
      Serial.printf("[BT] - connect() failed: rc=%d (%s) addrType=%d\n",
                    rc, NimBLEUtils::returnCodeToString(rc),
                    (int) bluettiAddress.getType());
      return false;
    }
    Serial.println(F("[BT] - Connected to server"));
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print(F("[BT] Failed to find our service UUID: "));
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(F("[BT] - Found our service"));


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteWriteCharacteristic = pRemoteService->getCharacteristic(WRITE_UUID);
    if (pRemoteWriteCharacteristic == nullptr) {
      Serial.print(F("[BT] Failed to find our characteristic UUID: "));
      Serial.println(WRITE_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(F("[BT] - Found our Write characteristic"));

        // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteNotifyCharacteristic = pRemoteService->getCharacteristic(NOTIFY_UUID);
    if (pRemoteNotifyCharacteristic == nullptr) {
      Serial.print(F("[BT] Failed to find our characteristic UUID: "));
      Serial.println(NOTIFY_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(F("[BT] - Found our Write characteristic"));

    // Read the value of the characteristic.
    if(pRemoteWriteCharacteristic->canRead()) {
      std::string value = pRemoteWriteCharacteristic->readValue();
      Serial.print(F("[BT] The characteristic value was: "));
      Serial.println(value.c_str());
    }

    if(pRemoteNotifyCharacteristic->canNotify())
      pRemoteNotifyCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    lastBTMessage = millis();   // start the poll/watchdog clock from the connection, not from boot
     #ifdef RELAISMODE
      #ifdef DEBUG
        Serial.println(F("[BT] activate relais contact"));
      #endif
      digitalWrite(RELAIS_PIN, RELAIS_HIGH);
    #endif

    return true;
}


void handleBTCommandQueue(){

    bt_command_t command;
    if(xQueueReceive(sendQueue, &command, 0)) {
      
      if (pRemoteWriteCharacteristic == nullptr) {
        return;   // link dropped between queueing and writing; drop the frame
      }
#ifdef DEBUG
    Serial.print("[BT] Write Request FF02 - Value: ");
    
    for(int i=0; i<8; i++){
       if ( i % 2 == 0){ Serial.print(" "); };
       Serial.printf("%02x", ((uint8_t*)&command)[i]);
    }
    
    Serial.println("");
#endif
      pRemoteWriteCharacteristic->writeValue((uint8_t*)&command, sizeof(command),true);
 
     };  
}

void sendBTCommand(bt_command_t command){
    bt_command_t cmd = command;
    xQueueSend(sendQueue, &cmd, 0);
}

void handleBluetooth(){

  if (doConnect == true) {
    doConnect = false;
    if (everAttemptedConnect && (millis() - lastConnectAttempt) < connectBackoffMs()) {
      // Still inside the backoff window; keep scanning, do not retry yet.
      Serial.printf("[BT] backing off: %u consecutive failure(s), next attempt in %lus\n",
                    (unsigned) connectFailures,
                    (connectBackoffMs() - (millis() - lastConnectAttempt)) / 1000UL);
    } else {
      everAttemptedConnect = true;
      lastConnectAttempt = millis();
      if (connectToServer()) {
        connectFailures = 0;
        Serial.println(F("We are now connected to the Bluetti BLE Server."));
      } else {
        if (connectFailures < 6) { connectFailures++; }
        Serial.println(F("Failed to connect to the power station; retrying on the next scan."));
        scanDue = true;
      }
    }
  }

  // Last resort only: five minutes with no BLE message at all.
  if ((millis() - lastBTMessage) > (MAX_DISCONNECTED_TIME_UNTIL_REBOOT * 60000)){ 
    Serial.println(F("[BT] disconnected over allowed limit, reboot device"));
    #ifdef SLEEP_TIME_ON_BT_NOT_AVAIL
        esp_deep_sleep_start();
    #else
        ESP.restart();
    #endif
  }

  if (connected) {

    // poll for device state
    if ( millis() - lastBTMessage > BLUETOOTH_QUERY_MESSAGE_DELAY){

       bt_command_t command;
       command.prefix = 0x01;
       command.field_update_cmd = 0x03;
       command.page = bluetti_polling_command[pollTick].f_page;
       command.offset = bluetti_polling_command[pollTick].f_offset;
       command.len = (uint16_t) bluetti_polling_command[pollTick].f_size << 8;
       command.check_sum = modbus_crc((uint8_t*)&command,6);

       xQueueSend(commandHandleQueue, &command, portMAX_DELAY);
       xQueueSend(sendQueue, &command, portMAX_DELAY);

       if (pollTick == sizeof(bluetti_polling_command)/sizeof(device_field_data_t)-1 ){
           pollTick = 0;
       } else {
           pollTick++;
       }
            
      lastBTMessage = millis();
    }

    handleBTCommandQueue();
    
  } else if (!scanInProgress && (scanDue || (millis() - lastScanStart) >= ((unsigned long)BLUETOOTH_SCAN_INTERVAL_IN_SECONDS * 1000UL))) {
    // Periodic rescan. Recovery from a missed match or a dropped link must not
    // depend on the watchdog reboot; upstream only ever rescanned after a
    // *successful* match, so a single missed window was permanent.
    scanDue = false;
    lastScanStart = millis();
    scanInProgress = true;
    BLEDevice::getScan()->clearResults();   // no unbounded result growth across rescans
    if (!BLEDevice::getScan()->start(BLUETOOTH_SCAN_DURATION_IN_SECONDS, onScanComplete)) {
      scanInProgress = false;
      Serial.println(F("[BT] scan start failed; will retry"));
    }
  }
}

void btResetStack()
{
  connected=false;

}

bool isBTconnected(){
  return connected;
}

unsigned long getLastBTMessageTime(){
    return lastBTMessage;
}


