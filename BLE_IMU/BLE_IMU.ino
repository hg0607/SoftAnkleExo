/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
//#include "BLEScan.h"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ffe5-0000-1000-8000-00805f9a34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("0000ffe4-0000-1000-8000-00805f9a34fb");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static esp_bd_addr_t* myaddr;
int led = 8;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.write(pData, length);
    // Serial.println();
    static int count = 0;
    if(count % 3 == 0)
      digitalWrite(led, HIGH);   // turn the LED on 
    else
      digitalWrite(led, LOW);  
    count++; 
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    // Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    // Serial.print("Forming a connection to ");
    // Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    // Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    // Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      // Serial.print("Failed to find our service UUID: ");
      // Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    // Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      // Serial.print("Failed to find our characteristic UUID: ");
      // Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      // Serial.print("The characteristic value was: ");
      // Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Serial.print("BLE Advertised Device found: ");
    // Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    // if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) 
    if (advertisedDevice.haveName() && (advertisedDevice.getName()=="WT901BLE67"))
    {
      
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

      BLEAddress address = myDevice->getAddress();
      esp_bd_addr_t* addr = address.getNative();     
      esp_err_t re = esp_ble_gap_set_prefer_conn_params(*addr, 6,6,0,200);
      Serial.println("esp_ble_gap_set_prefer_conn_params result:");
      Serial.println(re, DEC);

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200, SERIAL_8N1, 1, 0);
  // Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("BLE-Client");
  BLEAddress myaddress = BLEDevice::getAddress();
  myaddr = myaddress.getNative();

  pinMode(led, OUTPUT);
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      // Serial.println("We are now connected to the BLE Server.");
      esp_gap_conn_params_t conn_params;
      BLEAddress address = myDevice->getAddress();
      esp_bd_addr_t* addr = address.getNative();     
      esp_ble_get_current_conn_params(*addr, &conn_params);
      Serial.println("server interval:");
      Serial.println(conn_params.interval, DEC);

      esp_ble_get_current_conn_params(*myaddr, &conn_params);
      Serial.println("client interval:");
      Serial.println(conn_params.interval, DEC);

    } else {
      // Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    // String newValue = "Time since boot: " + String(millis()/1000);
    // Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop
