
#include <ArduinoBLE.h>    // 🛠 Core BLE library for radio control

#define DEVICE_NAME "CLIENT-IOT-BLE"
#define DEVICE_ID "a38deac2-1ab2-4d23-8ba4-e68399782297"
// 160 units * 0.625 ms = 100 milliseconds (advertises 10 times a second).
#define ADV_INTERVAL 1600 * 5


// Advertise String
const char advString[] = DEVICE_NAME;

void setup() {
  // 1) Initialize BLE radio
  if (!BLE.begin()) {
    while (1);                // 🚫 Halt in an infinite loop if the Bluetooth radio is broken
  }

  // 2) Set the device name 
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setLocalName(DEVICE_NAME);

  // 3) Set the advertise interval
  BLE.setAdvertisingInterval(ADV_INTERVAL);

  // 4) Set the advertise data to the device ID.
  BLE.setManufacturerData(advString, sizeof(advString) - 1);

  // 6) Start advertising
  BLE.advertise();
}

void loop() {
  
}