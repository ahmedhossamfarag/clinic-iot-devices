#include <ArduinoBLE.h>

#define DEVICE_NAME "CLIENT-IOT-BLE"
#define SERVICE_ID "a38deac2-1ab2-4d23-8ba4-e68399782297"

#define MIN_RSSI -60
#define RESTART_INTERVAL 3000

unsigned long lastSeen = 0;

void setup()
{
    // 1) Configure Serial
    Serial.begin(115200);

    while (!Serial)
        ;

    // 2) Initialize BLE

    if (!BLE.begin())
    {
        Serial.println("BLE start failed");
        while (1)
            ;
    }

    // 3) Start scanning
    BLE.scan(true);

    Serial.println("Scanning...");
}

void loop()
{
    // 4) Check if a peripheral has been discovered
    BLEDevice peripheral = BLE.available();

    if (peripheral)
    {
        // 5) Print peripheral information if it is a device we are interested in
        if (peripheral.localName() == DEVICE_NAME &&
            peripheral.hasAdvertisedServiceUuid() &&
            peripheral.advertisedServiceUuid() == SERVICE_ID)
        {
            lastSeen = millis();

            if (peripheral.rssi() >= MIN_RSSI)
            {
                Serial.print("{\"mac\":\"");
                Serial.print(peripheral.address());
                Serial.print("\",\"rssi\":");
                Serial.print(peripheral.rssi());
                Serial.println("}");
            }
        }
    }

    // Recover scanner if stack stalls
    if (millis() - lastSeen > RESTART_INTERVAL)
    {
        BLE.stopScan();
        delay(100);
        BLE.scan(true);

        lastSeen = millis();
    }
}