#include <ArduinoBLE.h>

// Define a custom BLE service and characteristic
BLEService customService("180C"); // Custom service UUID
BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set device name and advertise the service
  BLE.setLocalName("ArduinoBLE");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(dataCharacteristic);
  BLE.addService(customService);

  BLE.advertise();
  Serial.println("BLE device is now advertising...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    int i = 0;

    // Stream data while the central is connected
    while (central.connected()) {
      dataCharacteristic.writeValue(i++); // Notify central of new value

      // Serial.print("Sent value: ");
      // Serial.println(sensorValue);

      delay(1000); // Adjust for desired streaming rate
    }

    Serial.println("Central disconnected.");
  }
}
