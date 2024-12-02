#include <ArduinoBLE.h>

// Define a custom BLE service and characteristic
BLEService customService("180C"); // Custom service UUID
BLEFloatCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

int initialize_ble() {
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    return 1;
  }

  // Set device name and advertise the service
  BLE.setLocalName("ArduinoBLE");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(dataCharacteristic);
  BLE.addService(customService);

  BLE.advertise();
  Serial.println("BLE device is now advertising...");

  return 0;
}

int send_ble(float value) {
  BLEDevice central = BLE.central();
  Serial.print("Value received is: ");
  Serial.print(value);

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    int i = 0;

    // Stream data while the central is connected
    if (central.connected()) {
      dataCharacteristic.writeValue(value); // Notify central of new value

      // Serial.print("Sent value: ");
      // Serial.println(sensorValue);

      // delay(1000); // Adjust for desired streaming rate
    } else {
      Serial.print("Central disconnected.");
      return -1;
    }

    return value;
  }
}