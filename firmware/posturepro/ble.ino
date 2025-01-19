#include <ArduinoBLE.h>

// Define a custom BLE service and characteristic
BLEService customService("180C"); // Custom service UUID
BLECharacteristic dataCharacteristic("2A57", BLERead | BLENotify | BLEWrite, PACKET_SIZE * sizeof(float)); // Notify characteristic

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

int send_ble(float* values, int num_values, bool ack) {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    if (central.connected()) {
      uint8_t dataBuffer[PACKET_SIZE * sizeof(float)]; // Allocate space for all values

      // Convert float values to bytes
      for (int i = 0; i < num_values; i++) {
        memcpy(&dataBuffer[i * sizeof(float)], &values[i], sizeof(float));
      }

      // Send entire packet as a single BLE notification
      dataCharacteristic.writeValue(dataBuffer, PACKET_SIZE * sizeof(float));

      Serial.println("Sent full data packet over BLE.");

      // Acknowledgment handling (if necessary)
      unsigned long startTime = millis();
      while (ack && (millis() - startTime < 100)) {
        float receivedValue;
        dataCharacteristic.readValue((uint8_t*)&receivedValue, sizeof(float));
        if (isnan(receivedValue)) {
          Serial.println("Received ack");
          break;
        }
        delay(10);
      }

      return 0;  // Success
    } else {
      Serial.println("Central disconnected.");
      return -1;
    }
  }

  return -1; // No central device connected
}