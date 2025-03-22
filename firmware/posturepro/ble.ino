#include <ArduinoBLE.h>

void onCalibrationWritten(BLEDevice central, BLECharacteristic characteristic);

// Define a custom BLE service and characteristic
BLEService customService("1201"); // Custom service UUID
BLECharacteristic dataCharacteristic("2A58", BLERead | BLEWrite | BLENotify, PACKET_SIZE * sizeof(float)); // Data characteristic
BLEByteCharacteristic calibrationCharacteristic("2A57", BLERead | BLEWrite | BLENotify); // calibration button


void onCalibrationWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t command;
  characteristic.readValue(&command, sizeof(command));

  if (command == CALIBRATE_REST) {
    Serial.println("Calibration started...");
    calibrate_rest();
    
    command = NO_CALIBRATION;
    characteristic.writeValue(command, false);
  } else if (command == CALIBRATE_MOVING) {
    Serial.println("Calibration started...");
    calibrate_moving();
    
    command = NO_CALIBRATION;
    characteristic.writeValue(command, false);
  }
}

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
  customService.addCharacteristic(calibrationCharacteristic);
  BLE.addService(customService);

  calibrationCharacteristic.setEventHandler(BLEWritten, onCalibrationWritten);

  BLE.advertise();
  Serial.println("BLE device is now advertising...");

  return 0;
}

int send_ble(float* values, int num_values, bool ack) {
  BLEDevice central = BLE.central();

  if (central) {
    // Serial.print("Connected to central: ");
    // Serial.println(central.address());

    if (central.connected()) {
      uint8_t dataBuffer[num_values * sizeof(float)]; // Allocate space for all values

      // Convert float values to bytes
      for (int i = 0; i < num_values; i++) {
        memcpy(&dataBuffer[i * sizeof(float)], &values[i], sizeof(float));
      }

      // Send entire packet as a single BLE notification
      dataCharacteristic.writeValue(dataBuffer, num_values * sizeof(float));

      // Serial.println("Sent full data packet over BLE.");

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
