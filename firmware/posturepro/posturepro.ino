#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <ArduinoBLE.h>

#define PACKET_SIZE 9

// Define a custom BLE service and characteristic
// BLEService customService("180C"); // Custom service UUID
// BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float packet[PACKET_SIZE];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  status = initialize_ble();
  if (status) {
    Serial.println("Error initialziing BLE");
    while(1);
  }

  status = initialize_emg(&emg);
  if (status) {
    Serial.println("Error initialziing EMG");
    while(1);
  }

  status = initialize_imu(&mpu);
  if (status) {
    Serial.println("Error initialziing MPU");
    while(1);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  read_imu(&mpu, packet);
  read_emg(&emg, packet);

  for (int i = 0; i < PACKET_SIZE; i++) {
    Serial.print("Sending data at position: ");
    Serial.print(i);
    Serial.print(" with value: ");
    Serial.println(packet[i]);
    status = send_ble(packet[i]);

    if (status == -1) {
      Serial.println("Error sending over BLE");
    } else if (status == packet[i]) {
      Serial.println("Success");
    } else {
      Serial.print("Huh: ");
      Serial.println(status);
    }

    delay(1);
  }

  // Serial.print("EMG: ");
  // Serial.print(packet[0]);
  // Serial.print(", ");
  // Serial.print(packet[1]);
  // Serial.print(", ");
  // Serial.println(packet[2]);

  // Serial.print("Acceleration: ");
  // Serial.print(packet[3]);
  // Serial.print(", ");
  // Serial.print(packet[4]);
  // Serial.print(", ");
  // Serial.println(packet[5]);
  
  // Serial.print("Rotation: ");
  // Serial.print(packet[6]);
  // Serial.print(", ");
  // Serial.print(packet[7]);
  // Serial.print(", ");
  // Serial.println(packet[8]);

  delay(1000);
}
