#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "limits.h"
// #include <ArduinoBLE.h>

#define PACKET_SIZE 9

// Define a custom BLE service and characteristic
// BLEService customService("180C"); // Custom service UUID
// BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float packet[PACKET_SIZE];
float rest[PACKET_SIZE];
float max_emg;

void reset(float* packet) {
  for (int i = 0; i < PACKET_SIZE; ++i) {
    packet[i] = 0;
  }
}

void calibrate_rest() {
  // Initialize rest array
  Serial.println("Starting baseline calibration...");
  reset(rest);

  unsigned long startTime = millis();
  int numValues = 0;

  while (millis() - startTime < 10000) {
    read_imu(&mpu, packet);
    read_emg(&emg, packet);
    for (int j = 0; j < PACKET_SIZE; ++j) {
      rest[j] += packet[j];
    }
    numValues += 1;
  }

  for (int j = 0; j < PACKET_SIZE; ++j) {
      rest[j] = rest[j] / numValues;
  }

  // Serial.print("The rest value for AccX is: ");
  // Serial.println(rest[0]);
  // Serial.print("The rest value for AccY is: ");
  // Serial.println(rest[1]);
  // Serial.print("The rest value for AccZ is: ");
  // Serial.println(rest[2]);

  Serial.println("Completed baseline calibration.");
}

void calibrate_emg() {
  Serial.println("Calibrating EMG...");
  max_emg = INT_MIN;

  Serial.println("Flex your muscle to the maximum extent...");
  unsigned long startTime = millis();
  int numValues = 0;

  while (millis() - startTime < 10000) {
    read_emg(&emg, packet);
    if (packet[8] > max_emg) {
      max_emg = packet[8];
    }
  }

  Serial.println("Completed EMG calibration.");
}

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
    Serial.println("Error initializing MPU");
    while(1);
  }

  calibrate_rest();
  calibrate_emg();

}

void loop() {
  // put your main code here, to run repeatedly:
  read_imu(&mpu, packet);
  read_emg(&emg, packet);

  // Serial.println("Sending complete data packet...");
  // int status = send_ble(packet, PACKET_SIZE, true);

  // Serial.print("EMG: ");
  // Serial.print(packet[6]);
  // Serial.print(", ");
  // Serial.print(packet[7]);
  // Serial.print(", ");
  // Serial.println(packet[8]);

  Serial.print("Acceleration: ");
  Serial.print(packet[0]);
  Serial.print(", ");
  Serial.print(packet[1]);
  Serial.print(", ");
  Serial.println(packet[2]);
  
  Serial.print("Rotation: ");
  Serial.print(packet[3]);
  Serial.print(", ");
  Serial.print(packet[4]);
  Serial.print(", ");
  Serial.println(packet[5]);

  delay(1000);
}
