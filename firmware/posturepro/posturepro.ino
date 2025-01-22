#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "limits.h"
// #include <ArduinoBLE.h>

#define DATA_SIZE 9
#define HEADER_SIZE 1
#define PACKET_SIZE (HEADER_SIZE + DATA_SIZE)


// Define a custom BLE service and characteristic
// BLEService customService("180C"); // Custom service UUID
// BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float packet[PACKET_SIZE];
float rest[DATA_SIZE];
float max_emg;

const uint32_t DATA_PACKET = 0;
const uint32_t CALIBRATION_PACKET = 1;

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
    for (int j = 0; j < DATA_SIZE; ++j) {
      rest[j] += packet[j];
    }
    numValues += 1;
  }

  for (int j = 0; j < DATA_SIZE; ++j) {
    packet[j] = rest[j] / numValues;
  }

  memcpy(&packet[DATA_SIZE], &CALIBRATION_PACKET, sizeof(float));
  send_ble(packet, PACKET_SIZE, true);


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
  while (!Serial);

  Serial.println("Initializing BLE");
  status = initialize_ble();
  if (status) {
    Serial.println("Error initializing BLE");
    while(1);
  }

  Serial.println("Initializing EMG");
  status = initialize_emg(&emg);
  if (status) {
    Serial.println("Error initialziing EMG");
    while(1);
  }

  Serial.println("Initializing IMU");
  status = initialize_imu(&mpu);
  if (status) {
    Serial.println("Error initializing MPU");
    while(1);
  }

  
  Serial.println("Setup done");
}

void loop() {
  // put your main code here, to run repeatedly:
  read_imu(&mpu, packet);
  read_emg(&emg, packet);

  memcpy(&packet[DATA_SIZE], &DATA_PACKET, sizeof(float));
  int status = send_ble(packet, PACKET_SIZE, true);

  // Serial.print("EMG: ");
  // Serial.print(packet[6]);
  // Serial.print(", ");
  // Serial.print(packet[7]);
  // Serial.print(", ");
  // Serial.println(packet[8]);

  // Serial.print("Acceleration: ");
  // Serial.print(packet[0]);
  // Serial.print(", ");
  // Serial.print(packet[1]);
  // Serial.print(", ");
  // Serial.println(packet[2]);
  
  // Serial.print("Rotation: ");
  // Serial.print(packet[3]);
  // Serial.print(", ");
  // Serial.print(packet[4]);
  // Serial.print(", ");
  // Serial.println(packet[5]);

  delay(10);
}
