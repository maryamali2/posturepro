#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "limits.h"
// #include <ArduinoBLE.h>

#define DATA_SIZE 9
#define HEADER_SIZE 1
#define PACKET_SIZE (HEADER_SIZE + DATA_SIZE)
#define RAD_TO_PI_FACTOR (180 / 3.14159)
#define ANGLE_MARGIN 8.0


// Define a custom BLE service and characteristic
// BLEService customService("180C"); // Custom service UUID
// BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float packet[PACKET_SIZE];
float rest[DATA_SIZE];
float base_angles[3];
float max_emg;
int calibrated = 0;

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
  calibrated = 1;
  reset(rest);

  unsigned long startTime = millis();
  int numValues = 0;

  while (millis() - startTime < 5000) {
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

  base_angles[0] = acos(packet[0] / 9.81) * RAD_TO_PI_FACTOR;
  base_angles[1] = acos(packet[1] / 9.81) * RAD_TO_PI_FACTOR;
  base_angles[2] = acos(packet[2] / 9.81) * RAD_TO_PI_FACTOR;

  Serial.print("Base: ");
  Serial.print(base_angles[0]);
  Serial.print(", ");
  Serial.print(base_angles[1]);
  Serial.print(", ");
  Serial.print(base_angles[2]);

  // memcpy(&packet[DATA_SIZE], &CALIBRATION_PACKET, sizeof(float));
  // send_ble(packet, PACKET_SIZE, true);


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

int check_tilt() {
  if (calibrated) {
    float angleX = acos(packet[0] / 9.81) * RAD_TO_PI_FACTOR;
    float angleY = acos(packet[1] / 9.81) * RAD_TO_PI_FACTOR;
    float angleZ = acos(packet[2] / 9.81) * RAD_TO_PI_FACTOR;

    if ((angleX - base_angles[0] > ANGLE_MARGIN) && (angleY - base_angles[1] < -ANGLE_MARGIN)) {
      return 1;
    } else if ((angleX - base_angles[0] < -ANGLE_MARGIN) && (angleY - base_angles[1] > ANGLE_MARGIN)) {
      return 2;
    } else {
      return 0;
    }
  } else {
    return -1;
  }
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

  // calibrate_rest();

  // delay(3000);
  
  Serial.println("Setup done");
}

void loop() {
  // put your main code here, to run repeatedly:
  read_imu(&mpu, packet);
  read_emg(&emg, packet);

  memcpy(&packet[DATA_SIZE], &DATA_PACKET, sizeof(float));
  status = send_ble(packet, PACKET_SIZE, true);

  status = check_tilt();
  switch (status) {
    case -1:
      Serial.println("Error: IMU not calibrated");
      break;
    case 0:
      Serial.println("NEUTRAL");
      break;
    case 1:
      Serial.println("TILT RIGHT");
      break;
    case 2:
      Serial.println("TILT LEFT");
      break;
  }
  

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

  delay(1000);
}
