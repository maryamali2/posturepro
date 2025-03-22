#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "limits.h"
// #include <ArduinoBLE.h>

#define DATA_SIZE 7
#define HEADER_SIZE 1
#define PACKET_SIZE (HEADER_SIZE + DATA_SIZE)
#define RAD_TO_DEG (180 / 3.14159)
#define ANGLE_MARGIN 8.0
#define GRAVITY 9.81
#define ALPHA 0.2
#define EMG_THRESHOLD 400

#define NO_CALIBRATION 0
#define CALIBRATE_REST 1
#define CALIBRATE_MOVING 2


// Define a custom BLE service and characteristic
// BLEService customService("180C"); // Custom service UUID
// BLEIntCharacteristic dataCharacteristic("2A57", BLERead | BLENotify); // Notify characteristic

// Sensors
MyoWare emg1;
MyoWare emg2;
MyoWare emg3;
MyoWare emg4;
Adafruit_MPU6050 mpu;

int status;
float packet[PACKET_SIZE];                    // For first reading in data values
float lp_packet[DATA_SIZE];                   // Applies simple averaging filter
float final_packet[DATA_SIZE];                // Data packet sent to app
float calibrated_packet[DATA_SIZE];     // Calibrated packet sent to app
float temp_cal_packet[DATA_SIZE];       // Used in calibration function
// float euler[2];                            // 0 is Roll, 1 is Pitch
// float base_angles[3];
float max_emg;
int calibrated = 0;

const uint32_t DATA_PACKET = 0;
const uint32_t CALIBRATION_REST_PACKET = 1;
const uint32_t CALIBRATION_MOVING_PACKET = 2;

unsigned long startTime;
float roll_calibrated = 0.0;
float pitch_calibrated = 0.0;
float roll_estimate = 0.0;
float pitch_estimate = 0.0;
float depth = 0.0;
float dt = 0.0;

// Initial Estimates (for hardcoding -- if we don't use ML)
float roll_temp = 0.0;
float pitch_temp = 0.0;
float roll_at_rest = 0.0;
float pitch_at_rest = 0.0;
float emg1_at_rest = 0;
float emg2_at_rest = 0;
float emg3_at_rest = 0;
float emg4_at_rest = 0;

void simple_lowpass(float* lp_output, float* data_input) {
  for (int i = 0; i < DATA_SIZE; ++i) {
    lp_output[i] = (lp_output[i] + data_input[i]) / 2;
  }
}

void reset(float* packet, int size) {
  for (int i = 0; i < size; ++i) {
    packet[i] = 0;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     BLE SET UP       ///////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Initializing BLE");
  status = initialize_ble();
  if (status) {
    Serial.println("Error initializing BLE");
    while(1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     EMG SET UP       ///////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Initializing EMG");
  status = initialize_emg(&emg1, A0);
  if (status) {
    Serial.println("Error initializing EMG 1");
    while(1);
  }
  status = initialize_emg(&emg2, A2);
  if (status) {
    Serial.println("Error initializing EMG 2");
    while(1);
  }
  status = initialize_emg(&emg3, A2);
  if (status) {
    Serial.println("Error initializing EMG 3");
    while(1);
  }
  status = initialize_emg(&emg4, A3);
  if (status) {
    Serial.println("Error initialziing EMG 4");
    while(1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     IMU SET UP       ///////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Initializing IMU");
  status = initialize_imu(&mpu);
  if (status) {
    Serial.println("Error initializing MPU");
    while(1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     CALIBRATE REST       ///////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  reset(packet, DATA_SIZE);
  reset(lp_packet, DATA_SIZE);
  reset(calibrated_packet, DATA_SIZE);
  reset(temp_cal_packet, DATA_SIZE);
  // reset(base_angles, 3);

  startTime = millis();
  
  Serial.println("Setup done");
}

void loop() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     READ DATA FROM SENSORS       ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  read_imu(&mpu, packet);
  read_emg(&emg1, packet, 6);
  read_emg(&emg2, packet, 7);

  // Filter
  simple_lowpass(lp_packet, packet);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     COMPLEMENTARY FILTER       /////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  dt = (millis() - startTime) * 1/1000;
  startTime = millis();

  complementary_filter(dt, lp_packet, &roll_estimate, &pitch_estimate);

  depth = lp_packet[0]*sin(roll_estimate) - lp_packet[1]*cos(roll_estimate)*sin(pitch_estimate) + lp_packet[2]*cos(pitch_estimate)*cos(roll_estimate);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////        PRINT FOR DEBUGGING         ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print("Roll: ");
  Serial.print(roll_estimate);
  Serial.print("    Pitch: ");
  Serial.print(pitch_estimate);

  Serial.print("  EMG 1: ");
  Serial.print(lp_packet[6]);

  Serial.print("  EMG 2: ");
  Serial.println(lp_packet[7]);

  Serial.print("  EMG 3: ");
  Serial.println(lp_packet[8]);

  Serial.print("  EMG 4: ");
  Serial.println(lp_packet[9]);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////             SEND TO APP             ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  final_packet[0] = roll_estimate - roll_at_rest;
  final_packet[1] = pitch_estimate - pitch_at_rest;
  final_packet[2] = depth;
  final_packet[3] = lp_packet[6];
  final_packet[4] = lp_packet[7];
  final_packet[5] = lp_packet[8];
  final_packet[6] = lp_packet[9];

  memcpy(&final_packet[DATA_SIZE], &DATA_PACKET, sizeof(float));
  send_ble(final_packet, PACKET_SIZE, true);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     TILT & MUSCLE DETECTION       //////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial.print("Tilt Sideways: ");
  // // Serial.print("Tilt Front/Back: ");
  // if (abs(roll_estimate - roll_at_rest) > 0.30) {
  //   Serial.print("Y");
  // } else {
  //   Serial.print("N");
  // }
  // // Serial.print("  Tilt Sideways: ");
  // Serial.print("    Tilt Front/Back: ");
  // if (abs(pitch_estimate - pitch_at_rest) > 0.30) {
  //   Serial.println("Y");
  // } else {
  //   Serial.println("N");
  // }
  
  // Serial.print("    Muscle Flex: ");
  // if (packet[6] > emg1_at_rest + EMG_THRESHOLD) {
  //   Serial.println("Y");
  // } else {
  //   Serial.println("N");
  // }
  // Serial.println(packet[6]);

  delay(50);
}
