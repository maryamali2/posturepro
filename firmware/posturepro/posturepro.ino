#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "limits.h"

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
#define ZERO_DEPTH 3

typedef struct dataRead {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  float emg_1;
  float emg_2;
  float emg_3;
  float emg_4;
} dataRead_t;

dataRead_t dataRead;

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
float rest_acc = 0.0;
float max_acc_going_down = -1000.0;
float min_acc_going_up = 1000.0;

float vel = 0.0;
int acc_state = 0;

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
  status = initialize_emg(&emg2, A1);
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

  startTime = millis();
  
  Serial.println("Setup done");
}

void loop() {
  startTime = millis();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     READ DATA FROM SENSORS       ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  read_imu(&mpu, &dataRead);
  read_emg(&emg1, 1, &dataRead);
  read_emg(&emg2, 2, &dataRead);
  read_emg(&emg3, 3, &dataRead);
  read_emg(&emg4, 4, &dataRead);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////     COMPLEMENTARY FILTER       /////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  dt = (millis() - startTime) / 1000.0;
  // float dt_z = dt * 10000000;
  // startTime = millis();

  complementary_filter(dt, &roll_estimate, &pitch_estimate, &dataRead);

  // pitch_estimate *= 2;
  // if (rest_depth != 0.0) {
  //   // depth += (-lp_packet[0]*sin(pitch_estimate) + lp_packet[1]*cos(pitch_estimate)*sin(roll_estimate) + lp_packet[2]*cos(pitch_estimate)*cos(roll_estimate)) - rest_depth;
  // }

  float sin_phi = sinf(roll_estimate);
  float cos_phi = cosf(roll_estimate);
  float sin_theta = sinf(pitch_estimate);
  float cos_theta = cosf(pitch_estimate);

  float z_acc = ((-dataRead.acc_x * sin_theta) + ((dataRead.acc_y * sin_phi + dataRead.acc_z * cos_phi) * cos_theta)) - 9.7;

  z_acc = abs(z_acc) > .5 ? z_acc : 0.0;

  if (abs(dataRead.gyr_x) > 0.5 || abs(dataRead.gyr_y) > 0.5 || abs(dataRead.gyr_z) > 0.5) {
    z_acc = 0.0;
  } else {
    if (z_acc == 0.0) {
      acc_state++;
    } else {
      acc_state = 0;
    }

    if (acc_state > 10) {
      vel = 0.0;
      depth = 0.0; //
    } else {
      vel += z_acc;
    }
  }

  depth += vel;


  // depth = dataRead.acc_x - 9.81;
  // depth -= rest_depth;

  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////        PRINT FOR DEBUGGING         ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Serial.print("Roll: ");
  // Serial.print(roll_estimate);
  // Serial.print("    Pitch: ");
  // Serial.print(pitch_estimate);

  Serial.print(" DEPTAH: ");
  Serial.print(z_acc);

  Serial.print("  Depth: ");
  Serial.print(depth);

  // Serial.print("  Rest Depth: ");
  // Serial.print(rest_depth);

  // Serial.print("  Calced Z: ");
  // Serial.print(-lp_packet[0]*sin(pitch_estimate) + lp_packet[1]*cos(pitch_estimate)*sin(roll_estimate) + lp_packet[2]*cos(pitch_estimate)*cos(roll_estimate));

  // Serial.print(" X: ");
  // Serial.print(lp_packet[0]);

  // Serial.print(" Y: ");
  // Serial.print(lp_packet[1]);

  // Serial.print(" Z: ");
  // Serial.print(lp_packet[2]);

  Serial.print("  EMG 1: ");
  Serial.print(dataRead.emg_1);

  Serial.print("  EMG 2: ");
  Serial.print(dataRead.emg_2);

  Serial.print("  Roll Rest : ");
  Serial.print(roll_at_rest);

  Serial.print("  Current Roll : ");
  Serial.print(roll_estimate);

  Serial.print("  Roll Diff : ");
  Serial.print(roll_estimate - roll_at_rest);

  // Serial.print("  roll : ");
  // Serial.print(roll_estimate - roll_at_rest);

  // Serial.print("  EMG 2: ");
  // Serial.print(dataRead.emg_2);

  // Serial.print("  EMG 3: ");
  // Serial.print(dataRead.emg_3);

  // Serial.print("  EMG 4: ");
  // Serial.print(dataRead.emg_4);

  // Serial.print("  ADC: ");
  // Serial.println(analogRead(A0) * 10000);

  Serial.println();



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////             SEND TO APP             ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  final_packet[0] = roll_estimate - roll_at_rest;
  final_packet[1] = pitch_estimate - pitch_at_rest;
  final_packet[2] = depth;
  final_packet[3] = dataRead.emg_1;
  final_packet[4] = dataRead.emg_2;
  final_packet[5] = dataRead.emg_3;
  final_packet[6] = dataRead.emg_4;

  memcpy(&final_packet[DATA_SIZE], &DATA_PACKET, sizeof(float));
  send_ble(final_packet, PACKET_SIZE, true);

  // delay(100);
}
