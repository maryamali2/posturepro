#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float* emg_readings;
float* imu_readings;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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

  emg_readings = read_emg(&emg);
  imu_readings = read_imu(&mpu);

  Serial.print("EMG: ");
  Serial.print(emg_readings[0]);
  Serial.print(", ");
  Serial.print(emg_readings[1]);
  Serial.print(", ");
  Serial.println(emg_readings[2]);

  Serial.print("Acceleration: ");
  Serial.print(imu_readings[0]);
  Serial.print(", ");
  Serial.print(imu_readings[1]);
  Serial.print(", ");
  Serial.println(imu_readings[2]);
  
  Serial.print("Rotation: ");
  Serial.print(imu_readings[3]);
  Serial.print(", ");
  Serial.print(imu_readings[4]);
  Serial.print(", ");
  Serial.println(imu_readings[5]);

  delay(1000);
}
