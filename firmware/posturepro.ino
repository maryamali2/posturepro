#include <MyoWare.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

MyoWare emg;
Adafruit_MPU6050 mpu;
int status;
float packet[9];

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

  read_emg(&emg, packet);
  read_imu(&mpu, packet);

  Serial.print("EMG: ");
  Serial.print(packet[0]);
  Serial.print(", ");
  Serial.print(packet[1]);
  Serial.print(", ");
  Serial.println(packet[2]);

  Serial.print("Acceleration: ");
  Serial.print(packet[3]);
  Serial.print(", ");
  Serial.print(packet[4]);
  Serial.print(", ");
  Serial.println(packet[5]);
  
  Serial.print("Rotation: ");
  Serial.print(packet[6]);
  Serial.print(", ");
  Serial.print(packet[7]);
  Serial.print(", ");
  Serial.println(packet[8]);

  delay(1000);
}
