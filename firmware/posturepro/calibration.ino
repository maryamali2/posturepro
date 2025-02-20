float dt_calibration = 0.0;
float lp_calibrated[DATA_SIZE];

// EsTimates
float roll_at_rest = 0.0;
float pitch_at_rest = 0.0;
float emg1_at_rest = 0;
float emg2_at_rest = 0;
float emg3_at_rest = 0;
float emg4_at_rest = 0;

// Sums
float sum_roll = 0.0;
float sum_pitch = 0.0;

void calibrate_rest() {
  // Initialize rest array
  Serial.println("Starting baseline calibration...");
  calibrated = 1;
  reset(rest, PACKET_SIZE);

  int numValues = 0;
  unsigned long start_time_cal = millis();
  unsigned long start_dt = millis();

  while (millis() - start_time_cal < 5000) {
    read_imu(&mpu, packet);
    read_emg(&emg, packet);

    simple_lowpass(lp_calibrated, packet);

    dt_calibration = (millis() - start_dt) * 1/1000;
    start_dt = millis();

    complementary_filter(dt_calibration, lp_calibrated, &roll_at_rest, &pitch_at_rest);

    sum_roll += roll_at_rest;
    sum_pitch += pitch_at_rest;
    emg1_at_rest += packet[6];
    // emg2_at_rest += packet[7];
    // emg3_at_rest += packet[8];
    // emg4_at_rest += packet[9];
    numValues += 1;
  }

  // Get average
  roll_at_rest = sum_roll / numValues;
  pitch_at_rest = sum_pitch / numValues;

  memcpy(&packet[DATA_SIZE], &CALIBRATION_PACKET, sizeof(float));
  send_ble(packet, PACKET_SIZE, true);

  // Serial.print("Calibrated Roll is: ");
  // Serial.print(roll_at_rest);
  // Serial.print("  Calibrated Pitch is: ");
  // Serial.println(pitch_at_rest);

  Serial.println("Completed baseline calibration.");
}

void calibrate_moving() {

}

void calibrate_emg() {
  Serial.println("Calibrating EMG...");
  max_emg = INT_MIN;
  // max_emg = 0;

  Serial.println("Flex your muscle to the maximum extent...");
  unsigned long startCal = millis();

  while (millis() - startCal < 10000) {
    read_emg(&emg, packet);
    if (packet[8] > max_emg) {
      max_emg = packet[8];
    }
  }

  Serial.println("Completed EMG calibration.");
}