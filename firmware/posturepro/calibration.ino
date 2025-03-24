float dt_calibration = 0.0;
float lp_calibrated[DATA_SIZE];

// Sums
float sum_roll = 0.0;
float sum_pitch = 0.0;

void calibrate_rest() {
  // Initialize rest array
  Serial.println("Starting baseline calibration...");
  reset(calibrated_packet, DATA_SIZE);
  reset(temp_cal_packet, DATA_SIZE);

  int numValues = 0;
  unsigned long calibration_start = millis();
  unsigned long start_dt = millis();

  while (millis() - calibration_start < 5000) {
    // Read IMU values into packet

    //////////// Using temp_cal_packet
    read_imu(&mpu, temp_cal_packet);

    // Read 4 EMG values into packet
    read_emg(&emg1, temp_cal_packet, 6);
    read_emg(&emg2, temp_cal_packet, 7);
    read_emg(&emg3, temp_cal_packet, 8);
    read_emg(&emg4, temp_cal_packet, 9);

    simple_lowpass(lp_calibrated, temp_cal_packet);

    //////////// Using lp_calibrated

    dt_calibration = (millis() - start_dt) * 1/1000;
    start_dt = millis();

    complementary_filter(dt_calibration, lp_calibrated, &roll_temp, &pitch_temp);

    rest_depth += -lp_calibrated[0]*sin(roll_temp) + lp_calibrated[1]*cos(roll_temp)*sin(pitch_temp) + lp_calibrated[2]*cos(pitch_temp)*cos(roll_temp);

    sum_roll += roll_temp;
    sum_pitch += pitch_temp;
    emg1_at_rest += lp_calibrated[6];
    emg2_at_rest += lp_calibrated[7];
    emg3_at_rest += lp_calibrated[8];
    emg4_at_rest += lp_calibrated[9];
    numValues += 1;
  }

  // Get average
  roll_at_rest = sum_roll / numValues;
  pitch_at_rest = sum_pitch / numValues;
  emg1_at_rest = emg1_at_rest / numValues;
  emg2_at_rest = emg2_at_rest / numValues;
  emg3_at_rest = emg3_at_rest / numValues;
  emg4_at_rest = emg4_at_rest / numValues;
  rest_depth = rest_depth / numValues;

  //////////// Using calibrated_packet
  calibrated_packet[0] = emg1_at_rest;
  calibrated_packet[1] = emg2_at_rest;
  calibrated_packet[2] = emg3_at_rest;
  calibrated_packet[3] = emg4_at_rest;
  calibrated_packet[4] = 0;
  calibrated_packet[5] = 0;
  calibrated_packet[6] = 0;

  ////////////// PACKET STRUCTURE //////////////////////////
  //   EMG1    EMG2    EMG3    EMG4    0    0   0 ///////////

  memcpy(&calibrated_packet[DATA_SIZE], &CALIBRATION_REST_PACKET, sizeof(float));
  send_ble(calibrated_packet, PACKET_SIZE, true);

  Serial.println("Completed baseline calibration.");
}

void calibrate_moving() {
  Serial.println("Starting moving calibration...");
  reset(calibrated_packet, DATA_SIZE);
  reset(temp_cal_packet, DATA_SIZE);

  int numValues = 0;
  unsigned long calibration_start = millis();
  unsigned long start_dt = millis();
  float max_emg[4];
  reset(max_emg, 4);
  float current_depth = 0.0;
  float max_depth = INT_MIN;

  // 10s to do a squat
  while (millis() - calibration_start < 10000) {
    read_imu(&mpu, temp_cal_packet);
    read_emg(&emg1, temp_cal_packet, 6);
    read_emg(&emg2, temp_cal_packet, 7);
    read_emg(&emg3, temp_cal_packet, 8);
    read_emg(&emg4, temp_cal_packet, 9);

    if (temp_cal_packet[6] > max_emg[0]) {
      max_emg[0] = temp_cal_packet[6];
    }
    if (temp_cal_packet[7] > max_emg[1]) {
      max_emg[1] = temp_cal_packet[7];
    }
    if (temp_cal_packet[8] > max_emg[2]) {
      max_emg[2] = temp_cal_packet[8];
    }
    if (temp_cal_packet[9] > max_emg[3]) {
      max_emg[3] = temp_cal_packet[9];
    }

    simple_lowpass(lp_calibrated, temp_cal_packet);

    dt_calibration = (millis() - start_dt) * 1/1000;
    start_dt = millis();

    complementary_filter(dt_calibration, lp_calibrated, &roll_temp, &pitch_temp);

    current_depth += -lp_calibrated[0]*sin(roll_temp) + lp_calibrated[1]*cos(roll_temp)*sin(pitch_temp) + lp_calibrated[2]*cos(pitch_temp)*cos(roll_temp);
    if (abs(current_depth) > max_depth) {
      max_depth = abs(current_depth);
    }
  }

  // Finalize calibrated packet
  calibrated_packet[0] = max_depth;
  calibrated_packet[1] = max_emg[0];
  calibrated_packet[2] = max_emg[1];
  calibrated_packet[3] = max_emg[2];
  calibrated_packet[4] = max_emg[3];
  calibrated_packet[5] = 0;
  calibrated_packet[6] = 0;

  ////////////// PACKET STRUCTURE //////////////////////////
  // Depth   EMG1    EMG2    EMG3    EMG4    0    0/////////


  memcpy(&calibrated_packet[DATA_SIZE], &CALIBRATION_MOVING_PACKET, sizeof(float));
  send_ble(calibrated_packet, PACKET_SIZE, true);

  Serial.println("Completed moving calibration.");
}