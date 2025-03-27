float dt_calibration = 0.0;
float lp_calibrated[DATA_SIZE];

// Sums
float sum_roll = 0.0;
float sum_pitch = 0.0;

// Max
float max_emg_1 = -1000;
float max_emg_2 = -1000;
float max_emg_3 = -1000;
float max_emg_4 = -1000;

dataRead_t calibrateRead;

void calibrate_rest() {
  // Initialize rest array

  dataRead_t calibrateRead;

  Serial.println("Starting baseline calibration...");
  reset(calibrated_packet, DATA_SIZE);
  reset(temp_cal_packet, DATA_SIZE);

  int numValues = 0;
  unsigned long calibration_start = millis();
  unsigned long start_dt = millis();

  while (millis() - calibration_start < 5000) {
    // Read IMU values into packet

    //////////// Using temp_cal_packet
    read_imu(&mpu, &calibrateRead);

    // Read 4 EMG values into packet
    read_emg(&emg1, 1, &calibrateRead);
    read_emg(&emg2, 2, &calibrateRead);
    read_emg(&emg3, 3, &calibrateRead);
    read_emg(&emg4, 4, &calibrateRead);

    // simple_lowpass(lp_calibrated, temp_cal_packet);

    //////////// Using lp_calibrated

    dt_calibration = (millis() - start_dt) * 1/1000;
    start_dt = millis();

    complementary_filter(dt_calibration, &roll_temp, &pitch_temp, &calibrateRead);

    rest_acc += calibrateRead.acc_x - 9.8;
    
    // -lp_calibrated[0]*sin(pitch_temp) + lp_calibrated[1]*cos(pitch_temp)*sin(roll_temp) + lp_calibrated[2]*cos(pitch_temp)*cos(roll_temp);

    sum_roll += roll_temp;
    sum_pitch += pitch_temp;
    emg1_at_rest += calibrateRead.emg_1;
    emg2_at_rest += calibrateRead.emg_2;
    emg3_at_rest += calibrateRead.emg_3;
    emg4_at_rest += calibrateRead.emg_4;
    numValues += 1;
  }

  // Get average
  roll_at_rest = sum_roll / numValues;
  pitch_at_rest = sum_pitch / numValues;
  emg1_at_rest = emg1_at_rest / numValues;
  emg2_at_rest = emg2_at_rest / numValues;
  emg3_at_rest = emg3_at_rest / numValues;
  emg4_at_rest = emg4_at_rest / numValues;
  rest_acc = rest_acc / numValues;

  //////////// Using calibrated_packet
  calibrated_packet[0] = calibrateRead.emg_1;
  calibrated_packet[1] = calibrateRead.emg_2;
  calibrated_packet[2] = calibrateRead.emg_3;
  calibrated_packet[3] = calibrateRead.emg_4;
  calibrated_packet[4] = 0;
  calibrated_packet[5] = 0;
  calibrated_packet[6] = 0;

  ////////////// PACKET STRUCTURE //////////////////////////
  //   EMG1    EMG2    EMG3    EMG4    0    0   0 ///////////

  memcpy(&calibrated_packet[DATA_SIZE], &CALIBRATION_REST_PACKET, sizeof(float));
  send_ble(calibrated_packet, PACKET_SIZE, true);

  depth = 0.0;

  Serial.println("Completed baseline calibration.");
}

void calibrate_moving() {
  Serial.println("Starting moving calibration...");
  reset(calibrated_packet, DATA_SIZE);
  reset(temp_cal_packet, DATA_SIZE);

  int numValues = 0;
  unsigned long calibration_start = millis();
  unsigned long start_dt = millis();

  float current_depth = 0.0;
  float max_depth = INT_MIN;

  // 10s to do a squat
  while (millis() - calibration_start < 10000) {
    read_imu(&mpu, &calibrateRead);
    read_emg(&emg1, 1, &calibrateRead);
    read_emg(&emg2, 2, &calibrateRead);
    read_emg(&emg3, 3, &calibrateRead);
    read_emg(&emg4, 4, &calibrateRead);

    if (calibrateRead.emg_1 > max_emg_1) {
      max_emg_1 = calibrateRead.emg_1;
    }
    if (calibrateRead.emg_2 > max_emg_2) {
      max_emg_2 = calibrateRead.emg_2;
    }
    if (calibrateRead.emg_3 > max_emg_3) {
      max_emg_3 = calibrateRead.emg_3;
    }
    if (calibrateRead.emg_4 > max_emg_4) {
      max_emg_4 = calibrateRead.emg_4;
    }

    // simple_lowpass(lp_calibrated, temp_cal_packet);

    dt_calibration = (millis() - start_dt) * 1/1000;
    start_dt = millis();

    complementary_filter(dt_calibration, &roll_temp, &pitch_temp, &calibrateRead);

    // current_depth += -lp_calibrated[0]*sin(pitch_temp) + lp_calibrated[1]*cos(pitch_temp)*sin(roll_temp) + lp_calibrated[2]*cos(pitch_temp)*cos(roll_temp);
    if (calibrateRead.acc_x - 9.8 > max_acc_going_down) {
      max_acc_going_down = lp_calibrated[0] - 9.8;
    }

    if (calibrateRead.acc_x - 9.8 < min_acc_going_up) {
      min_acc_going_up = lp_calibrated[0] - 9.8;
    }

    // current_depth += lp_calibrated[0] - 9.88;
    // if (abs(current_depth) > max_depth) {
    //   max_depth = abs(current_depth);
    // }
  }

  // Finalize calibrated packet
  calibrated_packet[0] = max_depth;
  calibrated_packet[1] = max_emg_1;
  calibrated_packet[2] = max_emg_2;
  calibrated_packet[3] = max_emg_3;
  calibrated_packet[4] = max_emg_4;
  calibrated_packet[5] = max_acc_going_down;
  calibrated_packet[6] = min_acc_going_up;

  ////////////// PACKET STRUCTURE //////////////////////////
  // Depth   EMG1    EMG2    EMG3    EMG4    0    0/////////


  memcpy(&calibrated_packet[DATA_SIZE], &CALIBRATION_MOVING_PACKET, sizeof(float));
  send_ble(calibrated_packet, PACKET_SIZE, true);

  Serial.println("Completed moving calibration.");
}