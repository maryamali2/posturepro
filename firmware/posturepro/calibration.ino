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

float start_dt = 0.01;

void calibrate_rest() {
  // Initialize rest array
  roll_at_rest = 0.0;
  pitch_at_rest = 0.0;
  sum_roll = 0.0;
  sum_pitch = 0.0;

  dataRead_t calibrateRead;

  Serial.println("Starting baseline calibration...");

  depth = 0.0;

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
    unsigned long currentTime = millis();
    dt_calibration = (currentTime - start_dt) * 1/1000;
    start_dt = currentTime;

    // dt_calibration = (millis() - start_dt) / 1000.0;
    // start_dt = millis();

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

  depth = 0.0;

  int numValues = 0;
  unsigned long calibration_start = millis();

  float current_depth = 0.0;
  float max_depth = 1000;

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

    dt_calibration = (millis() - start_dt) / 1000.0;
    // float dt_z = dt_calibration * 1000;
    start_dt = millis();

    complementary_filter(dt_calibration, &roll_temp, &pitch_temp, &calibrateRead);

    // current_depth += -lp_calibrated[0]*sin(pitch_temp) + lp_calibrated[1]*cos(pitch_temp)*sin(roll_temp) + lp_calibrated[2]*cos(pitch_temp)*cos(roll_temp);
    float sin_phi = sinf(roll_estimate);
    float cos_phi = cosf(roll_estimate);
    float sin_theta = sinf(pitch_estimate);
    float cos_theta = cosf(pitch_estimate);

    float z_acc = ((-calibrateRead.acc_x * sin_theta) + ((calibrateRead.acc_y * sin_phi + calibrateRead.acc_z * cos_phi) * cos_theta)) - 9.7;

    z_acc = abs(z_acc) > .5 ? z_acc : 0.0;

    if (abs(calibrateRead.gyr_x) > 0.5 || abs(calibrateRead.gyr_y) > 0.5 || abs(calibrateRead.gyr_z) > 0.5) {
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

    Serial.print(" vel: ");
    Serial.print(vel);
    Serial.print(" depth: ");
    Serial.print(depth);
    Serial.println("");

    if (depth < max_depth) {
      max_depth = depth;
    }
  }

  // Finalize calibrated packet
  calibrated_packet[0] = abs(max_depth)/1000;
  calibrated_packet[1] = max_emg_1;
  calibrated_packet[2] = max_emg_2;
  calibrated_packet[3] = max_emg_3;
  calibrated_packet[4] = max_emg_4;
  calibrated_packet[5] = max_acc_going_down;
  calibrated_packet[6] = min_acc_going_up;

  ////////////// PACKET STRUCTURE //////////////////////////
  // Depth   EMG1    EMG2    EMG3    EMG4    0    0/////////


  depth = 0.0;

  memcpy(&calibrated_packet[DATA_SIZE], &CALIBRATION_MOVING_PACKET, sizeof(float));
  send_ble(calibrated_packet, PACKET_SIZE, true);
  Serial.println(abs(max_depth));
  Serial.println("Completed moving calibration.");
}