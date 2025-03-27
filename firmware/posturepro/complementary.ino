void complementary_filter(float dt, float* roll_estimate, float* pitch_estimate, dataRead_t* new_values) {
  // COMPLEMENTARY FILTER
  // float phi_acc = atan(new_values[1] / new_values[2]); // Roll
  // float theta_acc = atan(new_values[0] / GRAVITY); // Pitch


  // float phi_acc = atan2(new_values[1], new_values[2]); // Roll
  // float theta_acc = atan2(-new_values[0], sqrt(new_values[1]*new_values[1] + new_values[2]*new_values[2])); // Pitch

  float phi_acc = atan2(new_values->acc_y, new_values->acc_z); // Roll
  float theta_acc = atan2(-new_values->acc_x, sqrt(new_values->acc_y*new_values->acc_y + new_values->acc_z*new_values->acc_z)); // Pitch

  // float phi_rate = new_values[3] + (sin(phi_acc))*(tan(theta_acc))*(new_values[4]) + (cos(phi_acc))*(tan(theta_acc))*(new_values[5]);
  // float theta_rate = (cos(phi_acc))*(new_values[4]) - (sin(phi_acc))*(new_values[5]);

  float phi_rate = new_values->gyr_x + (sin(phi_acc))*(tan(theta_acc))*(new_values->gyr_y) + (cos(phi_acc))*(tan(theta_acc))*(new_values->gyr_z);
  float theta_rate = (cos(phi_acc))*(new_values->gyr_y) - (sin(phi_acc))*(new_values->gyr_z);

  *roll_estimate = (phi_acc * ALPHA) + (1 - ALPHA) * (*roll_estimate + dt * phi_rate);
  *pitch_estimate = (theta_acc * ALPHA) + (1 - ALPHA) * (*pitch_estimate + dt * theta_rate);
}