void complementary_filter(float dt, float* new_values, float* roll_estimate, float* pitch_estimate) {
  // COMPLEMENTARY FILTER
  float phi_acc = atan(new_values[1] / new_values[2]); // Roll
  float theta_acc = atan(new_values[0] / GRAVITY); // Pitch

  float phi_rate = new_values[3] + (sin(phi_acc))*(tan(theta_acc))*(new_values[4]) + (cos(phi_acc))*(tan(theta_acc))*(new_values[5]);
  float theta_rate = (cos(phi_acc))*(new_values[4]) - (sin(phi_acc))*(new_values[5]);

  *roll_estimate = (phi_acc * ALPHA) + (1 - ALPHA) * (*roll_estimate + dt * phi_rate);
  *pitch_estimate = (theta_acc * ALPHA) + (1 - ALPHA) * (*pitch_estimate + dt * theta_rate);
}