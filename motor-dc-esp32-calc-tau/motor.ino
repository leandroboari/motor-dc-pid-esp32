#define L_PWM_PIN 12
#define R_PWM_PIN 14
#define H_BRIDGE_FREQUENCY 5000
#define H_BRIDGE_RESOLUTION 12

void setupHBridge() {
  ledcAttach(L_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
  ledcAttach(R_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
}

void controlMotor(int value) {
  value = constrain(value, -4095, 4095); // Limitar o valor para o range permitido
  if (value > 0) {
    ledcWrite(L_PWM_PIN, value);
    ledcWrite(R_PWM_PIN, 0);
  } else {
    value = -value;
    ledcWrite(L_PWM_PIN, 0);
    ledcWrite(R_PWM_PIN, value);
  }
}

void calculateMotorSpeed() {
 
}
