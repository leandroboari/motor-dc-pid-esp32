#define ENCODER_PIN 13
#define POT_PIN 34
#define L_PWM_PIN 12
#define R_PWM_PIN 14
#define H_BRIDGE_FREQUENCY 5000
#define H_BRIDGE_RESOLUTION 12
#define PULSES_PER_REV 20.0
#define UPDATE_INTERVAL_US 10000
#define NUM_SAMPLES 100
#define SETPOINT_MAX_VALUE 280.0

volatile unsigned long pulseCount = 0;
unsigned long pulseCountPrev = 0;
unsigned long previousTime = 0;
float RPM = 0;
float speedBuffer[NUM_SAMPLES] = {};
int bufferIndex = 0;

// PID parameters
float Kp = 1;
float Ki = 3;
float Kd = 0.1;
float integral = 0.0;
float previousError = 0.0;
float penultimateError = 0.0;

void IRAM_ATTR handleEncoder() {
  pulseCount++;
}

void controlMotor(int value) {
  value = constrain(value, -4095, 4095);
  if (value > 0) {
    ledcWrite(L_PWM_PIN, value);
    ledcWrite(R_PWM_PIN, 0);
  } else {
    value = -value;
    ledcWrite(L_PWM_PIN, 0);
    ledcWrite(R_PWM_PIN, value);
  }
}

int getPotRead() {
  return analogRead(POT_PIN);
}

void setup() {
  Serial.begin(115200);

  // Setup potentiometer
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  // Setup encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoder, RISING);

  // Setup motor control
  ledcAttach(L_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
  ledcAttach(R_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);

  // Start motor stopped
  controlMotor(0);
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - previousTime >= UPDATE_INTERVAL_US) {

    // Get current pulse count
    noInterrupts();
    unsigned long currentPulseCount = pulseCount;
    interrupts();

    // Get pot analog read value to set RPM set point
    int potValue = getPotRead();
    float setpointRPM = map(potValue, 0, 4095, 0.0, SETPOINT_MAX_VALUE);

    // Calculate speed RPM
    unsigned long pulses = currentPulseCount - pulseCountPrev;
    float elapsedTime = (currentTime - previousTime) / 1e6;
    float currentRPM = (pulses * 60.0 / PULSES_PER_REV) / elapsedTime;

    // Filter
    speedBuffer[bufferIndex] = currentRPM;
    bufferIndex = (bufferIndex + 1) % NUM_SAMPLES;
    RPM = 0.0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      RPM += speedBuffer[i];
    }
    RPM /= NUM_SAMPLES;

    // PID control
    float error = setpointRPM - RPM;

    float P = Kp * error;
    integral += error * (UPDATE_INTERVAL_US / 1e6);
    float I = Ki * integral;

    float D = Kd * (error - 2 * previousError + penultimateError) / (UPDATE_INTERVAL_US / 1e6);
    float output = P + I + D;

    penultimateError = previousError;
    previousError = error;

    // Control motor with PID output
    int pwmValue = (int)output;
    controlMotor(pwmValue);

    Serial.print("Setpoint:");
    Serial.print(setpointRPM);
    Serial.print(",");
    Serial.print("Speed:");
    Serial.print(RPM);
    Serial.println();

    // Update previous variables
    previousTime = currentTime;
    pulseCountPrev = currentPulseCount;
  }
}
