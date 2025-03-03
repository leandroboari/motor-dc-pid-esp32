#define PPR 20
#define INTERVAL 50000
#define STARTUP_DELAY_MS 2000
#define L_PWM_PIN 12
#define R_PWM_PIN 14
#define H_BRIDGE_FREQUENCY 5000
#define H_BRIDGE_RESOLUTION 12
#define ENCODER_PIN 13
#define NUM_SAMPLES 10

volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;
unsigned long pulseCountPrev = 0;

float rpmSamples[NUM_SAMPLES] = {0};
int sampleIndex = 0;

float getFilteredRPM(float newRPM) {
  rpmSamples[sampleIndex] = newRPM;
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += rpmSamples[i];
  }
  return sum / NUM_SAMPLES;
}

void IRAM_ATTR handleEncoderInterrupt() {
  pulseCount++;
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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // H Bridge
  ledcAttach(L_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
  ledcAttach(R_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);

  // Encoder
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoderInterrupt, CHANGE);

  delay(STARTUP_DELAY_MS);

  controlMotor(4095);

}

void loop() {
  unsigned long currentTime = micros();

//  if (currentTime > 3000000) {
//    controlMotor(0);
//    return;
//  }

  noInterrupts();
  unsigned long currentPulseCount = pulseCount;
  interrupts();

  unsigned long pulses = currentPulseCount - pulseCountPrev;
  previousTime = currentTime;
  pulseCountPrev = currentPulseCount;

  float rpm = (pulses * 60.0f / PPR) / (INTERVAL / 1000000.0f);
  rpm = getFilteredRPM(rpm);
  
  Serial.print(float(currentTime / 1000000.0f));
  Serial.print(",");
  Serial.print(pulses);
  Serial.print(",");
  Serial.print(rpm);
  Serial.println();

  delayMicroseconds(INTERVAL);
}
