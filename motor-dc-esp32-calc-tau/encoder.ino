#define ENCODER_PIN 13

void IRAM_ATTR handleEncoderInterrupt() {
  pulseCount++;
}

void setupEncoder() {
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoderInterrupt, RISING);
}
