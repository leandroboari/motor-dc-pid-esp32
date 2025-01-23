const int encoderPin = 13;

volatile unsigned long pulseCount = 0;

void IRAM_ATTR handleEncoderInterrupt() {
  noInterrupts();
  pulseCount++;
  interrupts();
}



void setupEncoder() {
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoderInterrupt, RISING);
}


int getEncoderCount() {
  noInterrupts();
  unsigned long currentPulseCount = pulseCount;
  interrupts();
  return currentPulseCount;
}
