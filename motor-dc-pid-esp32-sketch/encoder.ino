const int encoderPin = 4;

volatile unsigned long pulseCount = 0;

void IRAM_ATTR handleEncoderInterrupt() {
  pulseCount++;
}


void setupEncoder() {
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoderInterrupt, RISING);
}


void loopEncoder() {
  noInterrupts();
  unsigned long currentPulseCount = pulseCount;
  interrupts();
  float rotations = currentPulseCount / 20.0;

  Serial.print("Pulsos: ");
  Serial.print(currentPulseCount);
  Serial.print(" | Rotacoes: ");
  Serial.println(rotations);

  delay(500);
}
