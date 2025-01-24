const int potPin = 34;

void setupPot() {
  pinMode(potPin, INPUT);
  analogSetPinAttenuation(potPin, ADC_0db);
}

int getPotValue() {
  return analogRead(potPin);
}
