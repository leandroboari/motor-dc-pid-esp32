const int potentiometerPin = 34;

void setupPotentiometer() {
  pinMode(potentiometerPin, INPUT);
  analogSetPinAttenuation(potentiometerPin, ADC_0db);
}

float getVoltageValue() {
  int analogValue = analogRead(potentiometerPin);
  float voltageValue = analogValue * (3.3 / 4095.0);
  Serial.print(F("[GET VOLTAGE VALUE] Valor analógico: "));
  Serial.print(analogValue);
  Serial.print(F(" - Valor em tensão: "));
  Serial.print(voltageValue);
  Serial.print(F(" V \n"));
  return voltageValue;
}
