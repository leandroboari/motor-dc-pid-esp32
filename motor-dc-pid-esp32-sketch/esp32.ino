void setupEsp32() {
  Serial.begin(115200);
  Serial.print(F("\n\n[SETUP ESP32] Software inicializado com sucesso.\n\n"));
  analogReadResolution(12);
}
