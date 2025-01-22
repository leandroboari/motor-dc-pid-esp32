void setup() {
  setupEsp32();
  setupButton();
  setupPotentiometer();
  setupHBridge();
  setupEncoder();
  
}

void loop() {
  if(getButtonState()) controlMotor(4000);
  else controlMotor(0);
  //  getVoltageValue();
  //  if (getButtonState()) {
  //    Serial.println("BOTÃO PRESSIONADO");
  //  }
  loopEncoder();
}
