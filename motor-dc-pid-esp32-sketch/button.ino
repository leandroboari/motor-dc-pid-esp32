#define BUTTON_PIN 25
#define BUTTON_GND_PIN 32

void setupButton() {
  pinMode(BUTTON_GND_PIN, OUTPUT);
  digitalWrite(BUTTON_GND_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

bool getButtonState() {
  int buttonState = digitalRead(BUTTON_PIN);
  return buttonState == LOW ? true : false;
}
