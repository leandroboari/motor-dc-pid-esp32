const int buttonPin = 18;

void setupButton() {
  pinMode(buttonPin, INPUT_PULLUP);
}

bool getButtonState() {
  int buttonState = digitalRead(buttonPin);
  return buttonState == LOW ? true : false;
}
