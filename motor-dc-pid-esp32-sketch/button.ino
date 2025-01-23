const int buttonPin = 25;
const int buttonPinGnd = 32;

void setupButton() {
  pinMode(buttonPinGnd, OUTPUT);
  digitalWrite(buttonPinGnd, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
}

bool getButtonState() {
  int buttonState = digitalRead(buttonPin);
  return buttonState == LOW ? true : false;
}
