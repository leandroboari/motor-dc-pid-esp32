#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int displaySdaPin = 19;
const int displaySclPin = 18;
const int displayVccPin = 17;
const int displayGndPin = 5;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setupDisplay() {
  pinMode(displayVccPin, OUTPUT);
  pinMode(displayGndPin, OUTPUT);
  digitalWrite(displayVccPin, HIGH);
  digitalWrite(displayGndPin, LOW);
  Wire.begin(displaySdaPin, displaySclPin);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void writeDisplay(String text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Potenciometro:"));
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println(text);
  display.display();
}
