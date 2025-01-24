#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DISPLAY_SDA 19
#define DISPLAY_SCL 18
#define DISPLAY_VCC 17
#define DISPLAY_GND 5

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setupDisplay() {
  pinMode(DISPLAY_VCC, OUTPUT);
  pinMode(DISPLAY_GND, OUTPUT);
  digitalWrite(DISPLAY_VCC, HIGH);
  digitalWrite(DISPLAY_GND, LOW);
  Wire.begin(DISPLAY_SDA, DISPLAY_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

void writeDisplay(String text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(text);
  display.display();
}
