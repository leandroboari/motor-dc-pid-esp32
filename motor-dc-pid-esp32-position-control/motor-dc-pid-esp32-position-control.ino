#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DISPLAY_SDA 19
#define DISPLAY_SCL 18
#define DISPLAY_VCC 17
#define DISPLAY_GND 5
#define L_PWM_PIN 12
#define R_PWM_PIN 14
#define H_BRIDGE_FREQUENCY 5000
#define H_BRIDGE_RESOLUTION 12
#define ENCODER_PIN 13
#define POT_PIN 34
#define BUTTON_PIN 25
#define BUTTON_GND_PIN 32
#define STARTUP_DELAY_MS 3000
#define LOOP_DELAY_MS 5
#define SETPOINT_MAX_VALUE 360  // 360 graus
#define PULSES_PER_REVOLUTION 20 // Pulsos por rotação do motor

#define FACTOR 15.3846154
#define Kp 2
#define Ki 4
#define Kd 0.01

#define CORRECTION 0.39

Adafruit_SSD1306 display(128, 64, &Wire, -1);

volatile long pulseCount = 0;
long positionSetPoint = 0;
float previousError = 0;
float integral = 0;
float output = 0;

void IRAM_ATTR handleEncoderInterrupt() {
  pulseCount++;
}

int getPotValue() {
  return analogRead(POT_PIN);
}

void controlMotor(int value) {
  value = constrain(value, -4095, 4095);
  if (value > 0) {
    ledcWrite(L_PWM_PIN, value);
    ledcWrite(R_PWM_PIN, 0);
  } else {
    value = -value;
    ledcWrite(L_PWM_PIN, 0);
    ledcWrite(R_PWM_PIN, value);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Configuração do botão
  pinMode(BUTTON_GND_PIN, OUTPUT);
  digitalWrite(BUTTON_GND_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configuração do potenciômetro
  pinMode(POT_PIN, INPUT);
  analogSetPinAttenuation(POT_PIN, ADC_0db);

  // Configuração do motor
  ledcAttach(L_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
  ledcAttach(R_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);

  // Configuração do encoder
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoderInterrupt, RISING);

  // Configuração do display
  pinMode(DISPLAY_VCC, OUTPUT);
  pinMode(DISPLAY_GND, OUTPUT);
  digitalWrite(DISPLAY_VCC, HIGH);
  digitalWrite(DISPLAY_GND, LOW);
  Wire.begin(DISPLAY_SDA, DISPLAY_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  delay(STARTUP_DELAY_MS);
}

void loop() {
  // Leitura do potenciômetro e conversão para setpoint de posição
  int potValue = getPotValue();
  positionSetPoint = map(potValue, 0, 4095, 0, SETPOINT_MAX_VALUE);

  // Conversão de pulsos do encoder para posição (graus)
  float currentPosition = (pulseCount * 360.0) / PULSES_PER_REVOLUTION;

  // Controle PID
  float error = positionSetPoint - currentPosition;
  float P = Kp * error;
  integral += error * (LOOP_DELAY_MS / 1000.0);
  float I = Ki * integral;
  float D = Kd * (error - previousError) / (LOOP_DELAY_MS / 1000.0);
  output = P + I + D;
  previousError = error;

  // Controle do motor
  controlMotor(output);

  // Atualiza o display
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(5, 0);
  display.print("POSICAO");

  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Setpoint:");

  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print((String)int(positionSetPoint));

  display.setTextSize(1);
  display.setCursor(80, 25);
  display.print("Atual:");

  display.setTextSize(2);
  display.setCursor(80, 40);
  display.print((String)int(currentPosition * CORRECTION));

  display.display();

  // Envia por serial
  Serial.print(float(millis() / 1000.0f));
  Serial.print(",");
  Serial.print(positionSetPoint);
  Serial.print(",");
  Serial.print(currentPosition * CORRECTION);
  Serial.print(",0,500");
  Serial.println();

  delay(LOOP_DELAY_MS);
}
