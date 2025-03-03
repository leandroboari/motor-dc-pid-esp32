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
#define NUM_SAMPLES 20
#define LOOP_DELAY_MS 5 // 50
#define SETPOINT_MAX_VALUE 380

//#define FACTOR 15.3846154
#define FACTOR 15.3846154
#define Kp 2
#define Ki 4
#define Kd 0.01

Adafruit_SSD1306 display(128, 64, &Wire, -1);

volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;
unsigned long pulseCountPrev = 0;
float speedBuffer[NUM_SAMPLES] = {0};
int bufferIndex = 0;
int motorSpeed = 0;
int speedSetPoint = 0;
const unsigned long intervalMs = 50;
float previousError = 0;
float penultimateError = 0;
float integral = 0;

int output = 0;

void IRAM_ATTR handleEncoderInterrupt() {
  pulseCount++;
}
int getPotValue() {
  return analogRead(POT_PIN);
}

void controlMotor(int value) {
  value = constrain(value, -4095, 4095); // Limitar o valor para o range permitido
  if (value > 0) {
    ledcWrite(L_PWM_PIN, value);
    ledcWrite(R_PWM_PIN, 0);
  } else {
    value = -value;
    ledcWrite(L_PWM_PIN, 0);
    ledcWrite(R_PWM_PIN, value);
  }
}

void calculateMotorSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= intervalMs) {
    noInterrupts();
    unsigned long currentPulseCount = pulseCount;
    interrupts();

    unsigned long pulses = currentPulseCount - pulseCountPrev;
    float currentSpeedRPM = (pulses * 60.0 / 20.0) / (intervalMs / 1000.0);

    // Filtro de média móvel
    speedBuffer[bufferIndex] = currentSpeedRPM;
    bufferIndex = (bufferIndex + 1) % NUM_SAMPLES;

    motorSpeed = 0.0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      motorSpeed += speedBuffer[i];
    }
    motorSpeed /= NUM_SAMPLES;

    // Atualizar variáveis
    previousTime = currentTime;
    pulseCountPrev = currentPulseCount;
  }
}

bool getButtonState() {
  return digitalRead(BUTTON_PIN) == LOW ? true : false;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Button
  pinMode(BUTTON_GND_PIN, OUTPUT);
  digitalWrite(BUTTON_GND_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Potentiometer
  pinMode(POT_PIN, INPUT);
  analogSetPinAttenuation(POT_PIN, ADC_0db);

  // H Bridge
  ledcAttach(L_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);
  ledcAttach(R_PWM_PIN, H_BRIDGE_FREQUENCY, H_BRIDGE_RESOLUTION);

  // Encoder
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoderInterrupt, RISING);

  // Display
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
  // Leitura do potenciômetro e cálculo do setpoint
  int potValue = getPotValue();
  speedSetPoint = map(potValue, 0, 4095, 0, SETPOINT_MAX_VALUE);

  // Calcula a velocidade atual do motor
  calculateMotorSpeed();

//  if(motorSpeed > 295) {
//    controlMotor(output);
//    Serial.print(float(millis() / 1000.0f));
//    Serial.print(",");
//    Serial.print(speedSetPoint);
//    Serial.print(",");
//    Serial.print(motorSpeed);
//    Serial.print(",0,500");
//    Serial.println();
//    delay(LOOP_DELAY_MS * 1.8);
//    return;
//  }

  // Controle PID
  float error = speedSetPoint - motorSpeed;
  float P = Kp * error;
  integral += error * (intervalMs / 1000.0);
  float I = Ki * integral;
  float D = Kd * (error - 2 * previousError + penultimateError) / (intervalMs / 1000.0);
  output = P + I + D;
  penultimateError = previousError;
  previousError = error;

  // Controla o motor
  controlMotor(output);

  // Atualiza display
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(5, 0);
  display.print("VELOCIDADE");

  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Setpoint:");

  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print((String)speedSetPoint);

  display.setTextSize(1);
  display.setCursor(80, 25);
  display.print("Sensor:");

  display.setTextSize(2);
  display.setCursor(80, 40);
  display.print((String)motorSpeed);

  display.display();

  // Envia por serial
  Serial.print(float(millis() / 1000.0f));
  Serial.print(",");
  Serial.print(speedSetPoint);
  Serial.print(",");
  Serial.print(motorSpeed);
  Serial.print(",0,500");
  Serial.println();

  delay(LOOP_DELAY_MS);
}
