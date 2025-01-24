#define NUM_SAMPLES 20

volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;
unsigned long pulseCountPrev = 0;

float speedBuffer[NUM_SAMPLES] = {0};
int bufferIndex = 0;
float motorSpeedRPM = 0;
float speedSetPoint = 0;
float motorTension = 0;

// Ajuste dos ganhos PID
float Kp = 2.0;
float Ki = 5.0;
float Kd = 0.1;

const unsigned long intervalMs = 50;

float previousError = 0;
float penultimateError = 0;

float integral = 0;

void setup() {
  setupEsp32();
  setupButton();
  setupPot();
  setupHBridge();
  setupEncoder();
  setupDisplay();
  delay(1000);
}

void loop() {
  // Leitura do potenciômetro e cálculo do setpoint
  float potValue = (float)getPotValue();
  motorTension = map(potValue, 0.0, 4095.0, 0.0, 5.0);
  speedSetPoint = map(potValue, 0.0, 4095.0, 0.0, 480.0);

  // Calcula a velocidade atual do motor
  calculateMotorSpeed();

  // Controle PID
  float error = speedSetPoint - motorSpeedRPM;

  float P = Kp * error;
  integral += error * (intervalMs / 1000.0);
  float I = Ki * integral;

  float D = Kd * (error - 2 * previousError + penultimateError) / (intervalMs / 1000.0);
  float output = P + I + D;

  penultimateError = previousError;
  previousError = error;

  // Controla o motor com base na saída do PID
  controlMotor(output);

  // Atualiza os valores do PID via Serial Monitor
  updatePIDValuesFromSerial();

  // Exibição no display
  String text = "";
  text += "Pot: ";
  text += (String)potValue;
  text += "\n";

  text += "Voltage: ";
  text += (String)motorTension;
  text += " V\n";

  text += "Setpoint: ";
  text += (String)speedSetPoint;
  text += " rpm\n";

  text += "Speed: ";
  text += String(motorSpeedRPM, 1);
  text += " rpm \n";

  writeDisplay(text);


  // Saída serial para debug
  Serial.print("Setpoint:");
  Serial.print(speedSetPoint);
  Serial.print(",RPM:");
  Serial.println(motorSpeedRPM);

  delay(50);
}
