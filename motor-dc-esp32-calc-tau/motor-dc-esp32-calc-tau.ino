#define NUM_SAMPLES 20

volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;
unsigned long pulseCountPrev = 0;

float speedBuffer[NUM_SAMPLES] = {0};
int bufferIndex = 0;
float motorSpeedRPM = 0;
float speedFinal = 0; // Velocidade final em RPM
float speed63Percent = 0; // 63% da velocidade final
float tau = 0; // Constante de tempo calculada

const unsigned long intervalMs = 10;

unsigned long startTime = 0;
unsigned long time63Percent = 0;
bool measuringTau = true;

void setup() {
  setupEsp32();
  setupEncoder();
  setupHBridge();
  
  controlMotor(4095);
  startTime = millis();
}

void loop() {
  calculateMotorSpeed();


  Serial.print("Velocidade Atual: ");
  Serial.print(motorSpeedRPM);
  Serial.print(" RPM");
  Serial.println();

  delay(intervalMs);
}
