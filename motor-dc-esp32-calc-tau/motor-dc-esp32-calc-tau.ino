#define NUM_SAMPLES 5

volatile unsigned long pulseCount = 0;
unsigned long pulseCountPrev = 0;
unsigned long previousTime = 0;

float speedBuffer[NUM_SAMPLES] = {};
int bufferIndex = 0;
float motorSpeedRPM = 0;
float speedFinal = 480; // Velocidade final em RPM
float speed63Percent = 480 * 0.633; // 63% da velocidade final
float tau = 0; // Constante de tempo calculada

const unsigned long intervalMs = 1000;

unsigned long startTime = 0;
unsigned long time63Percent = 0;
bool measuringTau = true;

void setup() {
  setupEsp32();
  setupEncoder();
  setupHBridge();
  delay(3000);
  controlMotor(4095);
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= intervalMs) {
    noInterrupts();
    unsigned long currentPulseCount = pulseCount;
    interrupts();

    unsigned long pulses = currentPulseCount - pulseCountPrev;
    motorSpeedRPM = (pulses * 60.0 / 20.0) / (intervalMs / 1000.0);

    previousTime = currentTime;
    pulseCountPrev = currentPulseCount;

    Serial.print(millis()/1000);
    Serial.print(",");
    Serial.print(motorSpeedRPM);
    Serial.println();

  }
}
