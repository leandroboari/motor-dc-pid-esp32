float Kp = 1.0; // Ganho Proporcional
float Ki = 0.1; // Ganho Integral
float Kd = 0.01; // Ganho Derivativo

float targetSpeed = 0; // Velocidade desejada (potenciômetro)
float currentSpeed = 0; // Velocidade atual (encoder)
float controlOutput = 0; // Saída do controlador PID
float integral = 0; // Acumulador do erro integral
float previousError = 0; // Erro da iteração anterior
unsigned long previousTime = 0; // Tempo da última execução do PID

const int pulsesPerRevolution = 20; // Altere para o número de pulsos por rotação do seu encoder
const float gearRatio = 1.0; // Relação de redução do motor, se houver
const float timeInterval = 0.05; // Intervalo de tempo entre leituras (50ms -> 0.05s)

const float factor = (60.0 / (pulsesPerRevolution * gearRatio * timeInterval));


void setup() {
  setupEsp32();
  setupButton();
  setupPotentiometer();
  setupHBridge();
  setupEncoder();
  setupDisplay();
}

void loop() {
  int potentiometerValue = getAnalogicValue();

  // Mapear o valor do potenciômetro para o range de velocidade do encoder
  targetSpeed = map(potentiometerValue, 0, 4095, 0, 100); // Velocidade de 0 a 100 RPM

  // Obter a velocidade atual do encoder (em RPM)
  currentSpeed = getEncoderCount() * factor; // Converte pulsos em RPM

  // Calcular o PID
  controlOutput = calculatePID(targetSpeed, currentSpeed);

  // Enviar a saída para o motor
  controlMotor((int)controlOutput);

  // Exibir no display
  writeDisplay("T: " + String(targetSpeed) + " C: " + String(currentSpeed));

  delay(50); // Ajuste conforme necessário
}
