

float calculatePID(float target, float current) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Tempo em segundos
    previousTime = currentTime;

    float error = target - current;
    integral += error * deltaTime; // Erro acumulado
    float derivative = (error - previousError) / deltaTime; // Mudança no erro
    previousError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    return constrain(output, -4095, 4095); // Limitar a saída
}
