void updatePIDValuesFromSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Lê a string enviada pelo Serial Monitor
    input.trim(); // Remove espaços em branco

    // Divide a string em três valores separados por vírgula
    int commaIndex1 = input.indexOf(',');
    int commaIndex2 = input.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 > 0 && commaIndex2 > commaIndex1) {
      // Extrai os valores de Kp, Ki, e Kd
      float newKp = input.substring(0, commaIndex1).toFloat();
      float newKi = input.substring(commaIndex1 + 1, commaIndex2).toFloat();
      float newKd = input.substring(commaIndex2 + 1).toFloat();

      // Atualiza os ganhos do PID
      Kp = newKp;
      Ki = newKi;
      Kd = newKd;

      // Exibe os novos valores no Serial Monitor
      Serial.println("Novos valores de PID atualizados:");
      Serial.print("Kp: ");
      Serial.println(Kp);
      Serial.print("Ki: ");
      Serial.println(Ki);
      Serial.print("Kd: ");
      Serial.println(Kd);
    } else {
      Serial.println("Formato incorreto! Use: kp,ki,kd (exemplo: 2.5,4.0,0.1)");
    }
  }
}
