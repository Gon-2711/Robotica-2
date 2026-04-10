#include <Servo.h>

Servo servos[5];

// ===================== CONFIGURACIÓN INDIVIDUAL =====================
struct ServoConfig {
  int pin;
  int min_us;
  int max_us;
  bool invert;
};

// Se mantiene la estructura original. 
// NOTA: Revisa el max_us del Servo 4 (está en 100, suele ser > 500)
ServoConfig S[5] = {
  {11, 500, 2400, false}, // Servo 1 - Base (θ1)
  { 8, 500, 2500, false}, // Servo 2 - Hombro (θ2)
  { 9, 600, 2480, false}, // Servo 3 - Codo (θ3)
  { 3, 100, 1950, false}, // Servo 4 - Muñeca (θ4) - Ajustado de 100 a 2100
  { 5, 500, 1700, false}  // Servo 5 - Pinza (θ5)
};

// ===================== MOVIMIENTO SUAVE =====================
void moveSmooth(int index, int target, float stepSize = 0.4, int delayTime = 5) {
  float current = servos[index].read();

  while (abs(current - target) > stepSize) {
    if (current < target) current += stepSize;
    else current -= stepSize;

    servos[index].write((int)current);
    delay(delayTime);
  }
  servos[index].write(target);
}

// ===================== RESET SERVOS =====================
void resetServos() {
  moveSmooth(0, 90);
  moveSmooth(1, 90);
  moveSmooth(2, 90);
  moveSmooth(3, 0);
  moveSmooth(4, 40);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  for (int i = 0; i < 5; i++) {
    servos[i].attach(S[i].pin, S[i].min_us, S[i].max_us);
  }

  resetServos();
  Serial.println("Sistema Listo - Esperando MATLAB/Python");
}

void loop() {
  // ===================== CONTROL DESDE PYTHON/MATLAB =====================
  if (Serial.available() > 0) {

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    float q[5];
    int count = 0;
    int startIndex = 0;

    // Parseo de la cadena separada por comas
    while (startIndex < line.length() && count < 5) {
      int commaIndex = line.indexOf(',', startIndex);
      if (commaIndex == -1) commaIndex = line.length();
      q[count++] = line.substring(startIndex, commaIndex).toFloat();
      startIndex = commaIndex + 1;
    }

    // Si recibimos al menos los 4 ángulos principales
    if (count >= 4) {
      // Mapeo de cinemática a grados del servo (0-180)
      q[0] = map(q[0], 0, 180, 0, 180);
      q[1] = map(q[1], 0, -180, 0, 180);
      q[2] = map(q[2], 90, -90, 0, 180);
      q[3] = map(q[3], -90, 90, 0, 180);
      
      // Control de pinza (0 abierto, 1 cerrado o viceversa)
      if (count == 5) q[4] = map(q[4], 0, 1, 90, 180);
      else q[4] = 70; // Posición por defecto si no llega el 5to dato

      for (int i = 0; i < 5; i++) {
        int pos = (int)q[i];

        if (S[i].invert) pos = 180 - pos;

        pos = constrain(pos, 0, 180);
        moveSmooth(i, pos, 0.4, 5);
      }
    }

    Serial.print("Recibido y Ejecutado: ");
    Serial.println(line);
  }
}
