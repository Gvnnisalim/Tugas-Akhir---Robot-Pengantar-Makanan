#include <Arduino.h>

//PIN
#define RPWM 10
#define LPWM 12
#define REN 6
#define LEN 8
#define TRIG_PIN 6
#define ECHO_PIN 7

#define IR1 7
#define IR2 9
#define IR3 11
#define IR4 19
#define IR5 21

#define PWM_MIN 80
#define PWM_MAX 255

// PID
float setpoint = 27.5;
float Kp = 8.0;
float Ki = 0.8;
float Kd = 1.2;

float error = 0;
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

char trayState;
bool commandActive = false;
char tray_status = '0';
bool detect = false;
bool manualMasuk = false;

unsigned long delayStart = 0;
bool delaySelesai = false;
bool delayMasukStart = false;
unsigned long delayStartMasuk = 0;
bool delayMasukSelesai = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotor();
  Serial.println("Arduino Mega siap. Ketik 'm' untuk TUTUP TRAY manual.");
  lastTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("Perintah manual: "); Serial.println(cmd);
    if (cmd == 'm') {
      manualMasuk = true;
      trayState = '2';
      commandActive = true;
      delayMasukStart = true;
      delayStartMasuk = millis();
    }
  }

  if (Serial2.available() > 0) {
    trayState = Serial2.read();
    Serial.print("Dapat perintah dari ESP32: "); Serial.println(trayState);
    commandActive = true;
    delayStart = millis();
    delaySelesai = false;
  }

  if (commandActive && trayState == '1') {
    setpoint = 27.5; // setpoint keluar
    if (!delaySelesai) {
      if (millis() - delayStart >= 3000) {
        delaySelesai = true;
        Serial.println("Delay 3 detik selesai → Tray keluar (PID).");
      }
    } else {
      float jarak = getDistance();
      float output = hitungPID(jarak);
      setMotorPWM((int)output);

      Serial.print("Keluar | Jarak: "); Serial.print(jarak);
      Serial.print(" cm | Output: "); Serial.println(output);

      // Check mendekati setpoint
      if (abs(jarak - setpoint) < 1.0) {
        stopMotor();
        trayState = '2';
        integral = 0; // reset integral
        lastError = 0;
        Serial.println("Tray keluar selesai, lanjut ke tray masuk (PID).");
      }
    }
  }

  else if (commandActive && trayState == '2') {
    setpoint = 8.5; // setpoint masuk
    if ((semuaIRTidakMendeteksi() || manualMasuk) && !delayMasukSelesai) {
      if (!delayMasukStart) {
        delayMasukStart = true;
        delayStartMasuk = millis();
        Serial.println("IR clear → Delay 2 detik sebelum tray masuk (PID).");
      } else if (millis() - delayStartMasuk >= 2000) {
        delayMasukSelesai = true;
        Serial.println("Delay 2 detik selesai → Tray mulai masuk (PID).");
      }
    }

    if ((semuaIRTidakMendeteksi() || manualMasuk) && delayMasukSelesai) {
      float jarak = getDistance();
      float output = hitungPID(jarak);
      setMotorPWM((int)output);

      Serial.print("Masuk | Jarak: "); Serial.print(jarak);
      Serial.print(" cm | Output: "); Serial.println(output);

      if (abs(jarak - setpoint) < 1.0) {
        stopMotor();
        detect = !manualMasuk;
        kirimKeESP();
        resetStatus();
        Serial.println("Tray masuk selesai, kirim status ke ESP32.");
      }
    } else if (!manualMasuk && !semuaIRTidakMendeteksi()) {
      detect = true;
      Serial.println("IR mendeteksi benda, tunggu...");
    }
  }

  delay(100);
}

bool semuaIRTidakMendeteksi() {
  return (digitalRead(IR1) == HIGH && digitalRead(IR2) == HIGH &&
          digitalRead(IR3) == HIGH && digitalRead(IR4) == HIGH &&
          digitalRead(IR5) == HIGH);
}

void kirimKeESP() {
  tray_status = '1';
  Serial2.print(tray_status);
}

void resetStatus() {
  commandActive = false;
  trayState = '0';
  manualMasuk = false;
  delaySelesai = false;
  delayMasukStart = false;
  delayMasukSelesai = false;
  integral = 0;
  lastError = 0;
}

void stopMotor() {
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void setMotorPWM(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  if (pwm > 0) {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -pwm);
  } else {
    stopMotor();
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2.0;
  return distance;
}

float hitungPID(float jarakSekarang) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt < 0.01) dt = 0.01;

  error = setpoint - jarakSekarang;
  integral += error * dt;
  integral = constrain(integral, -100, 100);
  float derivative = (error - lastError) / dt;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  if (abs(error) < 0.2) {
    output = 0;
    integral = 0;
  }

  if (output > 0 && output < PWM_MIN) output = PWM_MIN;
  if (output < 0 && output > -PWM_MIN) output = -PWM_MIN;

  output = constrain(output, -PWM_MAX, PWM_MAX);

  lastError = error;
  lastTime = now;

  return output;
}
