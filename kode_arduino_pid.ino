#include <PID_v1.h>

// PIN
#define LIMIT_KELUAR 2
#define LIMIT_MASUK  4
#define RPWM 10
#define LPWM 12
#define REN 6
#define LEN 8
#define IR1 7
#define IR2 9
#define IR3 11
#define IR4 19
#define IR5 21
#define TRIG_PIN 3
#define ECHO_PIN 5

// Variabel
char trayState;
bool commandActive = false;
bool trayOut = false;
bool trayIn = false;
char tray_status = '0';
bool detect = false;
bool manualMasuk = false;

// Delay
unsigned long delayStart = 0;
bool delaySelesai = false;
bool delayMasukStart = false;
unsigned long delayStartMasuk = 0;
bool delayMasukSelesai = false;

long duration;
float distance_cm;

// PID setup
double setpointKeluar = 27.5;
double setpointMasuk = 8.5;
double input = 0;
double output = 0;
double outputMasuk = 0;

// PID kontrol
double Kp = 3.6;
double Ki = 0.5;
double Kd = 3.0;

PID pidKeluar(&input, &output, &setpointKeluar, Kp, Ki, Kd, DIRECT);
PID pidMasuk(&input, &outputMasuk, &setpointMasuk, Kp, Ki, Kd, REVERSE);

// Soft-start PWM
static int lastPWM = 0;

// Monitoring
unsigned long startMoveTime = 0;
unsigned long moveDuration = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(LIMIT_KELUAR, INPUT_PULLUP);
  pinMode(LIMIT_MASUK, INPUT_PULLUP);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  pidKeluar.SetMode(AUTOMATIC);
  pidMasuk.SetMode(AUTOMATIC);
  pidKeluar.SetOutputLimits(0, 150);
  pidMasuk.SetOutputLimits(0, 150);

  stopMotor();
  Serial.println("=== Sistem Gabungan Tray + PID Siap ===");
}

void loop() {
  distance_cm = bacaJarak();

  // Perintah manual via Serial
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

  // Perintah dari ESP32 
  if (Serial2.available() > 0) {
    trayState = Serial2.read();
    Serial.print("Dapat perintah dari ESP32: "); Serial.println(trayState);
    commandActive = true;
    delayStart = millis();
    delaySelesai = false;
  }

  // TRAY KELUAR
  if (commandActive && trayState == '1') {
    if (!delaySelesai) {
      if (millis() - delayStart >= 1500) {
        delaySelesai = true;
        Serial.println("Delay 1,5 detik selesai → PID mulai tray keluar");
      }
    } else {
      if (fabs(distance_cm - setpointKeluar) < 1.0 || digitalRead(LIMIT_KELUAR) == LOW) {
        stopMotor();
        trayOut = true;
        trayState = '2';
        Serial.println("Tray keluar selesai → lanjut cek IR & tray masuk");
      } else {
        input = distance_cm;
        pidKeluar.Compute();
        int pwmOut = kontrolMotor(output, distance_cm, setpointKeluar);
        motorKeluarPWM(pwmOut);
      }
    }
  }

  // TRAY MASUK (IR)
  else if (commandActive && trayState == '2') {
    if ((semuaIRTidakMendeteksi() || manualMasuk) && !delayMasukSelesai) {
      if (!delayMasukStart) {
        delayMasukStart = true;
        delayStartMasuk = millis();
        Serial.println("IR clear → Delay 2 detik sebelum tray masuk");
      } else if (millis() - delayStartMasuk >= 2000) {
        delayMasukSelesai = true;
        Serial.println("Delay 2 detik selesai → PID tray masuk mulai");
      }
    }

    if ((semuaIRTidakMendeteksi() || manualMasuk) && delayMasukSelesai) {
      if (fabs(distance_cm - setpointMasuk) < 0.5 || digitalRead(LIMIT_MASUK) == LOW) {
        stopMotor();
        trayIn = true;
        detect = !manualMasuk;
        kirimKeESP();
        resetStatus();
        Serial.println("Tray masuk selesai → kirim status ke ESP32");
      } else {
        input = distance_cm;
        pidMasuk.Compute();
        int pwmOut = kontrolMotor(outputMasuk, distance_cm, setpointMasuk);
        motorMasukPWM(pwmOut);
      }
    } else if (!manualMasuk && !semuaIRTidakMendeteksi()) {
      detect = true;
      Serial.println("IR mendeteksi objek → tunggu...");
    }
  }

  delay(100);
}

// Fungsi Tambahan
bool semuaIRTidakMendeteksi() {
  return (digitalRead(IR1) == HIGH && digitalRead(IR2) == HIGH &&
          digitalRead(IR3) == HIGH && digitalRead(IR4) == HIGH &&
          digitalRead(IR5) == HIGH);
}

int kontrolMotor(double outVal, double jarak, double setp) {
  static double filtPWM = 0;
  filtPWM = 0.85 * filtPWM + 0.15 * outVal;
  int pwmOut = (int)filtPWM;
  if (fabs(jarak - setp) < 2.0 && pwmOut < 55) pwmOut = 55;
  if (pwmOut < 40 && pwmOut > 0) pwmOut = 40;
  if (pwmOut > lastPWM + 5) pwmOut = lastPWM + 5;
  lastPWM = pwmOut;
  return pwmOut;
}

float bacaJarak() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2;
}

void motorKeluarPWM(int pwm) {
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, pwm);
  analogWrite(LPWM, 0);
}

void motorMasukPWM(int pwm) {
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, pwm);
}

void kirimKeESP() { tray_status = '1'; Serial2.print(tray_status); }

void resetStatus() {
  trayOut = false; trayIn = false; commandActive = false;
  trayState = '0'; manualMasuk = false;
  delaySelesai = false; delayMasukStart = false; delayMasukSelesai = false;
}

void stopMotor() {
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
