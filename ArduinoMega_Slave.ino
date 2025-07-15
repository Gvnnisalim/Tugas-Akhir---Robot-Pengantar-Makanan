// === Arduino Mega Pro Mini ===

// === PIN DEFINISI ===
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

// === Variabel Global ===
char trayState;            
bool commandActive = false;
bool trayOut = false;      
bool trayIn = false;       
char tray_status = '0';    
bool detect = false;       
bool manualMasuk = false;  

// Variabel delay
unsigned long delayStart = 0;
bool delaySelesai = false;
bool delayMasukStart = false;
unsigned long delayStartMasuk = 0;
bool delayMasukSelesai = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600); // komunikasi dengan ESP32

  pinMode(LIMIT_KELUAR, INPUT_PULLUP);
  pinMode(LIMIT_MASUK, INPUT_PULLUP);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  stopMotor();
  Serial.println("Arduino Mega siap. Ketik 'm' untuk TUTUP TRAY manual.");
}

void loop() {
  // === Terima perintah manual dari Serial monitor ===
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("Perintah manual: "); Serial.println(cmd);
    if (cmd == 'm') {
      manualMasuk = true; 
      trayState = '2';
      commandActive = true;
      delayMasukStart = true;
      delayStartMasuk = millis(); // start delay 2 detik manual
    }
  }

  // === Terima perintah dari ESP32 ===
  if (Serial2.available() > 0) {
    trayState = Serial2.read();
    Serial.print("Dapat perintah dari ESP32: "); Serial.println(trayState);
    commandActive = true;
    delayStart = millis(); 
    delaySelesai = false;
  }

  // === Logika Tray Keluar dengan delay 3 detik ===
  if (commandActive && trayState == '1') {
    if (!delaySelesai) {
      if (millis() - delayStart >= 3000) {
        delaySelesai = true;
        Serial.println("Delay 3 detik selesai → Mulai tray keluar.");
      }
    } else {
      if (digitalRead(LIMIT_KELUAR) == HIGH) {
        motorKeluar();
      } else {
        stopMotor();
        trayOut = true;
        trayState = '2'; 
        Serial.println("Tray keluar selesai, lanjut ke tray masuk (cek IR).");
      }
    }
  }

  // === Logika IR & Tray Masuk ===
  else if (commandActive && trayState == '2') {
    if ((semuaIRTidakMendeteksi() || manualMasuk) && !delayMasukSelesai) {
      if (!delayMasukStart) {
        delayMasukStart = true;
        delayStartMasuk = millis(); // mulai delay 2 detik sebelum masuk
        Serial.println("IR clear → Delay 2 detik sebelum tray masuk.");
      } else if (millis() - delayStartMasuk >= 2000) {
        delayMasukSelesai = true;
        Serial.println("Delay 2 detik selesai → Tray mulai masuk.");
      }
    }

    if ((semuaIRTidakMendeteksi() || manualMasuk) && delayMasukSelesai) {
      if (digitalRead(LIMIT_MASUK) == LOW) {
        stopMotor();
        trayIn = true;
        detect = !manualMasuk; 
        kirimKeESP();
        resetStatus();
        Serial.println("Tray masuk selesai, kirim status ke ESP32.");
      } else {
        motorMasuk();
      }
    } else if (!manualMasuk && !semuaIRTidakMendeteksi()) {
      detect = true; 
      Serial.println("IR mendeteksi benda, tunggu...");
    }
  }

  delay(200); 
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
  trayOut = false;
  trayIn = false;
  commandActive = false;
  trayState = '0';
  manualMasuk = false;
  delaySelesai = false;
  delayMasukStart = false;
  delayMasukSelesai = false;
}

// === FUNGSI MOTOR ===
void motorKeluar() {
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 80);
  analogWrite(LPWM, 0);
}

void motorMasuk() {
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 80);
}

void stopMotor() {
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
