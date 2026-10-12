/*
  ========================================================================
  SISTEM KONTROL MOTOR DC ENCODER 25JGA370 + L298N + ARDUINO NANO
  ------------------------------------------------------------------------
  Mode 0 : STOP
  Mode 1 : OPEN LOOP (PWM Direct Control: -255 s/d 255)
  Mode 2 : CLOSED LOOP RPM (PID Control Kecepatan)
  Mode 3 : CLOSED LOOP POSISI / SERVO (PID Control Posisi Derajat)
  
  PERINTAH SERIAL:
  - "MODE:x"         : Pindah ke Mode x (0-3)
  - "SET_PWM:val"    : Set PWM Mode 1 (-255 s/d 255)
  - "SET_RPM:val"    : Set Target RPM Mode 2
  - "SET_ANG:val"    : Set Target Sudut Derajat Mode 3
  - "ZERO"           : Pindah ke Mode 3 & memutar motor secara fisik ke 90°
  - "RESET_ORIGIN"   : Mereset titik acuan encoder saat ini menjadi 0°
  - "PID_RPM:p,i,d"  : Set Parameter PID RPM
  - "PID_POS:p,i,d"  : Set Parameter PID Posisi
  ========================================================================
*/

// --- PIN DEFINITION ---
const int PIN_ENA   = 9;  // PWM Speed Control (L298N)
const int PIN_IN1   = 7;  // Direction Pin 1
const int PIN_IN2   = 8;  // Direction Pin 2
const int PIN_ENC_A = 2;  // Encoder Phase A (Hardware Interrupt INT0)
const int PIN_ENC_B = 4;  // Encoder Phase B (Digital Input)

// --- KONSTANTA MOTOR & ENCODER ---
const float PPR = 104.0;  // Pulses Per Revolution hasil kalibrasi

// --- VARIABEL GLOBAL SENSOR ---
volatile long encoderTicks = 0;
long prevTicks = 0;

float currentRPM = 0.0;
float currentAngle = 0.0;

// --- VARIABEL OPERASIONAL & TARGET ---
int currentMode = 0;       // Default Mode 0 (STOP)
float targetPWM = 0.0;     // Target PWM (-255 s/d 255)
float targetRPM = 0.0;     // Target Kecepatan (RPM)
float targetAngle = 90.0;  // Default Target Angle diset ke 90 derajat

// --- VARIABEL & GAIN PID RPM (MODE 2) ---
float Kp_rpm = 1.2;
float Ki_rpm = 0.5;
float Kd_rpm = 0.01;
float integral_rpm = 0.0;
float error_rpm_prev = 0.0;

// --- VARIABEL & GAIN PID POSISI (MODE 3 - DITINGKATKAN TORSINYA) ---
float Kp_pos = 5.0;        // Dinaikkan dari 3.5 ke 5.0 agar lebih presisi menahan posisi
float Ki_pos = 0.2;        // Dinaikkan dari 0.15 ke 0.2 untuk mengikis error gesekan kecil
float Kd_pos = 0.1;
float integral_pos = 0.0;
float error_pos_prev = 0.0;

// --- TIMING / PEWAKTUAN ---
unsigned long prevTime = 0;
const float dt = 0.02;     // Loop kontrol 20ms (0.02 detik)

// --- FUNCTION DECLARATION ---
void encoderISR();
void readSerialCommands();
void computeTelemetry();
void runPIDRPM(float dt_val);
void runPIDPosition(float dt_val);
void setMotorOutput(float pwmVal);

void setup() {
  Serial.begin(115200);

  // Inisialisasi Pin Motor Driver
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  // Pastikan motor mati saat startup
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);

  // Inisialisasi Pin Encoder
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // Interrupt pada RISING edge Pin D2
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, RISING);

  prevTime = millis();
}

void loop() {
  // 1. Baca Perintah dari Serial Communication
  readSerialCommands();

  // 2. Eksekusi Kontrol Rutin Setiap 20 ms
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 20) {
    prevTime = currentTime;

    // Kalkulasi telemetri (RPM & Sudut)
    computeTelemetry();

    // Pilih Mode Kontrol Motor
    switch (currentMode) {
      case 0: // STOP
        setMotorOutput(0);
        break;

      case 1: // OPEN LOOP PWM
        setMotorOutput(targetPWM);
        break;

      case 2: // CLOSED LOOP RPM
        runPIDRPM(dt);
        break;

      case 3: // CLOSED LOOP POSISI (SERVO)
        runPIDPosition(dt);
        break;

      default:
        setMotorOutput(0);
        break;
    }

    // Format Telemetri: DATA:Mode,CurrentRPM,TargetRPM,CurrentAngle,TargetAngle,CurrentPWM
    Serial.print("DATA:");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.print(currentRPM);
    Serial.print(",");
    Serial.print(targetRPM);
    Serial.print(",");
    Serial.print(currentAngle);
    Serial.print(",");
    Serial.print(targetAngle);
    Serial.print(",");
    Serial.println(targetPWM);
  }
}

// ==========================================
// INTERRUPT SERVICE ROUTINE (ISR) ENCODER
// ==========================================
void encoderISR() {
  // Cek arah putaran berdasarkan Phase B
  if (digitalRead(PIN_ENC_B) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// ==========================================
// KALKULASI TELEMETRI (RPM & ANGULATION)
// ==========================================
void computeTelemetry() {
  long currentTicks;
  
  noInterrupts();
  currentTicks = encoderTicks;
  interrupts();

  long deltaTicks = currentTicks - prevTicks;
  prevTicks = currentTicks;

  // Hitung RPM: (Delta Ticks / PPR) * (60 detik / 0.02s)
  currentRPM = ((float)deltaTicks / PPR) * 3000.0;

  // Hitung Akumulasi Sudut Derajat
  currentAngle = ((float)currentTicks / PPR) * 360.0;
}

// ==========================================
// PID KONTROL RPM (MODE 2)
// ==========================================
void runPIDRPM(float dt_val) {
  float error = targetRPM - currentRPM;

  // Akumulasi integral langsung untuk menaikkan output PWM
  integral_rpm += (Ki_rpm * error * dt_val);
  
  // Anti-windup: batasi akumulasi integral sesuai rentang PWM driver (-255 s/d 255)
  integral_rpm = constrain(integral_rpm, -255.0, 255.0);

  float derivative = (error - error_rpm_prev) / dt_val;
  error_rpm_prev = error;

  // Hitung total PWM
  float outputPWM = (Kp_rpm * error) + integral_rpm + (Kd_rpm * derivative);
  setMotorOutput(outputPWM);
}

// ==========================================
// PID KONTROL POSISI / SERVO (MODE 3)
// ==========================================
void runPIDPosition(float dt_val) {
  float error = targetAngle - currentAngle;

  integral_pos += error * dt_val;
  integral_pos = constrain(integral_pos, -255.0, 255.0); // Prevent Windup

  float derivative = (error - error_pos_prev) / dt_val;
  error_pos_prev = error;

  float outputPWM = (Kp_pos * error) + (Ki_pos * integral_pos) + (Kd_pos * derivative);

  // Kompensasi Deadband Driver (mencegah motor tertahan gesekan mendekati setpoint)
  if (abs(error) > 0.5) {
    if (outputPWM > 0 && outputPWM < 35) outputPWM = 35;
    if (outputPWM < 0 && outputPWM > -35) outputPWM = -35;
  } else {
    outputPWM = 0; // Kunci tepat di posisi jika error < 0.5 derajat
  }

  setMotorOutput(outputPWM);
}

// ==========================================
// EKSEKUSI DRIVER MOTOR L298N
// ==========================================
void setMotorOutput(float pwmVal) {
  targetPWM = constrain(pwmVal, -255.0, 255.0);

  if (targetPWM > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, (int)targetPWM);
  } else if (targetPWM < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, (int)abs(targetPWM));
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  }
}

// ==========================================
// BACA PERINTAH SERIAL COMMUNICATION
// ==========================================
void readSerialCommands() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr.startsWith("MODE:")) {
      int newMode = inputStr.substring(5).toInt();
      if (newMode >= 0 && newMode <= 3) {
        currentMode = newMode;
        // Reset akumulator PID saat berganti mode
        integral_rpm = 0;
        integral_pos = 0;
        error_rpm_prev = 0;
        error_pos_prev = 0;
      }
    } 
    else if (inputStr.startsWith("SET_PWM:")) {
      targetPWM = inputStr.substring(8).toFloat();
      targetPWM = constrain(targetPWM, -255.0, 255.0);
    } 
    else if (inputStr.startsWith("SET_RPM:")) {
      targetRPM = inputStr.substring(8).toFloat();
    } 
    else if (inputStr.startsWith("SET_ANG:")) {
      targetAngle = inputStr.substring(8).toFloat();
    } 
    else if (inputStr.equals("ZERO")) {
      // 1. Otomatis masuk ke Mode 3 (Closed Loop Posisi)
      currentMode = 3;
      
      // 2. Perintahkan motor memutar fisik ke 90 derajat
      targetAngle = 90.0; 
      
      // 3. Reset akumulator PID agar pergerakan mulus
      integral_pos = 0;
      error_pos_prev = 0;
    }
    else if (inputStr.equals("RESET_ORIGIN")) {
      // Menjadikan posisi fisik motor saat ini sebagai titik acuan 0 derajat
      noInterrupts();
      encoderTicks = 0;
      interrupts();
      currentAngle = 0.0;
      targetAngle = 0.0;
    }
    else if (inputStr.startsWith("PID_RPM:")) {
      int firstComma = inputStr.indexOf(',');
      int secondComma = inputStr.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        Kp_rpm = inputStr.substring(8, firstComma).toFloat();
        Ki_rpm = inputStr.substring(firstComma + 1, secondComma).toFloat();
        Kd_rpm = inputStr.substring(secondComma + 1).toFloat();
      }
    }
    else if (inputStr.startsWith("PID_POS:")) {
      int firstComma = inputStr.indexOf(',');
      int secondComma = inputStr.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        Kp_pos = inputStr.substring(8, firstComma).toFloat();
        Ki_pos = inputStr.substring(firstComma + 1, secondComma).toFloat();
        Kd_pos = inputStr.substring(secondComma + 1).toFloat();
      }
    }
  }
}