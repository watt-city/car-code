/*
 * WATT City -- MAIN BOARD firmware (ESP32-S3)
 * Prototype target: ESP32-S3-DevKitC-1  (later: your WROOM-1-N8 PCB)
 *
 * Does:
 *   - QTR-MD-08RC line following (PID)
 *   - TB6612FNG dual motor drive (ESP32 LEDC PWM)
 *   - Quadrature encoders (100:1 gearmotor, 12 CPR) -> speed/odometry
 *   - VL53L0X forward ToF -> obstacle slow/stop
 *   - LSM6DS3TR-C IMU (the *breakout*; PCB uses LSM6DSM -> swap driver later)
 *   - UART link from XIAO vision node (AprilTag id/pose + camera obstacle)
 *   - ESP-NOW V2X receive (traffic-light state)
 *   - Serial test menu so you can bring up each subsystem alone
 *
 * Libraries (Arduino Library Manager):
 *   QTRSensors            (Pololu)
 *   Adafruit_VL53L0X
 *   Adafruit_LSM6DS       (provides Adafruit_LSM6DS3TRC)
 *   Board core: esp32 by Espressif, v3.x   (LEDC API below is v3.x)
 *
 * !!! PINS BELOW ARE PLACEHOLDERS -- replace with your WATT City GPIO map. !!!
 */

#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <esp_now.h>
#include <WiFi.h>
#include "watt_link.h"

// ======================= PIN MAP (EDIT ME) =======================
// TB6612FNG motor driver
static const int PIN_STBY = 4;
static const int PIN_AIN1 = 5, PIN_AIN2 = 6, PIN_PWMA = 7;   // left motor
static const int PIN_BIN1 = 15, PIN_BIN2 = 16, PIN_PWMB = 17; // right motor

// Quadrature encoders
static const int PIN_ENCL_A = 8,  PIN_ENCL_B = 9;
static const int PIN_ENCR_A = 10, PIN_ENCR_B = 11;

// QTR-MD-08RC: 8 sensor pins + emitter control (CTRL/LEDON)
static const uint8_t QTR_PINS[8] = {12, 13, 14, 21, 47, 48, 45, 35};
static const int PIN_QTR_EMITTER = 36;

// I2C (VL53L0X + IMU share the bus)
static const int PIN_SDA = 1, PIN_SCL = 2;

// UART to XIAO vision node (Serial1)
static const int PIN_VISION_RX = 44;  // main RX  <- XIAO TX
static const int PIN_VISION_TX = 43;  // main TX  -> XIAO RX (optional)
static const uint32_t VISION_BAUD = 115200;

// ======================= TUNABLES =======================
static const int    PWM_FREQ = 20000;   // 20 kHz, above audible
static const int    PWM_RES  = 8;       // 8-bit -> duty 0..255
static const int    BASE_SPEED_DEFAULT = 120;   // 0..255
static const int    MAX_SPEED = 220;

// PID on line position (QTR readLineBlack -> 0..7000, center 3500)
static float Kp = 0.08f, Ki = 0.0f, Kd = 1.2f;

// Obstacle thresholds (mm)
static const uint16_t TOF_STOP_MM = 90;
static const uint16_t TOF_SLOW_MM = 220;

// Encoder geometry: 12 CPR on motor shaft (x4 quadrature) * gearbox
static const float GEAR_RATIO   = 100.37f;   // Pololu 100:1 HPCB actual
static const float CPR_MOTOR    = 12.0f;      // counts/motor-rev at x4 decode
static const float COUNTS_PER_OUTPUT_REV = CPR_MOTOR * GEAR_RATIO;  // ~1204

// ======================= STATE =======================
enum RunMode { MODE_IDLE, MODE_CALIBRATE, MODE_RUN };
volatile RunMode mode = MODE_IDLE;

QTRSensors qtr;
uint16_t qtrValues[8];

Adafruit_VL53L0X lox;
bool tofOK = false;
uint16_t lastRange = 8000;

Adafruit_LSM6DS3TRC imu;
bool imuOK = false;

volatile long encCountL = 0, encCountR = 0;

VisionPacket vision = {0, -1, 0, 0, 0, 0};
uint32_t lastVisionMs = 0;

V2XPacket v2x = {0, LIGHT_UNKNOWN, 0};
uint32_t lastV2XMs = 0;

int baseSpeed = BASE_SPEED_DEFAULT;
float lastError = 0, integral = 0;

// ======================= ENCODER ISRs (x4 decode) =======================
void IRAM_ATTR isrL_A() { bool a=digitalRead(PIN_ENCL_A), b=digitalRead(PIN_ENCL_B); encCountL += (a==b)? 1 : -1; }
void IRAM_ATTR isrL_B() { bool a=digitalRead(PIN_ENCL_A), b=digitalRead(PIN_ENCL_B); encCountL += (a!=b)? 1 : -1; }
void IRAM_ATTR isrR_A() { bool a=digitalRead(PIN_ENCR_A), b=digitalRead(PIN_ENCR_B); encCountR += (a==b)? 1 : -1; }
void IRAM_ATTR isrR_B() { bool a=digitalRead(PIN_ENCR_A), b=digitalRead(PIN_ENCR_B); encCountR += (a!=b)? 1 : -1; }

// ======================= MOTORS (TB6612 + LEDC v3.x) =======================
void motorInit() {
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);   // esp32 core v3.x API
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
}
// speed: -255..255
void setMotor(int in1, int in2, int pwmPin, int speed) {
  bool fwd = speed >= 0;
  int duty = constrain(abs(speed), 0, 255);
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW  : HIGH);
  ledcWrite(pwmPin, duty);
}
void drive(int left, int right) {
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, left);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, right);
}
void stopMotors() { drive(0, 0); }

// ======================= SENSOR SETUP =======================
void qtrInit() {
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(PIN_QTR_EMITTER);
}
void calibrateQTR() {
  Serial.println(F("[QTR] Calibrating -- sweep the array across the black line for ~6 s..."));
  for (int i = 0; i < 300; i++) { qtr.calibrate(); delay(20); }
  Serial.println(F("[QTR] Calibration done."));
}
void tofInit() {
  tofOK = lox.begin();
  Serial.println(tofOK ? F("[ToF] VL53L0X OK") : F("[ToF] VL53L0X NOT found"));
}
uint16_t readToF() {
  if (!tofOK) return 8000;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000; // 4 = out of range
}
void imuInit() {
  imuOK = imu.begin_I2C();
  Serial.println(imuOK ? F("[IMU] LSM6DS3TR-C OK") : F("[IMU] IMU NOT found"));
}

// ======================= LINKS =======================
// --- UART framer: parse VisionPacket from XIAO ---
void pollVision() {
  static uint8_t st = 0, type = 0, len = 0, idx = 0, buf[64], sum = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    switch (st) {
      case 0: st = (b == WATT_SYNC0) ? 1 : 0; break;
      case 1: st = (b == WATT_SYNC1) ? 2 : 0; break;
      case 2: type = b; st = 3; break;
      case 3: len = b; idx = 0; st = (len <= sizeof(buf)) ? 4 : 0; break;
      case 4:
        buf[idx++] = b;
        if (idx >= len) st = 5;
        break;
      case 5:
        sum = watt_checksum(type, buf, len);
        if (b == sum && type == MSG_VISION && len == sizeof(VisionPacket)) {
          memcpy(&vision, buf, sizeof(VisionPacket));
          lastVisionMs = millis();
        }
        st = 0;
        break;
    }
  }
}

// --- ESP-NOW V2X receive ---
void onEspNow(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == sizeof(V2XPacket)) {
    memcpy(&v2x, data, sizeof(V2XPacket));
    lastV2XMs = millis();
  }
}
void v2xInit() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println(F("[V2X] ESP-NOW init FAILED")); return; }
  esp_now_register_recv_cb(onEspNow);
  Serial.print(F("[V2X] ESP-NOW ready. This board MAC: "));
  Serial.println(WiFi.macAddress());   // give this MAC to the roadside unit / other cars
}

// ======================= POLICY: how tags + lights + obstacles gate motion =======================
// Returns a speed scale 0.0..1.0 applied to baseSpeed. 0 = stop.
float motionGate() {
  // 1) hard stop on near ToF obstacle or fresh camera obstacle
  bool visionFresh = (millis() - lastVisionMs) < 300;
  if (lastRange <= TOF_STOP_MM) return 0.0f;
  if (visionFresh && vision.obstacle) return 0.0f;

  // 2) intersection logic: if the camera reports a tag that matches a V2X
  //    intersection id, obey the light.
  bool v2xFresh = (millis() - lastV2XMs) < 2000;
  if (visionFresh && vision.tagId >= 0 && v2xFresh &&
      vision.tagId == v2x.intersectionId) {
    if (v2x.lightState == LIGHT_RED)    return (vision.range_m < 0.35f) ? 0.0f : 0.35f;
    if (v2x.lightState == LIGHT_YELLOW) return 0.5f;
    // green falls through to normal speed
  }

  // 3) soft slow when something is moderately close
  if (lastRange <= TOF_SLOW_MM) return 0.45f;

  return 1.0f;
}

// ======================= LINE FOLLOW =======================
void lineFollowStep() {
  uint16_t pos = qtr.readLineBlack(qtrValues);   // 0..7000, center 3500
  float error = (float)pos - 3500.0f;

  integral = constrain(integral + error, -20000.0f, 20000.0f);
  float derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  float scale = motionGate();
  int spd = (int)(baseSpeed * scale);

  int left  = constrain(spd + (int)correction, -MAX_SPEED, MAX_SPEED);
  int right = constrain(spd - (int)correction, -MAX_SPEED, MAX_SPEED);
  drive(left, right);
}

// ======================= SERIAL TEST MENU =======================
void printMenu() {
  Serial.println(F("\n===== WATT City main-board test menu ====="));
  Serial.println(F(" m : motor sweep test (both motors fwd/rev)"));
  Serial.println(F(" q : QTR raw read (5 s)"));
  Serial.println(F(" t : ToF distance read (5 s)"));
  Serial.println(F(" i : IMU read (5 s)"));
  Serial.println(F(" e : encoder counts (5 s)"));
  Serial.println(F(" c : calibrate QTR"));
  Serial.println(F(" g : GO (autonomous line follow)"));
  Serial.println(F(" s : STOP"));
  Serial.println(F(" + / - : base speed up/down"));
  Serial.println(F("==========================================\n"));
}
void motorTest() {
  Serial.println(F("[motor] L fwd")); drive(120,0);   delay(700);
  Serial.println(F("[motor] L rev")); drive(-120,0);  delay(700);
  Serial.println(F("[motor] R fwd")); drive(0,120);   delay(700);
  Serial.println(F("[motor] R rev")); drive(0,-120);  delay(700);
  stopMotors(); Serial.println(F("[motor] done"));
}
void qtrRaw() {
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    qtr.read(qtrValues);
    for (int i=0;i<8;i++){ Serial.print(qtrValues[i]); Serial.print('\t'); }
    Serial.println(); delay(150);
  }
}
void tofRead() {
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) { Serial.print(F("range mm: ")); Serial.println(readToF()); delay(150); }
}
void imuRead() {
  if (!imuOK) { Serial.println(F("IMU not present")); return; }
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    sensors_event_t a, g, tmp; imu.getEvent(&a, &g, &tmp);
    Serial.printf("ax %.2f ay %.2f az %.2f | gx %.2f gy %.2f gz %.2f\n",
                  a.acceleration.x,a.acceleration.y,a.acceleration.z,
                  g.gyro.x,g.gyro.y,g.gyro.z);
    delay(150);
  }
}
void encRead() {
  long l0=encCountL, r0=encCountR; uint32_t t0=millis();
  while (millis() - t0 < 5000) {
    long dl=encCountL-l0, dr=encCountR-r0;
    Serial.printf("L %ld  R %ld   (rev L %.2f  R %.2f)\n",
                  encCountL, encCountR, dl/COUNTS_PER_OUTPUT_REV, dr/COUNTS_PER_OUTPUT_REV);
    delay(300);
  }
}
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'm': stopMotors(); motorTest(); break;
    case 'q': qtrRaw(); break;
    case 't': tofRead(); break;
    case 'i': imuRead(); break;
    case 'e': encRead(); break;
    case 'c': mode = MODE_IDLE; stopMotors(); calibrateQTR(); break;
    case 'g': integral=0; lastError=0; mode = MODE_RUN; Serial.println(F("[mode] RUN")); break;
    case 's': mode = MODE_IDLE; stopMotors(); Serial.println(F("[mode] IDLE")); break;
    case '+': baseSpeed = min(baseSpeed+10, MAX_SPEED); Serial.printf("base %d\n", baseSpeed); break;
    case '-': baseSpeed = max(baseSpeed-10, 0);         Serial.printf("base %d\n", baseSpeed); break;
    case '\n': case '\r': break;
    default: printMenu(); break;
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nWATT City main board booting..."));

  Wire.begin(PIN_SDA, PIN_SCL);
  Serial1.begin(VISION_BAUD, SERIAL_8N1, PIN_VISION_RX, PIN_VISION_TX);

  motorInit(); stopMotors();

  pinMode(PIN_ENCL_A, INPUT_PULLUP); pinMode(PIN_ENCL_B, INPUT_PULLUP);
  pinMode(PIN_ENCR_A, INPUT_PULLUP); pinMode(PIN_ENCR_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCL_A, isrL_A, CHANGE);
  attachInterrupt(PIN_ENCL_B, isrL_B, CHANGE);
  attachInterrupt(PIN_ENCR_A, isrR_A, CHANGE);
  attachInterrupt(PIN_ENCR_B, isrR_B, CHANGE);

  qtrInit();
  tofInit();
  imuInit();
  v2xInit();

  printMenu();
  Serial.println(F("Tip: run 'c' to calibrate QTR, then 'g' to go."));
}

void loop() {
  handleSerial();
  pollVision();
  lastRange = readToF();

  if (mode == MODE_RUN) lineFollowStep();
  else                  stopMotors();
}
