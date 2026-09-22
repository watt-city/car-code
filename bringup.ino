// bringup.ino — staged hardware check before PID
// Target: ESP32-S3, Arduino-ESP32 core 3.x (ledcAttach/ledcWrite API).
//   On core 2.x, replace ledcAttach(pin,f,b) with ledcSetup(ch,f,b)+ledcAttachPin(pin,ch)
//   and write to the channel instead of the pin.
// Tools: USB CDC On Boot = Enabled. Libraries: QTRSensors (Pololu).
// Serial monitor @115200, send one letter per command (see printMenu()).
// Put the chassis on a stand (wheels off the table) before motor tests.

#include <Wire.h>
#include <QTRSensors.h>

// ---------- Pins, from Log Book 8/17 and 8/24 ----------
#define PIN_SDA   8   // Log conflict: 8/14 says SDA=8/SCL=9, 8/17 says SDA=9/SCL=8.
#define PIN_SCL   9   // If the I2C scan finds nothing, swap these two.
#define PIN_XSHUT 12  // VL53L0X: must be HIGH or the sensor stays off

#define PWMA 15
#define AIN1 16
#define AIN2 17
#define PWMB 18
#define BIN1 39
#define BIN2 40

#define ENC_L_A 13
#define ENC_L_B 14
#define ENC_R_A 21
#define ENC_R_B 38

const uint8_t QTR_PINS[8] = {1, 2, 3, 4, 5, 6, 7, 10};  // OUT1..OUT15 (odd)
#define QTR_CTRL 11

const uint32_t PWM_FREQ = 20000;  // 20 kHz: above hearing, within TB6612 limit
const uint8_t  PWM_BITS = 8;      // duty 0..255
const int      TEST_SPEED = 120;  // low duty for first spin

QTRSensors qtr;
uint16_t qtrVals[8];

// ---------- Quadrature decoding (every edge on A and B) ----------
volatile long encL = 0, encR = 0;
volatile uint8_t stL = 0, stR = 0;
// index = (prev<<2)|curr, states AB: 00->01->11->10 counts +1
const int8_t QDEC[16] = {0,+1,-1,0, -1,0,0,+1, +1,0,0,-1, 0,-1,+1,0};

void IRAM_ATTR isrL() {
  uint8_t s = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
  encL += QDEC[(stL << 2) | s]; stL = s;
}
void IRAM_ATTR isrR() {
  uint8_t s = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
  encR += QDEC[(stR << 2) | s]; stR = s;
}

long getL() { noInterrupts(); long v = encL; interrupts(); return v; }
long getR() { noInterrupts(); long v = encR; interrupts(); return v; }

// speed: -255..255. 0 = coast (IN1=IN2=LOW).
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  ledcWrite(pwmPin, abs(speed));
}
void stopAll() { setMotor(PWMA, AIN1, AIN2, 0); setMotor(PWMB, BIN1, BIN2, 0); }

bool keyPressed() { if (Serial.available()) { while (Serial.available()) Serial.read(); return true; } return false; }

// ---------- Tests ----------
void i2cScan() {
  Serial.println("I2C scan (expect 0x6A = LSM6DS3TR-C, 0x29 = VL53L0X default):");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("  found 0x%02X\n", a); n++; }
  }
  if (!n) Serial.println("  nothing found -> check SDA/SCL swap, 3V3, GND, XSHUT");
}

void qtrRaw() {
  Serial.println("QTR raw RC times in us (timeout 2500). Low = white, high = black. Any key stops.");
  Serial.println("Slide a strip of black tape under ONE sensor at a time, left to right.");
  while (!keyPressed()) {
    qtr.read(qtrVals);
    for (int i = 0; i < 8; i++) Serial.printf("%5u ", qtrVals[i]);
    Serial.println();
    delay(100);
  }
}

void qtrCalLine() {
  Serial.println("Calibrating 5 s: sweep the array across the line repeatedly...");
  qtr.resetCalibration();
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) qtr.calibrate();
  Serial.print("min: "); for (int i = 0; i < 8; i++) Serial.printf("%5u ", qtr.calibrationOn.minimum[i]); Serial.println();
  Serial.print("max: "); for (int i = 0; i < 8; i++) Serial.printf("%5u ", qtr.calibrationOn.maximum[i]); Serial.println();
  Serial.println("Position 0 (sensor 1) .. 7000 (sensor 8). Any key stops.");
  while (!keyPressed()) {
    uint16_t pos = qtr.readLineBlack(qtrVals);
    for (int i = 0; i < 8; i++) Serial.printf("%4u ", qtrVals[i]);
    Serial.printf(" | pos=%u\n", pos);
    delay(100);
  }
}

void motorTest(const char *name, int pwm, int in1, int in2) {
  long l0 = getL(), r0 = getR();
  Serial.printf("%s motor FORWARD %d for 1 s...\n", name, TEST_SPEED);
  setMotor(pwm, in1, in2, TEST_SPEED); delay(1000); setMotor(pwm, in1, in2, 0); delay(300);
  long l1 = getL(), r1 = getR();
  Serial.printf("  dL=%ld dR=%ld\n", l1 - l0, r1 - r0);
  Serial.printf("%s motor REVERSE %d for 1 s...\n", name, TEST_SPEED);
  setMotor(pwm, in1, in2, -TEST_SPEED); delay(1000); setMotor(pwm, in1, in2, 0); delay(300);
  Serial.printf("  dL=%ld dR=%ld\n", getL() - l1, getR() - r1);
  Serial.println("PASS if: the intended wheel spun, FORWARD moved it robot-forward,");
  Serial.println("its own encoder went + then -, and the other encoder stayed ~0.");
}

void encStream() {
  Serial.println("Hand-spin wheels. Counts stream below. Any key stops.");
  while (!keyPressed()) { Serial.printf("L=%8ld  R=%8ld\n", getL(), getR()); delay(100); }
}

void printMenu() {
  Serial.println("\n--- bring-up ---");
  Serial.println(" i  I2C scan");
  Serial.println(" q  QTR raw readings");
  Serial.println(" c  QTR calibrate + line position");
  Serial.println(" l  left motor test (A side)");
  Serial.println(" r  right motor test (B side)");
  Serial.println(" e  stream encoder counts (hand spin)");
  Serial.println(" z  zero encoders");
  Serial.println(" s  stop motors");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_XSHUT, OUTPUT); digitalWrite(PIN_XSHUT, HIGH);
  delay(10);
  Wire.begin(PIN_SDA, PIN_SCL);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
  stopAll();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  stL = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
  stR = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
  attachInterrupt(ENC_L_A, isrL, CHANGE); attachInterrupt(ENC_L_B, isrL, CHANGE);
  attachInterrupt(ENC_R_A, isrR, CHANGE); attachInterrupt(ENC_R_B, isrR, CHANGE);

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(QTR_CTRL);

  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'i': i2cScan(); break;
    case 'q': qtrRaw(); break;
    case 'c': qtrCalLine(); break;
    case 'l': motorTest("LEFT", PWMA, AIN1, AIN2); break;
    case 'r': motorTest("RIGHT", PWMB, BIN1, BIN2); break;
    case 'e': encStream(); break;
    case 'z': noInterrupts(); encL = encR = 0; interrupts(); Serial.println("zeroed"); break;
    case 's': stopAll(); Serial.println("stopped"); break;
    default: if (c > ' ') printMenu();
  }
}
