// pid_line.ino — cascaded PID line follower
//   Inner loop: per-wheel speed PI + feedforward (encoders, counts/s)
//   Outer loop: line-position PD -> differential wheel speed targets
// Target: ESP32-S3, Arduino-ESP32 core 3.x. Library: QTRSensors (Pololu).
// Serial Monitor: 115200, line ending = "New Line". Type "help" for commands.
// Output lines are "label:value" pairs, so Tools > Serial Plotter graphs them.
//
// Starting gains below are rough guesses from the bring-up numbers
// (duty 120 -> ~1700 counts/s), NOT derived from a model. Tune them.

#include <QTRSensors.h>

// ---------- Pins (verified in bring-up) ----------
#define PWMA 15
#define AIN1 16
#define AIN2 17
#define PWMB 18
#define BIN1 39
#define BIN2 40

#define ENC_L_A 13
#define ENC_L_B 14
#define ENC_R_A 38   // swapped in bring-up so robot-forward counts +
#define ENC_R_B 21

const uint8_t QTR_PINS[8] = {1, 2, 3, 4, 5, 6, 7, 10};
#define QTR_CTRL 11

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_BITS = 8;
const uint32_t LOOP_US  = 10000;          // 100 Hz control loop
const float    DT       = LOOP_US / 1e6f;

// ---------- Tunables (change live over Serial) ----------
float kff  = 0.07f;  // feedforward: duty per count/s
float skp  = 0.02f;  // speed P: duty per count/s of error
float ski  = 0.10f;  // speed I: duty per (count/s * s)
float lkp  = 0.30f;  // line P: count/s of steering per position unit
float lkd  = 0.0f;   // line D: count/s per (position unit / s). Start at 0.
float base = 600;    // line-following base speed, counts/s
int   lineSign = 1;  // flip to -1 if the car steers away from the line

// Outer-sensor guard: sensors 0,1 and 6,7 should read light (line stays on 2..5).
// If either outer pair sees the line, slow down and steer harder until it's back.
float outerThr  = 400;   // calibrated value (0..1000) that counts as "line on this sensor"
float outerGain = 2.0f;  // line-loop gain multiplier while an outer sensor sees the line
float outerSlow = 0.4f;  // base speed fraction while an outer sensor sees the line

// ---------- Encoders ----------
volatile long encL = 0, encR = 0;
volatile uint8_t stL = 0, stR = 0;
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

// ---------- Motors ----------
void setMotor(int pwmPin, int in1, int in2, int duty) {
  duty = constrain(duty, -255, 255);
  digitalWrite(in1, duty > 0);
  digitalWrite(in2, duty < 0);
  ledcWrite(pwmPin, abs(duty));
}
void stopAll() { setMotor(PWMA, AIN1, AIN2, 0); setMotor(PWMB, BIN1, BIN2, 0); }

// ---------- Speed PI ----------
struct SpeedPI {
  float integ = 0;
  int update(float target, float meas) {
    if (target == 0) { integ = 0; return 0; }
    float err = target - meas;
    integ = constrain(integ + ski * err * DT, -80.0f, 80.0f);  // anti-windup
    return constrain((int)(kff * target + skp * err + integ), -255, 255);
  }
};
SpeedPI piL, piR;

// ---------- State ----------
QTRSensors qtr;
uint16_t qtrVals[8];
bool calibrated = false;

enum Mode { IDLE, SPEED, LINE } mode = IDLE;
float speedTarget = 0;              // for SPEED mode
float velL = 0, velR = 0;           // filtered counts/s
long  prevCL = 0, prevCR = 0;
float prevErr = 0; bool firstStep = true;
uint32_t lastSeenLine = 0;
int loopCount = 0;
bool outerHit = false;

void resetCtrl() {
  piL.integ = piR.integ = 0;
  firstStep = true;
  lastSeenLine = millis();
}

void goIdle(const char *why) {
  mode = IDLE; stopAll();
  Serial.printf("# IDLE (%s)\n", why);
}

// ---------- Control step (runs at 100 Hz) ----------
void controlStep() {
  long cL = getL(), cR = getR();
  float rawL = (cL - prevCL) / DT, rawR = (cR - prevCR) / DT;
  prevCL = cL; prevCR = cR;
  velL += 0.3f * (rawL - velL);     // light low-pass on quantized speed
  velR += 0.3f * (rawR - velR);

  float tL = 0, tR = 0, err = 0, corr = 0;

  if (mode == SPEED) {
    tL = tR = speedTarget;
  } else if (mode == LINE) {
    uint16_t pos = qtr.readLineBlack(qtrVals);
    uint16_t peak = 0;
    for (int i = 0; i < 8; i++) peak = max(peak, qtrVals[i]);
    if (peak > 300) lastSeenLine = millis();
    else if (millis() - lastSeenLine > 500) { goIdle("line lost"); return; }

    err = lineSign * ((float)pos - 3500.0f);          // -3500..+3500
    float d = firstStep ? 0 : (err - prevErr) / DT;
    prevErr = err; firstStep = false;
    outerHit = qtrVals[0] > outerThr || qtrVals[1] > outerThr ||
               qtrVals[6] > outerThr || qtrVals[7] > outerThr;
    float g = outerHit ? outerGain : 1.0f;          // scales err, so lineSign still applies
    float b = outerHit ? base * outerSlow : base;
    corr = g * (lkp * err + lkd * d);
    tL = b + corr;                                  // may go negative = pivot turn
    tR = b - corr;
  }

  if (mode == IDLE) return;

  int dutyL = piL.update(tL, velL);
  int dutyR = piR.update(tR, velR);
  setMotor(PWMA, AIN1, AIN2, dutyL);
  setMotor(PWMB, BIN1, BIN2, dutyR);

  if (++loopCount % 5 == 0) {       // 20 Hz telemetry
    if (mode == SPEED)
      Serial.printf("tgt:%.0f,velL:%.0f,velR:%.0f,dutyL:%d,dutyR:%d\n",
                    speedTarget, velL, velR, dutyL, dutyR);
    else
      Serial.printf("err:%.0f,corr:%.0f,outer:%d,velL:%.0f,velR:%.0f\n",
                    err, corr, outerHit ? 1000 : 0, velL, velR);
  }
}

// ---------- Calibration (blocking, motors off) ----------
void calibrate() {
  goIdle("calibrating");
  Serial.println("# Sweep the array across the line for 5 s...");
  qtr.resetCalibration();
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) qtr.calibrate();
  Serial.print("# min: "); for (int i = 0; i < 8; i++) Serial.printf("%u ", qtr.calibrationOn.minimum[i]);
  Serial.print("\n# max: "); for (int i = 0; i < 8; i++) Serial.printf("%u ", qtr.calibrationOn.maximum[i]);
  Serial.println();
  calibrated = true;
}

// ---------- Serial commands ----------
void printGains() {
  Serial.printf("# kff=%.4f skp=%.4f ski=%.4f | lkp=%.4f lkd=%.5f base=%.0f sign=%d\n",
                kff, skp, ski, lkp, lkd, base, lineSign);
  Serial.printf("# othr=%.0f ogain=%.2f oslow=%.2f\n", outerThr, outerGain, outerSlow);
}
void printHelp() {
  Serial.println("# Commands (New Line ending):");
  Serial.println("#  stop | s          stop motors");
  Serial.println("#  v <cps>           both wheels to speed (wheels OFF the ground)");
  Serial.println("#  cal               5 s line calibration");
  Serial.println("#  go                line follow (after cal)");
  Serial.println("#  kff|skp|ski <x>   speed loop gains");
  Serial.println("#  lkp|lkd <x>       line loop gains");
  Serial.println("#  base <cps>        line base speed");
  Serial.println("#  sign              flip steering direction");
  Serial.println("#  othr|ogain|oslow <x>  outer-sensor guard (threshold, gain x, speed frac)");
  Serial.println("#  p                 print gains");
}

void handleCmd(String s) {
  s.trim();
  if (!s.length()) return;
  int sp = s.indexOf(' ');
  String k = (sp < 0) ? s : s.substring(0, sp);
  bool hasV = sp >= 0;
  float v = hasV ? s.substring(sp + 1).toFloat() : 0;

  if (k == "stop" || k == "s") goIdle("stop");
  else if (k == "v" && hasV) { speedTarget = v; resetCtrl(); mode = SPEED; }
  else if (k == "cal") calibrate();
  else if (k == "go") {
    if (!calibrated) Serial.println("# run 'cal' first");
    else { resetCtrl(); mode = LINE; Serial.println("# LINE"); }
  }
  else if (k == "kff" && hasV) kff = v;
  else if (k == "skp" && hasV) skp = v;
  else if (k == "ski" && hasV) ski = v;
  else if (k == "lkp" && hasV) lkp = v;
  else if (k == "lkd" && hasV) lkd = v;
  else if (k == "base" && hasV) base = v;
  else if (k == "sign") lineSign = -lineSign;
  else if (k == "othr" && hasV) outerThr = v;
  else if (k == "ogain" && hasV) outerGain = v;
  else if (k == "oslow" && hasV) outerSlow = v;
  else if (k == "p") { printGains(); return; }
  else { printHelp(); return; }
  printGains();
}

String cmdBuf;
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleCmd(cmdBuf); cmdBuf = ""; }
    else if (cmdBuf.length() < 40) cmdBuf += c;
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

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
  prevCL = getL(); prevCR = getR();

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(QTR_CTRL);

  printHelp();
  printGains();
}

void loop() {
  pollSerial();
  static uint32_t next = micros();
  if ((int32_t)(micros() - next) >= 0) {
    next += LOOP_US;
    controlStep();
  }
}
