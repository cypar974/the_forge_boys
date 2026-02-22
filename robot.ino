#include <Arduino.h>
#include <Controller.h>
#include <Servo.h>

// -------------------- Pins --------------------
static const int ENA = 9;   // PWM
static const int IN1 = 7;
static const int IN2 = 6;
static const int ENB = 10;  // PWM
static const int IN3 = 5;
static const int IN4 = 4;

static const int SERVO_PIN = 12;

// -------------------- WiFi Controller --------------------
Controller controller("1738", "12345678");

// -------------------- Servo (Catapult) --------------------
Servo catapult;

// Tune these
static const int RETRACT_ANGLE = 180;   // was 0

// Different “huck levels” mirrored (was 95/115/135)
static const int HUCK1_ANGLE = 85;      // 95
static const int HUCK2_ANGLE = 65;      // 115
static const int HUCK3_ANGLE = 45;      // 135

volatile int targetAngle = RETRACT_ANGLE;

// -------------------- Wheel drive tuning --------------------
int wheelTrim = 0;                  // + => left stronger, - => right stronger
unsigned long rampUpMs   = 200;
unsigned long rampDownMs = 150;

// -------------------- Motion macro (edit for your zigzag) --------------------
enum StepType : uint8_t { STEP_FWD, STEP_BACK, STEP_TURN_L, STEP_TURN_R, STEP_PAUSE };

struct Step {
  StepType type;
  uint16_t ms;
  uint8_t pwm;
};

Step path[] = {
  { STEP_FWD,    700, 150 },
  { STEP_TURN_R, 250, 140 },
  { STEP_FWD,    600, 150 },
  { STEP_TURN_L, 250, 140 },
  { STEP_FWD,    650, 150 },
  { STEP_TURN_R, 250, 140 },
  { STEP_FWD,    600, 150 },
  { STEP_PAUSE,  200,   0 },
};
const int PATH_LEN = sizeof(path) / sizeof(path[0]);

bool macroRunning = false;
int stepIdx = 0;
unsigned long stepStartMs = 0;

// -------------------- Motor cowpwpe;ww;;eww;w;;;w;''''''''''''''''''''''''''''''''''''''''entrol --------------------
void setMotorRaw(int enaPwm, bool aForward, int enbPwm, bool bForward) {
  enaPwm = constrain(enaPwm, 0, 255);
  enbPwm = constrain(enbPwm, 0, 255);

  digitalWrite(IN1, aForward ? HIGH : LOW);
  digitalWrite(IN2, aForward ? LOW  : HIGH);

  digitalWrite(IN3, bForward ? HIGH : LOW);
  digitalWrite(IN4, bForward ? LOW  : HIGH);

  analogWrite(ENA, enaPwm);
  analogWrite(ENB, enbPwm);
}

void stopWheels() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveDifferential(int leftPwm, bool leftFwd, int rightPwm, bool rightFwd,
                       unsigned long elapsed, unsigned long total) {
  int L = constrain(leftPwm  + wheelTrim, 0, 255);
  int R = constrain(rightPwm - wheelTrim, 0, 255);

  auto scale = [&](int pwm) -> int {
    if (total == 0) return 0;

    if (elapsed < rampUpMs) {
      float k = (float)elapsed / (float)max(1UL, rampUpMs);
      return (int)(pwm * constrain(k, 0.0f, 1.0f));
    }
    unsigned long remaining = (total > elapsed) ? (total - elapsed) : 0;
    if (remaining < rampDownMs) {
      float k = (float)remaining / (float)max(1UL, rampDownMs);
      return (int)(pwm * constrain(k, 0.0f, 1.0f));
    }
    return pwm;
  };

  setMotorRaw(scale(L), leftFwd, scale(R), rightFwd);
}

void runCurrentStep() {
  Step s = path[stepIdx];
  unsigned long now = millis();
  unsigned long elapsed = now - stepStartMs;

  if (elapsed >= s.ms) {
    stepIdx++;
    if (stepIdx >= PATH_LEN) {
      macroRunning = false;
      stopWheels();
      Serial.println("MACRO: DONE");
      return;
    }
    stepStartMs = now;
    return;
  }

  switch (s.type) {
    case STEP_FWD:    driveDifferential(s.pwm, true,  s.pwm, true,  elapsed, s.ms); break;
    case STEP_BACK:   driveDifferential(s.pwm, false, s.pwm, false, elapsed, s.ms); break;
    case STEP_TURN_L: driveDifferential(s.pwm, false, s.pwm, true,  elapsed, s.ms); break;
    case STEP_TURN_R: driveDifferential(s.pwm, true,  s.pwm, false, elapsed, s.ms); break;
    case STEP_PAUSE:  stopWheels(); break;
  }
}

// -------------------- Buttons --------------------
void btnRun()   { macroRunning = true; stepIdx = 0; stepStartMs = millis(); Serial.println("MACRO: START"); }
void btnStop()  { macroRunning = false; stopWheels(); Serial.println("MACRO: STOP"); }

void btnRetract() { targetAngle = RETRACT_ANGLE; Serial.println("SERVO: RETRACT"); }
void btnH1()      { targetAngle = HUCK1_ANGLE;   Serial.println("SERVO: H1"); }
void btnH2()      { targetAngle = HUCK2_ANGLE;   Serial.println("SERVO: H2"); }
void btnH3()      { targetAngle = HUCK3_ANGLE;   Serial.println("SERVO: H3"); }

void btnTrimL() { wheelTrim -= 2; Serial.print("TRIM="); Serial.println(wheelTrim); }
void btnTrimR() { wheelTrim += 2; Serial.print("TRIM="); Serial.println(wheelTrim); }

// -------------------- Setup / loop --------------------
void setup() {
  Serial.begin(115200);

  controller.configureL298N(ENA, IN1, IN2, ENB, IN3, IN4);
  controller.setMotorMinPWM(90);
  controller.setFailsafeTimeoutMs(1000);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  catapult.attach(SERVO_PIN);
  catapult.write(RETRACT_ANGLE);

  // No spaces in button names
  controller.registerButton("RUN", btnRun);
  controller.registerButton("STOP", btnStop);

  controller.registerButton("RETRACT", btnRetract);
  controller.registerButton("H1", btnH1);
  controller.registerButton("H2", btnH2);
  controller.registerButton("H3", btnH3);

  controller.registerButton("TRIML", btnTrimL);
  controller.registerButton("TRIMR", btnTrimR);

  controller.beginAP(true);
}

void loop() {
  controller.update();          // keep WiFi responsive
  catapult.write(targetAngle);  // keep servo commanded
  if (macroRunning) runCurrentStep();
}