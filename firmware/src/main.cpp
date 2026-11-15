// RADA Align horn mount: four stepper axes, beacon LED, line-based serial protocol.
//
// One command per line, one reply line per command: "OK ..." or "ERR <reason>".
//   PING                         keep-alive; motion needs one at least every second
//   MOVE <axis> <steps>          relative move
//   MOVETO <axis> <pos>          absolute move (steps)
//   HOME <axis>                  with limit switches: seek the switch, set zero there;
//                                without: declare the current position zero
//   STOP                         stop every axis at once
//   STATUS                       OK pos=x,z,p,t moving=0000 limits=0000 estop=0 wd=0 homed=0000
//   SPEED <axis> <v> <a>         max speed (steps/s) and acceleration (steps/s^2)
//   LED <on|off|blink hz>
// Axes: X Z PAN TILT. Safety: soft limits, e-stop input, 1 s watchdog.
#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"

static const char *AXIS_NAME[AXIS_COUNT] = {"X", "Z", "PAN", "TILT"};

static AccelStepper steppers[AXIS_COUNT] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[3], DIR_PIN[3]),
};

static bool homing[AXIS_COUNT] = {false, false, false, false};
static bool homed[AXIS_COUNT] = {false, false, false, false};
static unsigned long lastPingMs = 0;
static bool watchdogTripped = true;   // no motion until the first PING
static bool estopLatched = false;

static bool ledBlink = false;
static unsigned long ledHalfPeriodMs = 0;
static unsigned long ledToggleMs = 0;
static bool ledState = false;

static char line[96];
static size_t lineLen = 0;

static void setDriversEnabled(bool on) {
  digitalWrite(ENABLE_PIN, (on == (ENABLE_ACTIVE_LOW == 1)) ? LOW : HIGH);
}

// Hard stop: no deceleration ramp. Used for STOP, e-stop and watchdog.
static void stopAll() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    steppers[i].setCurrentPosition(steppers[i].currentPosition());  // also zeroes the speed
    homing[i] = false;
  }
}

static bool estopActive() { return digitalRead(ESTOP_PIN) == ESTOP_TRIPPED_LEVEL; }

static bool limitActive(int axis) {
#if HAS_LIMIT_SWITCHES
  return digitalRead(LIMIT_PIN[axis]) == (LIMIT_ACTIVE_LOW ? LOW : HIGH);
#else
  (void)axis;
  return false;
#endif
}

static int axisIndex(const char *name) {
  if (name == nullptr) return -1;
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (strcmp(name, AXIS_NAME[i]) == 0) return i;
  }
  return -1;
}

static bool parseLong(const char *text, long &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  value = strtol(text, &end, 10);
  return *end == '\0';
}

static bool parseFloat(const char *text, float &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return *end == '\0';
}

// Returns nullptr if motion is allowed, otherwise the reason it is not.
static const char *motionBlocked() {
  if (estopLatched || estopActive()) return "ESTOP";
  if (watchdogTripped) return "WATCHDOG";
  return nullptr;
}

static void replyStatus() {
  Serial.print("OK pos=");
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(steppers[i].currentPosition());
  }
  Serial.print(" moving=");
  for (int i = 0; i < AXIS_COUNT; i++) Serial.print(steppers[i].distanceToGo() != 0 || homing[i] ? '1' : '0');
  Serial.print(" limits=");
  for (int i = 0; i < AXIS_COUNT; i++) Serial.print(limitActive(i) ? '1' : '0');
  Serial.print(" estop=");
  Serial.print(estopLatched || estopActive() ? '1' : '0');
  Serial.print(" wd=");
  Serial.print(watchdogTripped ? '1' : '0');
  Serial.print(" homed=");
  for (int i = 0; i < AXIS_COUNT; i++) Serial.print(homed[i] ? '1' : '0');
  Serial.println();
}

static void commandMove(bool absolute) {
  int axis = axisIndex(strtok(nullptr, " "));
  long value;
  if (axis < 0) { Serial.println("ERR BAD_AXIS"); return; }
  if (!parseLong(strtok(nullptr, " "), value)) { Serial.println("ERR BAD_NUMBER"); return; }
  const char *blocked = motionBlocked();
  if (blocked) { Serial.print("ERR "); Serial.println(blocked); return; }
  if (homing[axis]) { Serial.println("ERR HOMING"); return; }
  long target = absolute ? value : steppers[axis].targetPosition() + value;
  if (target < SOFT_MIN[axis] || target > SOFT_MAX[axis]) { Serial.println("ERR SOFT_LIMIT"); return; }
  steppers[axis].moveTo(target);
  Serial.print("OK ");
  Serial.println(target);
}

static void commandHome() {
  int axis = axisIndex(strtok(nullptr, " "));
  if (axis < 0) { Serial.println("ERR BAD_AXIS"); return; }
  const char *blocked = motionBlocked();
  if (blocked) { Serial.print("ERR "); Serial.println(blocked); return; }
#if HAS_LIMIT_SWITCHES
  homing[axis] = true;
  homed[axis] = false;
  steppers[axis].setSpeed(-HOMING_SPEED[axis]);
  Serial.println("OK HOMING");
#else
  steppers[axis].setCurrentPosition(0);
  homed[axis] = true;
  Serial.println("OK ZEROED_HERE");   // no switch: the current position becomes zero
#endif
}

static void commandSpeed() {
  int axis = axisIndex(strtok(nullptr, " "));
  float v, a;
  if (axis < 0) { Serial.println("ERR BAD_AXIS"); return; }
  if (!parseFloat(strtok(nullptr, " "), v) || !parseFloat(strtok(nullptr, " "), a)) { Serial.println("ERR BAD_NUMBER"); return; }
  if (v <= 0 || a <= 0 || v > MAX_SPEED_LIMIT) { Serial.println("ERR OUT_OF_RANGE"); return; }
  steppers[axis].setMaxSpeed(v);
  steppers[axis].setAcceleration(a);
  Serial.println("OK");
}

static void commandLed() {
  const char *mode = strtok(nullptr, " ");
  if (mode == nullptr) { Serial.println("ERR BAD_ARGUMENT"); return; }
  if (strcmp(mode, "ON") == 0 || strcmp(mode, "OFF") == 0) {
    ledBlink = false;
    ledState = strcmp(mode, "ON") == 0;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.println("OK");
  } else if (strcmp(mode, "BLINK") == 0) {
    float hz;
    if (!parseFloat(strtok(nullptr, " "), hz) || hz <= 0 || hz > 500) { Serial.println("ERR BAD_NUMBER"); return; }
    ledBlink = true;
    ledHalfPeriodMs = (unsigned long)(500.0f / hz);
    if (ledHalfPeriodMs == 0) ledHalfPeriodMs = 1;
    ledToggleMs = millis();
    Serial.println("OK");
  } else {
    Serial.println("ERR BAD_ARGUMENT");
  }
}

static void handleLine() {
  for (size_t i = 0; i < lineLen; i++) line[i] = toupper((unsigned char)line[i]);
  char *command = strtok(line, " ");
  if (command == nullptr) return;   // empty line
  if (strcmp(command, "PING") == 0) {
    lastPingMs = millis();
    watchdogTripped = false;
    // An e-stop latch is released only by PING with the button released.
    if (estopLatched && !estopActive()) { estopLatched = false; setDriversEnabled(true); }
    Serial.println(estopLatched ? "ERR ESTOP" : "OK");
  } else if (strcmp(command, "MOVE") == 0) {
    commandMove(false);
  } else if (strcmp(command, "MOVETO") == 0) {
    commandMove(true);
  } else if (strcmp(command, "HOME") == 0) {
    commandHome();
  } else if (strcmp(command, "STOP") == 0) {
    stopAll();
    Serial.println("OK");
  } else if (strcmp(command, "STATUS") == 0) {
    replyStatus();
  } else if (strcmp(command, "SPEED") == 0) {
    commandSpeed();
  } else if (strcmp(command, "LED") == 0) {
    commandLed();
  } else {
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(ENABLE_PIN, OUTPUT);
  setDriversEnabled(false);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#if HAS_LIMIT_SWITCHES
  for (int i = 0; i < AXIS_COUNT; i++) pinMode(LIMIT_PIN[i], LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
#endif
  for (int i = 0; i < AXIS_COUNT; i++) {
    steppers[i].setPinsInverted(DIR_INVERT[i], false, false);
    steppers[i].setMaxSpeed(DEFAULT_SPEED[i]);
    steppers[i].setAcceleration(DEFAULT_ACCEL[i]);
    steppers[i].setCurrentPosition(0);
  }
  estopLatched = estopActive();
  if (!estopLatched) setDriversEnabled(true);
  Serial.println("OK RADA-ALIGN READY");
}

void loop() {
  // 1. Safety first, every pass.
  if (estopActive() && !estopLatched) {
    estopLatched = true;
    stopAll();
    setDriversEnabled(false);
  }
  if (!watchdogTripped && millis() - lastPingMs > WATCHDOG_MS) {
    watchdogTripped = true;
    stopAll();
  }

  // 2. Motion.
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (homing[i]) {
      if (limitActive(i)) {
        steppers[i].setCurrentPosition(0);
        homing[i] = false;
        homed[i] = true;
      } else {
        steppers[i].runSpeed();
      }
    } else {
      steppers[i].run();
    }
  }

  // 3. Beacon.
  if (ledBlink && millis() - ledToggleMs >= ledHalfPeriodMs) {
    ledToggleMs += ledHalfPeriodMs;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // 4. Commands.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      handleLine();
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;   // overlong line: drop it
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
}
