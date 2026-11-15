// Hardware configuration of the horn mount.
//
// EVERY value below is an ASSUMPTION until it is replaced by the real wiring
// and the real mechanics. Nothing here has been checked against hardware.
// The firmware works in STEPS only; millimetres and degrees are converted in
// the Python driver (rada_align/hw/esp32_actuators.py) from config.yaml.
#pragma once
#include <Arduino.h>

#define SERIAL_BAUD 115200           // must match hw.baud in config.yaml

// Axis order everywhere: X, Z, PAN, TILT.
#define AXIS_COUNT 4

// --- Pin map (STEP/DIR drivers: TMC2209 standalone, A4988, DRV8825 all fit) ---
// ASSUMPTION - replace with the real wiring.
static const uint8_t STEP_PIN[AXIS_COUNT]  = {16, 18, 21, 23};
static const uint8_t DIR_PIN[AXIS_COUNT]   = {17, 19, 22, 25};
static const uint8_t ENABLE_PIN            = 26;   // shared, active LOW on most step/dir drivers
#define ENABLE_ACTIVE_LOW 1                        // ASSUMPTION - replace with the real driver

// --- Limit switches ---
#define HAS_LIMIT_SWITCHES 0                       // ASSUMPTION - replace: 1 if a switch is fitted per axis
static const uint8_t LIMIT_PIN[AXIS_COUNT] = {32, 33, 34, 35};   // ASSUMPTION; 34/35 need external pull-ups
#define LIMIT_ACTIVE_LOW 1                         // ASSUMPTION - switch to ground, pull-up

// --- Emergency stop: normally-closed button to ground. Open circuit = STOP,
//     so a broken wire also stops the mount. ---
static const uint8_t ESTOP_PIN = 27;               // ASSUMPTION - replace with the real wiring
#define ESTOP_TRIPPED_LEVEL HIGH                   // with INPUT_PULLUP and an NC button to ground

// --- Beacon LED ---
static const uint8_t LED_PIN = 4;                  // ASSUMPTION - replace with the real wiring

// --- Motion, in steps. ASSUMPTION - replace with the real mechanics. ---
// Soft limits (steps from the zero set by HOME). Defaults correspond to
// config.yaml: X/Z +/-250 mm at 400 steps/mm, PAN/TILT +/-15 deg at 888.9 steps/deg.
static const long SOFT_MIN[AXIS_COUNT] = {-100000, -100000, -13333, -13333};
static const long SOFT_MAX[AXIS_COUNT] = { 100000,  100000,  13333,  13333};
// Speed (steps/s) and acceleration (steps/s^2) at power-up; SPEED changes them.
// Deliberately slow for first tests.
static const float DEFAULT_SPEED[AXIS_COUNT] = {2000, 2000, 500, 500};
static const float DEFAULT_ACCEL[AXIS_COUNT] = {4000, 4000, 1000, 1000};
static const float MAX_SPEED_LIMIT = 20000;        // SPEED above this is refused
static const bool  DIR_INVERT[AXIS_COUNT] = {false, false, false, false};
static const float HOMING_SPEED[AXIS_COUNT] = {800, 800, 200, 200};   // toward the switch (negative direction)

// --- Watchdog: all motion stops if no PING arrives for this long. ---
#define WATCHDOG_MS 1000
