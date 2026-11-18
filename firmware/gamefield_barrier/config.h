#pragma once
#include <Arduino.h>

// ---------------- Pins ----------------
constexpr uint8_t PIN_SERVO      = 9;
constexpr uint8_t PIN_POT        = A0;
constexpr uint8_t PIN_ULTRA_TRIG = 7;
constexpr uint8_t PIN_ULTRA_ECHO = 8;
constexpr uint8_t PIN_STATUS_LED = LED_BUILTIN;

// ---------------- Serial ----------------
constexpr unsigned long SERIAL_BAUD = 115200;

// ---------------- Servo ----------------
constexpr int   SERVO_PULSE_MIN_US = 544;    // tune for your servo
constexpr int   SERVO_PULSE_MAX_US = 2400;
constexpr float SERVO_SPEED_DPS    = 90.0f;  // degrees per second (smooth motion)
constexpr unsigned long SERVO_STEP_MS   = 20;   // motion update period
constexpr unsigned long SERVO_SETTLE_MS = 300;  // wait after arrival before OPEN/CLOSED
                                                // (servo has no position feedback)

// ---------------- Defaults for runtime-tunable settings ----------------
// Used on the very first boot and after the 'x1' command. Once values are saved in
// EEPROM, changing these and re-flashing has NO effect until you send 'x1'.
constexpr int DEFAULT_CLOSED_ANGLE = 0;      // start / closed angle (deg)
constexpr int DEFAULT_OPEN_ANGLE   = 90;     // open angle (deg)
constexpr int DEFAULT_DETECT_CM    = 30;     // ultrasonic detection distance (cm)
// Barrier was OPEN when object detected: close, hold closed this long, open
constexpr unsigned long DEFAULT_SEQ_FROM_OPEN_CLOSED_HOLD_MS = 5000;
// Barrier was CLOSED when object detected: wait, open, hold open, close
constexpr unsigned long DEFAULT_SEQ_FROM_CLOSED_DELAY_MS     = 5000;
constexpr unsigned long DEFAULT_SEQ_FROM_CLOSED_OPEN_HOLD_MS = 5000;

// ---------------- Limits for tunable settings ----------------
constexpr int ULTRA_MIN_CM        = 2;       // ignore readings below this (noise)
constexpr int ULTRA_MAX_CM        = 200;     // max detect distance / echo timeout range
constexpr int SETTINGS_MAX_TIME_S = 600;     // max sequence time you can set

// ---------------- Manual potentiometer ----------------
constexpr int  POT_OPEN_THRESHOLD  = 900;    // raw ADC (0-1023): above = knob fully OPEN
constexpr int  POT_CLOSE_THRESHOLD = 120;    // below = knob fully CLOSED
constexpr int  POT_HYSTERESIS      = 40;     // stops flicker at the thresholds
constexpr unsigned long POT_SAMPLE_MS   = 10;
constexpr unsigned long POT_DEBOUNCE_MS = 80; // zone must be stable this long

// ---------------- Ultrasonic (HC-SR04) ----------------
constexpr unsigned long ULTRA_SAMPLE_MS = 60;
constexpr uint8_t ULTRA_CONFIRM_COUNT = 3;   // consecutive hits to trigger
constexpr uint8_t ULTRA_CLEAR_COUNT   = 3;   // consecutive misses to clear

// ---------------- Detection sequence ----------------
constexpr bool SEQ_ENABLED = true;
// Abort if the barrier takes longer than this to reach a commanded position
constexpr unsigned long SEQ_STEP_TIMEOUT_MS = 5000;
// true = after a sequence, wait until the object clears before allowing another
constexpr bool SEQ_REARM_ON_CLEAR = true;

// ---------------- Comms watchdog (for the ROS/Pi side) ----------------
// Disabled by default so the serial monitor test doesn't fault.
constexpr bool          COMMS_WATCHDOG_ENABLED = false;
constexpr unsigned long COMMS_TIMEOUT_MS       = 3000;
// Where the barrier goes on a critical fault: true = open angle, false = closed angle.
// Check against the competition rules.
constexpr bool          SAFE_STATE_OPEN = true;

// ---------------- Reporting ----------------
constexpr unsigned long STATUS_PERIOD_MS = 0;  // 0 = only print on state change/'p'

//servo - D9
//pot - A0
//trigger - D7
//echo - D8