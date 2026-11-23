#include "config.h"
#include "settings.h"
#include "servo_barrier.h"
#include "manual_input.h"
#include "serial_cmd.h"
#include "ultrasonic.h"
#include "detect_sequence.h"

ServoBarrier   barrier;
ManualInput    manual;
SerialCommand  serialCmd;
Ultrasonic     ultra;
DetectSequence sequence;

BarrierState  lastReported = BarrierState::UNKNOWN;
unsigned long lastHeartbeat = 0;
unsigned long lastStatus = 0;
bool          watchdogArmed = false;

void printStatus() {
  Serial.print(F("STATE:"));  Serial.print(ServoBarrier::stateName(barrier.state()));
  Serial.print(F(" ANGLE:")); Serial.print(barrier.angle());
  Serial.print(F(" POT:"));   Serial.print(manual.raw());
  Serial.print(F(" DIST:"));  Serial.print(ultra.distanceCm());
  Serial.print(F(" SEQ:"));   Serial.print(sequence.active() ? F("RUN") : F("IDLE"));
  Serial.print(F(" FAULT:")); Serial.println(ServoBarrier::faultName(barrier.faultCode()));
}

void printSettings() {
  Serial.print(F("SETTINGS closed_angle:")); Serial.print(g_settings.closedAngle);
  Serial.print(F(" open_angle:"));           Serial.print(g_settings.openAngle);
  Serial.print(F(" detect_cm:"));            Serial.print(g_settings.detectCm);
  Serial.print(F(" wait_min_s:"));           Serial.print(g_settings.waitMinMs / 1000.0f, 2);
  Serial.print(F(" wait_max_s:"));           Serial.println(g_settings.waitMaxMs / 1000.0f, 2);
}

void printHelp() {
  Serial.println(F("Gamefield barrier firmware"));
  Serial.println(F("  s = open    c = close    r = reset/re-init"));
  Serial.println(F("  p = status  h = heartbeat  f = inject fault  ? = help"));
  Serial.println(F("  d<cm>  detect distance, e.g. d40 (2-200)"));
  Serial.println(F("  t<sec> MIN random wait before opening, e.g. t5 (0-600)"));
  Serial.println(F("  m<sec> MAX random wait before opening, e.g. m15 (0-600)"));
  Serial.println(F("  a<deg> start/closed angle, e.g. a20    o<deg> open angle, e.g. o110"));
  Serial.println(F("  v = show settings    x1 = restore defaults"));
  Serial.println(F("  Pot fully one way = open, fully the other = close"));
  Serial.println(F("  Object detected while closed -> random wait -> opens and STAYS open"));
}

void nakRange(const __FlashStringHelper* what, int lo, int hi) {
  Serial.print(F("NAK:")); Serial.print(what);
  Serial.print(F(" must be ")); Serial.print(lo);
  Serial.print('-'); Serial.println(hi);
}

void seedRandom() {
  // A1 is left unconnected: its noisy low bit gives us entropy
  uint32_t seed = 0;
  for (uint8_t i = 0; i < 32; i++) {
    seed = (seed << 1) ^ (uint32_t)(analogRead(PIN_RANDOM_SEED) & 1);
    delayMicroseconds(200);
  }
  randomSeed(seed ^ micros());
}

void handleCommand(const Command& cmd) {
  switch (cmd.type) {
    case CmdType::OPEN:
      sequence.cancel();
      if (barrier.open()) Serial.println(F("ACK:OPEN"));
      else                Serial.println(F("NAK:OPEN (fault or initializing)"));
      break;
    case CmdType::CLOSE:
      sequence.cancel();
      if (barrier.close()) Serial.println(F("ACK:CLOSE"));
      else                 Serial.println(F("NAK:CLOSE (fault or initializing)"));
      break;
    case CmdType::RESET:
      sequence.cancel();
      barrier.reset();
      Serial.println(F("ACK:RESET"));
      break;
    case CmdType::STATUS:
      printStatus();
      break;
    case CmdType::HEARTBEAT:
      Serial.println(F("ACK:HB"));
      break;
    case CmdType::FAULT_TEST:
      sequence.cancel();
      barrier.fault(FaultCode::TEST);
      Serial.println(F("ACK:FAULT_INJECTED"));
      break;
    case CmdType::HELP:
      printHelp();
      break;
    case CmdType::SHOW_SETTINGS:
      printSettings();
      break;

    case CmdType::SET_DETECT_CM:
      if (cmd.value >= ULTRA_MIN_CM && cmd.value <= ULTRA_MAX_CM) {
        g_settings.detectCm = (uint16_t)(cmd.value + 0.5f);
        settingsSave();
        Serial.print(F("ACK:DETECT_CM=")); Serial.println(g_settings.detectCm);
      } else {
        nakRange(F("DETECT_CM"), ULTRA_MIN_CM, ULTRA_MAX_CM);
      }
      break;

    case CmdType::SET_WAIT_MIN:
    case CmdType::SET_WAIT_MAX:
      if (cmd.value >= 0 && cmd.value <= SETTINGS_MAX_TIME_S) {
        uint32_t ms = (uint32_t)(cmd.value * 1000.0f + 0.5f);
        if (cmd.type == CmdType::SET_WAIT_MIN) {
          if (ms > g_settings.waitMaxMs) {
            Serial.println(F("NAK:WAIT_MIN must be <= WAIT_MAX (raise it first with m<sec>)"));
            break;
          }
          g_settings.waitMinMs = ms;
          Serial.print(F("ACK:WAIT_MIN_S="));
        } else {
          if (ms < g_settings.waitMinMs) {
            Serial.println(F("NAK:WAIT_MAX must be >= WAIT_MIN (lower it first with t<sec>)"));
            break;
          }
          g_settings.waitMaxMs = ms;
          Serial.print(F("ACK:WAIT_MAX_S="));
        }
        settingsSave();
        Serial.println(ms / 1000.0f, 2);
      } else {
        nakRange(F("WAIT_S"), 0, SETTINGS_MAX_TIME_S);
      }
      break;

    case CmdType::SET_CLOSED_ANGLE:
    case CmdType::SET_OPEN_ANGLE:
      if (cmd.value >= 0 && cmd.value <= 180) {
        sequence.cancel();
        uint8_t oldClosed = g_settings.closedAngle;
        uint8_t oldOpen   = g_settings.openAngle;
        uint8_t angle = (uint8_t)(cmd.value + 0.5f);
        if (cmd.type == CmdType::SET_CLOSED_ANGLE) {
          g_settings.closedAngle = angle;
          Serial.print(F("ACK:CLOSED_ANGLE="));
        } else {
          g_settings.openAngle = angle;
          Serial.print(F("ACK:OPEN_ANGLE="));
        }
        Serial.println(angle);
        settingsSave();
        barrier.onAnglesChanged(oldClosed, oldOpen);
      } else {
        nakRange(F("ANGLE"), 0, 180);
      }
      break;

    case CmdType::FACTORY_RESET:
      if (cmd.value == 1) {
        sequence.cancel();
        uint8_t oldClosed = g_settings.closedAngle;
        uint8_t oldOpen   = g_settings.openAngle;
        settingsDefaults();
        settingsSave();
        barrier.onAnglesChanged(oldClosed, oldOpen);
        Serial.println(F("ACK:DEFAULTS_RESTORED"));
        printSettings();
      } else {
        Serial.println(F("NAK:send x1 to restore defaults"));
      }
      break;

    case CmdType::BAD_VALUE:
      Serial.println(F("NAK:BAD_VALUE (example: d40, t5, m15, a20, o110)"));
      break;
    case CmdType::UNKNOWN:
      Serial.println(F("NAK:UNKNOWN_CMD (? for help)"));
      break;
    case CmdType::NONE:
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_STATUS_LED, OUTPUT);

  seedRandom();
  settingsLoad();      // must come before barrier.begin()
  barrier.begin();     // known state on power-up: INITIALIZING -> CLOSED
  manual.begin();
  ultra.begin();
  sequence.begin(&barrier);

  lastHeartbeat = millis();
  printHelp();
  printSettings();
  printStatus();
}

void loop() {
  // ---- Serial commands ----
  Command cmd = serialCmd.poll();
  if (cmd.type != CmdType::NONE) {
    lastHeartbeat = millis();
    watchdogArmed = true;     // any serial traffic counts as "ROS side is alive"
    handleCommand(cmd);
  }

  // ---- Manual knob override (works even in FAULT / without comms) ----
  ManualEvent ev = manual.poll();
  if (ev == ManualEvent::OPEN)  { sequence.cancel(); barrier.open(true);  Serial.println(F("MANUAL:OPEN")); }
  if (ev == ManualEvent::CLOSE) { sequence.cancel(); barrier.close(true); Serial.println(F("MANUAL:CLOSE")); }

  // ---- Comms-loss watchdog (disabled by default in config.h) ----
  if (COMMS_WATCHDOG_ENABLED && watchdogArmed &&
      barrier.state() != BarrierState::FAULT &&
      millis() - lastHeartbeat > COMMS_TIMEOUT_MS) {
    barrier.fault(FaultCode::COMMS_LOSS);
  }

  // ---- Ultrasonic + detection sequence ----
  ultra.update();
  sequence.update(ultra.detected());

  // ---- Motion + state machine ----
  barrier.update();

  // ---- Report state changes ----
  if (barrier.state() != lastReported) {
    lastReported = barrier.state();
    printStatus();
  }
  if (STATUS_PERIOD_MS > 0 && millis() - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = millis();
    printStatus();
  }

  // LED: on = OPEN, blinking = moving/init, fast blink = FAULT, off = CLOSED
  switch (barrier.state()) {
    case BarrierState::OPEN:   digitalWrite(PIN_STATUS_LED, HIGH); break;
    case BarrierState::CLOSED: digitalWrite(PIN_STATUS_LED, LOW);  break;
    case BarrierState::FAULT:  digitalWrite(PIN_STATUS_LED, (millis() / 100) % 2); break;
    default:                   digitalWrite(PIN_STATUS_LED, (millis() / 400) % 2); break;
  }
}