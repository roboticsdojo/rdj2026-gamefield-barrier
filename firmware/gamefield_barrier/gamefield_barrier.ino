#include "config.h"
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

void printHelp() {
  Serial.println(F("Gamefield barrier test firmware"));
  Serial.println(F("  s = open    c = close    r = reset/re-init"));
  Serial.println(F("  p = status  h = heartbeat  f = inject fault  ? = help"));
  Serial.println(F("  Pot fully one way = open, fully the other = close"));
  Serial.println(F("  Ultrasonic detection runs the timed sequence automatically"));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_STATUS_LED, OUTPUT);

  barrier.begin();     // known state on power-up: INITIALIZING -> CLOSED
  manual.begin();
  ultra.begin();
  sequence.begin(&barrier);

  lastHeartbeat = millis();
  printHelp();
  printStatus();
}

void loop() {
  // ---- Serial commands ----
  Command cmd = serialCmd.poll();
  if (cmd != Command::NONE) {
    lastHeartbeat = millis();
    watchdogArmed = true;     // any serial traffic counts as "ROS side is alive"
  }

  switch (cmd) {
    case Command::OPEN:
      sequence.cancel();
      if (barrier.open()) Serial.println(F("ACK:OPEN"));
      else                Serial.println(F("NAK:OPEN (fault or initializing)"));
      break;
    case Command::CLOSE:
      sequence.cancel();
      if (barrier.close()) Serial.println(F("ACK:CLOSE"));
      else                 Serial.println(F("NAK:CLOSE (fault or initializing)"));
      break;
    case Command::RESET:
      sequence.cancel();
      barrier.reset();
      Serial.println(F("ACK:RESET"));
      break;
    case Command::STATUS:
      printStatus();
      break;
    case Command::HEARTBEAT:
      Serial.println(F("ACK:HB"));
      break;
    case Command::FAULT_TEST:
      sequence.cancel();
      barrier.fault(FaultCode::TEST);
      Serial.println(F("ACK:FAULT_INJECTED"));
      break;
    case Command::HELP:
      printHelp();
      break;
    case Command::UNKNOWN:
      Serial.println(F("NAK:UNKNOWN_CMD (? for help)"));
      break;
    case Command::NONE:
      break;
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