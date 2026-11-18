#include "servo_barrier.h"
#include "config.h"
#include "settings.h"

void ServoBarrier::begin() {
  // Write BEFORE attach so the servo never twitches to a default position.
  _angle  = g_settings.closedAngle;
  _target = g_settings.closedAngle;
  _servo.write(g_settings.closedAngle);
  _servo.attach(PIN_SERVO, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

  _fault = FaultCode::NONE;
  _arrived = false;
  _lastStep = millis();
  _state = BarrierState::INITIALIZING;   // becomes CLOSED after settle time
}

bool ServoBarrier::open(bool force) {
  if (!force && (_state == BarrierState::FAULT || _state == BarrierState::INITIALIZING)) {
    return false;
  }
  if (_state == BarrierState::OPEN && _target == g_settings.openAngle) return true;

  _target = g_settings.openAngle;
  _arrived = false;
  if (_state != BarrierState::FAULT) _state = BarrierState::OPENING;
  return true;
}

bool ServoBarrier::close(bool force) {
  if (!force && (_state == BarrierState::FAULT || _state == BarrierState::INITIALIZING)) {
    return false;
  }
  if (_state == BarrierState::CLOSED && _target == g_settings.closedAngle) return true;

  _target = g_settings.closedAngle;
  _arrived = false;
  if (_state != BarrierState::FAULT) _state = BarrierState::CLOSING;
  return true;
}

void ServoBarrier::reset() {
  _fault = FaultCode::NONE;
  _target = g_settings.closedAngle;
  _arrived = false;
  _state = BarrierState::INITIALIZING;
}

void ServoBarrier::fault(FaultCode code) {
  _fault = code;
  _state = BarrierState::FAULT;
  _target = SAFE_STATE_OPEN ? g_settings.openAngle : g_settings.closedAngle;
  _arrived = false;
}

void ServoBarrier::onAnglesChanged(uint8_t oldClosed, uint8_t oldOpen) {
  if (_target == oldClosed)    _target = g_settings.closedAngle;
  else if (_target == oldOpen) _target = g_settings.openAngle;
  _arrived = false;
}

void ServoBarrier::update() {
  unsigned long now = millis();
  unsigned long dt = now - _lastStep;
  if (dt < SERVO_STEP_MS) return;
  _lastStep = now;

  if (_angle != (float)_target) {
    float step = SERVO_SPEED_DPS * dt / 1000.0f;
    float diff = _target - _angle;
    if (fabs(diff) <= step) _angle = _target;
    else _angle += (diff > 0) ? step : -step;
    _servo.write((int)(_angle + 0.5f));
    _arrived = false;
    return;
  }

  // At target
  if (!_arrived) {
    _arrived = true;
    _arrivedAt = now;
    return;
  }

  bool moving = (_state == BarrierState::OPENING ||
                 _state == BarrierState::CLOSING ||
                 _state == BarrierState::INITIALIZING);
  if (moving && (now - _arrivedAt >= SERVO_SETTLE_MS)) {
    _state = (_target == g_settings.openAngle) ? BarrierState::OPEN : BarrierState::CLOSED;
  }
}

const char* ServoBarrier::stateName(BarrierState s) {
  switch (s) {
    case BarrierState::UNKNOWN:      return "UNKNOWN";
    case BarrierState::INITIALIZING: return "INITIALIZING";
    case BarrierState::OPENING:      return "OPENING";
    case BarrierState::OPEN:         return "OPEN";
    case BarrierState::CLOSING:      return "CLOSING";
    case BarrierState::CLOSED:       return "CLOSED";
    case BarrierState::FAULT:        return "FAULT";
  }
  return "?";
}

const char* ServoBarrier::faultName(FaultCode f) {
  switch (f) {
    case FaultCode::NONE:       return "NONE";
    case FaultCode::COMMS_LOSS: return "COMMS_LOSS";
    case FaultCode::TEST:       return "TEST";
  }
  return "?";
}