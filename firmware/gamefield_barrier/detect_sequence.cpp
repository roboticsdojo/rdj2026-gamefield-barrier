#include "detect_sequence.h"
#include "config.h"

using Action = DetectSequence::Action;
using Step   = DetectSequence::Step;

// Barrier was OPEN: close, wait, open again
static const Step SEQ_FROM_OPEN[] = {
  { Action::CLOSE, SEQ_FROM_OPEN_CLOSED_HOLD_MS },
  { Action::OPEN,  0 }
};

// Barrier was CLOSED: wait, open, hold open, close again
static const Step SEQ_FROM_CLOSED[] = {
  { Action::NONE,  SEQ_FROM_CLOSED_DELAY_MS },
  { Action::OPEN,  SEQ_FROM_CLOSED_OPEN_HOLD_MS },
  { Action::CLOSE, 0 }
};

void DetectSequence::begin(ServoBarrier* barrier) {
  _barrier = barrier;
  _active = false;
  _armed = true;
}

void DetectSequence::cancel() {
  if (_active) {
    _active = false;
    Serial.println(F("SEQ:CANCELLED"));
  }
}

void DetectSequence::update(bool detected) {
  if (!SEQ_ENABLED || _barrier == nullptr) return;

  if (!_active) {
    // Re-arm only after the object has cleared (prevents endless re-triggering)
    if (!SEQ_REARM_ON_CLEAR || !detected) _armed = true;
    if (detected && _armed) tryStart();
    return;
  }

  unsigned long now = millis();
  BarrierState s = _barrier->state();

  // Anything abnormal -> abort and leave the barrier alone
  if (s == BarrierState::FAULT || s == BarrierState::INITIALIZING ||
      s == BarrierState::UNKNOWN) {
    finish(false);
    return;
  }

  const Step& st = _steps[_idx];

  if (!_reached) {
    BarrierState want = (st.action == Action::OPEN) ? BarrierState::OPEN
                                                    : BarrierState::CLOSED;
    if (s == want) {
      _reached = true;
      _dwellStart = now;
    } else if (now - _stepStart > SEQ_STEP_TIMEOUT_MS) {
      finish(false);   // barrier never reached the target
      return;
    } else {
      return;
    }
  }

  if (now - _dwellStart >= st.dwellMs) {
    if (_idx + 1 >= _count) finish(true);
    else startStep(_idx + 1);
  }
}

void DetectSequence::tryStart() {
  BarrierState s = _barrier->state();
  if (s == BarrierState::CLOSED) {
    _steps = SEQ_FROM_CLOSED;
    _count = sizeof(SEQ_FROM_CLOSED) / sizeof(Step);
    Serial.println(F("SEQ:START (from CLOSED)"));
  } else if (s == BarrierState::OPEN) {
    _steps = SEQ_FROM_OPEN;
    _count = sizeof(SEQ_FROM_OPEN) / sizeof(Step);
    Serial.println(F("SEQ:START (from OPEN)"));
  } else {
    return;   // moving / fault / init: ignore this detection
  }
  _armed = false;
  _active = true;
  startStep(0);
}

void DetectSequence::startStep(uint8_t idx) {
  _idx = idx;
  _stepStart = millis();
  _reached = false;

  const Step& st = _steps[idx];
  bool ok = true;
  if (st.action == Action::OPEN)  ok = _barrier->open();
  if (st.action == Action::CLOSE) ok = _barrier->close();
  if (!ok) { finish(false); return; }

  if (st.action == Action::NONE) {   // pure wait step
    _reached = true;
    _dwellStart = _stepStart;
  }
}

void DetectSequence::finish(bool ok) {
  _active = false;
  Serial.println(ok ? F("SEQ:DONE") : F("SEQ:ABORT"));
}