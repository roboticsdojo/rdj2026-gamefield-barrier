#include "detect_sequence.h"
#include "config.h"
#include "settings.h"

// Random wait between the configured minimum and maximum (inclusive)
static unsigned long pickWaitMs() {
  unsigned long lo = g_settings.waitMinMs;
  unsigned long hi = g_settings.waitMaxMs;
  if (hi <= lo) return lo;
  return (unsigned long)random((long)lo, (long)hi + 1);
}

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
  unsigned long waitMs = pickWaitMs();

  if (s == BarrierState::CLOSED) {
    // Closed: random wait, open, then stay open
    _buf[0] = Step{ Action::NONE, waitMs };
    _buf[1] = Step{ Action::OPEN, 0 };
    _count = 2;
    Serial.print(F("SEQ:START (from CLOSED) wait="));
  } else if (s == BarrierState::OPEN && SEQ_RETRIGGER_WHEN_OPEN) {
    // Optional: close, random wait, open again
    _buf[0] = Step{ Action::CLOSE, waitMs };
    _buf[1] = Step{ Action::OPEN, 0 };
    _count = 2;
    Serial.print(F("SEQ:START (from OPEN) wait="));
  } else {
    return;   // open (stay open), moving, fault or init: ignore this detection
  }
  Serial.print(waitMs / 1000.0f, 1);
  Serial.println('s');

  _steps = _buf;
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