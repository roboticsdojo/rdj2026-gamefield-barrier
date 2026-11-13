#include "manual_input.h"
#include "config.h"

void ManualInput::begin() {
  pinMode(PIN_POT, INPUT);
  _raw = analogRead(PIN_POT);
  _stable = zoneFor(_raw, ZONE_MID);   // no event at boot, just sync
  _candidate = _stable;
  _candidateSince = millis();
}

ManualInput::Zone ManualInput::zoneFor(int v, Zone cur) const {
  int openEnter  = POT_OPEN_THRESHOLD;
  int closeEnter = POT_CLOSE_THRESHOLD;
  // Hysteresis: easier to stay in a zone than to enter it
  if (cur == ZONE_OPEN)  openEnter  -= POT_HYSTERESIS;
  if (cur == ZONE_CLOSE) closeEnter += POT_HYSTERESIS;

  if (v >= openEnter)  return ZONE_OPEN;
  if (v <= closeEnter) return ZONE_CLOSE;
  return ZONE_MID;
}

ManualEvent ManualInput::poll() {
  unsigned long now = millis();
  if (now - _lastSample < POT_SAMPLE_MS) return ManualEvent::NONE;
  _lastSample = now;

  // Average a few reads to reduce ADC noise
  long sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(PIN_POT);
  _raw = sum / 4;

  Zone z = zoneFor(_raw, _stable);
  if (z != _candidate) {
    _candidate = z;
    _candidateSince = now;
    return ManualEvent::NONE;
  }

  if (_candidate != _stable && (now - _candidateSince >= POT_DEBOUNCE_MS)) {
    _stable = _candidate;
    if (_stable == ZONE_OPEN)  return ManualEvent::OPEN;
    if (_stable == ZONE_CLOSE) return ManualEvent::CLOSE;
  }
  return ManualEvent::NONE;
}