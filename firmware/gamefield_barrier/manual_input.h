#pragma once
#include <Arduino.h>

enum class ManualEvent : uint8_t { NONE, OPEN, CLOSE };

// Reads the pot. Fully turned one way = OPEN, fully the other way = CLOSE.
// Events are edge-triggered: one event when the knob ENTERS an end zone.
class ManualInput {
public:
  void begin();
  ManualEvent poll();
  int raw() const { return _raw; }

private:
  enum Zone : uint8_t { ZONE_MID, ZONE_OPEN, ZONE_CLOSE };
  Zone zoneFor(int value, Zone current) const;

  int  _raw = 0;
  Zone _stable = ZONE_MID;     // last confirmed zone
  Zone _candidate = ZONE_MID;  // zone being debounced
  unsigned long _candidateSince = 0;
  unsigned long _lastSample = 0;
};