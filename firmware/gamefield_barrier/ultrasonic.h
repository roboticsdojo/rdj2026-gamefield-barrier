#pragma once
#include <Arduino.h>

// HC-SR04 reader with debounce: needs several consecutive hits to report
// "detected" and several consecutive misses to report "clear".
class Ultrasonic {
public:
  void begin();
  void update();                    // call every loop(), rate-limited internally
  bool detected() const { return _detected; }
  int  distanceCm() const { return _dist; }   // -1 = no echo / out of range

private:
  int   _dist = -1;
  bool  _detected = false;
  uint8_t _hits = 0;
  uint8_t _misses = 0;
  unsigned long _last = 0;
};