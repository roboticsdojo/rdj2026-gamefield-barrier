#include "ultrasonic.h"
#include "config.h"
#include "settings.h"

void Ultrasonic::begin() {
  pinMode(PIN_ULTRA_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_ECHO, INPUT);
  digitalWrite(PIN_ULTRA_TRIG, LOW);
}

void Ultrasonic::update() {
  unsigned long now = millis();
  if (now - _last < ULTRA_SAMPLE_MS) return;
  _last = now;

  digitalWrite(PIN_ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRA_TRIG, LOW);

  // Timeout limits blocking time to the max range we care about (~58 us per cm)
  unsigned long us = pulseIn(PIN_ULTRA_ECHO, HIGH, (unsigned long)ULTRA_MAX_CM * 58UL);
  _dist = (us == 0) ? -1 : (int)(us / 58UL);

  bool hit = (_dist >= ULTRA_MIN_CM && _dist <= (int)g_settings.detectCm);
  if (hit) {
    if (_hits < 255) _hits++;
    _misses = 0;
    if (_hits >= ULTRA_CONFIRM_COUNT) _detected = true;
  } else {
    if (_misses < 255) _misses++;
    _hits = 0;
    if (_misses >= ULTRA_CLEAR_COUNT) _detected = false;
  }
}