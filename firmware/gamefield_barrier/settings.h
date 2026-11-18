#pragma once
#include <Arduino.h>

// Runtime-tunable settings, stored in EEPROM.
struct Settings {
  uint8_t  closedAngle;        // start / closed angle (deg)
  uint8_t  openAngle;          // open angle (deg)
  uint16_t detectCm;           // ultrasonic detection distance (cm)
  uint32_t openClosedHoldMs;   // barrier was OPEN: how long to stay closed
  uint32_t closedDelayMs;      // barrier was CLOSED: wait before opening
  uint32_t closedOpenHoldMs;   // barrier was CLOSED: how long to stay open
};

extern Settings g_settings;

void settingsDefaults();   // load defaults from config.h into g_settings (RAM only)
void settingsLoad();       // EEPROM if valid, otherwise defaults
void settingsSave();       // write g_settings to EEPROM