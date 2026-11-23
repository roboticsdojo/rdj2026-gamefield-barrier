#pragma once
#include <Arduino.h>

// Runtime-tunable settings, stored in EEPROM.
struct Settings {
  uint8_t  closedAngle;   // start / closed angle (deg)
  uint8_t  openAngle;     // open angle (deg)
  uint16_t detectCm;      // ultrasonic detection distance (cm)
  uint32_t waitMinMs;     // random wait before opening: minimum
  uint32_t waitMaxMs;     // random wait before opening: maximum
};

extern Settings g_settings;

void settingsDefaults();   // load defaults from config.h into g_settings (RAM only)
void settingsLoad();       // EEPROM if valid, otherwise defaults
void settingsSave();       // write g_settings to EEPROM