#include "settings.h"
#include "config.h"
#include <EEPROM.h>

Settings g_settings;

static const uint16_t SETTINGS_MAGIC = 0xB4D1;   // change if the struct layout changes
static const int      EEPROM_ADDR    = 0;

struct Stored {
  uint16_t magic;
  Settings s;
  uint8_t  checksum;
};

static uint8_t calcChecksum(const Settings& s) {
  const uint8_t* p = (const uint8_t*)&s;
  uint8_t x = 0x5A;
  for (size_t i = 0; i < sizeof(Settings); i++) x ^= (uint8_t)(p[i] + i);
  return x;
}

static bool isValid(const Settings& s) {
  const uint32_t maxMs = (uint32_t)SETTINGS_MAX_TIME_S * 1000UL;
  return s.closedAngle <= 180 && s.openAngle <= 180 &&
         s.detectCm >= ULTRA_MIN_CM && s.detectCm <= ULTRA_MAX_CM &&
         s.openClosedHoldMs <= maxMs && s.closedDelayMs <= maxMs &&
         s.closedOpenHoldMs <= maxMs;
}

void settingsDefaults() {
  g_settings.closedAngle      = DEFAULT_CLOSED_ANGLE;
  g_settings.openAngle        = DEFAULT_OPEN_ANGLE;
  g_settings.detectCm         = DEFAULT_DETECT_CM;
  g_settings.openClosedHoldMs = DEFAULT_SEQ_FROM_OPEN_CLOSED_HOLD_MS;
  g_settings.closedDelayMs    = DEFAULT_SEQ_FROM_CLOSED_DELAY_MS;
  g_settings.closedOpenHoldMs = DEFAULT_SEQ_FROM_CLOSED_OPEN_HOLD_MS;
}

void settingsLoad() {
  Stored st;
  EEPROM.get(EEPROM_ADDR, st);
  if (st.magic == SETTINGS_MAGIC && st.checksum == calcChecksum(st.s) && isValid(st.s)) {
    g_settings = st.s;
  } else {
    settingsDefaults();   // first boot or corrupted EEPROM
  }
}

void settingsSave() {
  Stored st;
  st.magic = SETTINGS_MAGIC;
  st.s = g_settings;
  st.checksum = calcChecksum(g_settings);
  EEPROM.put(EEPROM_ADDR, st);   // only rewrites bytes that changed
}