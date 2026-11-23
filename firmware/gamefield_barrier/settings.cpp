#include "settings.h"
#include "config.h"
#include <EEPROM.h>

Settings g_settings;

static const uint16_t SETTINGS_MAGIC = 0xB4D2;   // change if the struct layout changes
static const uint16_t LEGACY_MAGIC   = 0xB4D1;   // previous layout (fixed 5 s waits)
static const int      EEPROM_ADDR    = 0;

struct Stored {
  uint16_t magic;
  Settings s;
  uint8_t  checksum;
};

// Previous layout, read only to keep your tuned angles/distance on upgrade
struct LegacySettings {
  uint8_t  closedAngle;
  uint8_t  openAngle;
  uint16_t detectCm;
  uint32_t holdA;
  uint32_t holdB;
  uint32_t holdC;
};

struct LegacyStored {
  uint16_t magic;
  LegacySettings s;
  uint8_t  checksum;
};

static uint8_t calcChecksum(const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  uint8_t x = 0x5A;
  for (size_t i = 0; i < len; i++) x ^= (uint8_t)(p[i] + i);
  return x;
}

static bool basicsValid(uint8_t closedAngle, uint8_t openAngle, uint16_t detectCm) {
  return closedAngle <= 180 && openAngle <= 180 &&
         detectCm >= ULTRA_MIN_CM && detectCm <= ULTRA_MAX_CM;
}

static bool isValid(const Settings& s) {
  const uint32_t maxMs = (uint32_t)SETTINGS_MAX_TIME_S * 1000UL;
  return basicsValid(s.closedAngle, s.openAngle, s.detectCm) &&
         s.waitMinMs <= s.waitMaxMs && s.waitMaxMs <= maxMs;
}

void settingsDefaults() {
  g_settings.closedAngle = DEFAULT_CLOSED_ANGLE;
  g_settings.openAngle   = DEFAULT_OPEN_ANGLE;
  g_settings.detectCm    = DEFAULT_DETECT_CM;
  g_settings.waitMinMs   = DEFAULT_WAIT_MIN_MS;
  g_settings.waitMaxMs   = DEFAULT_WAIT_MAX_MS;
}

void settingsLoad() {
  Stored st;
  EEPROM.get(EEPROM_ADDR, st);
  if (st.magic == SETTINGS_MAGIC &&
      st.checksum == calcChecksum(&st.s, sizeof(Settings)) && isValid(st.s)) {
    g_settings = st.s;
    return;
  }

  // Upgrade from the previous layout: keep angles + distance, new default waits
  LegacyStored old;
  EEPROM.get(EEPROM_ADDR, old);
  settingsDefaults();
  if (old.magic == LEGACY_MAGIC &&
      old.checksum == calcChecksum(&old.s, sizeof(LegacySettings)) &&
      basicsValid(old.s.closedAngle, old.s.openAngle, old.s.detectCm)) {
    g_settings.closedAngle = old.s.closedAngle;
    g_settings.openAngle   = old.s.openAngle;
    g_settings.detectCm    = old.s.detectCm;
    settingsSave();          // rewrite in the new layout
  }
}

void settingsSave() {
  Stored st;
  st.magic = SETTINGS_MAGIC;
  st.s = g_settings;
  st.checksum = calcChecksum(&g_settings, sizeof(Settings));
  EEPROM.put(EEPROM_ADDR, st);   // only rewrites bytes that changed
}