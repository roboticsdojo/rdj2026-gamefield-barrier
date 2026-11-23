#include "serial_cmd.h"
#include <ctype.h>
#include <stdlib.h>

static const unsigned long IDLE_MS = 100;

static Command make(CmdType t, float v = 0) {
  Command c;
  c.type = t;
  c.value = v;
  return c;
}

static bool isImmediate(char ch) {
  switch (ch) {
    case 's': case 'c': case 'r': case 'p': case 'h':
    case 'f': case '?': case 'v':
      return true;
  }
  return false;
}

Command SerialCommand::poll() {
  while (Serial.available() > 0) {
    char ch = (char)tolower(Serial.read());
    _lastRx = millis();

    if (ch == ' ') continue;
    if (ch == '\r' || ch == '\n') {
      if (_len > 0) return finish();
      continue;
    }

    if (_len < sizeof(_buf) - 1) _buf[_len++] = ch;
    if (_len == 1 && isImmediate(ch)) return finish();
  }

  // No Enter sent (serial monitor set to "No line ending"): run after a short pause
  if (_len > 0 && millis() - _lastRx >= IDLE_MS) return finish();
  return make(CmdType::NONE);
}

Command SerialCommand::finish() {
  _buf[_len] = '\0';
  char cmd = _buf[0];
  const char* arg = _buf + 1;
  _len = 0;

  switch (cmd) {
    case 's': return make(CmdType::OPEN);
    case 'c': return make(CmdType::CLOSE);
    case 'r': return make(CmdType::RESET);
    case 'p': return make(CmdType::STATUS);
    case 'h': return make(CmdType::HEARTBEAT);
    case 'f': return make(CmdType::FAULT_TEST);
    case '?': return make(CmdType::HELP);
    case 'v': return make(CmdType::SHOW_SETTINGS);

    case 'd': case 't': case 'm': case 'a': case 'o': case 'x': {
      char* end;
      double v = strtod(arg, &end);
      if (end == arg || *end != '\0') return make(CmdType::BAD_VALUE);
      CmdType type = CmdType::UNKNOWN;
      if (cmd == 'd') type = CmdType::SET_DETECT_CM;
      if (cmd == 't') type = CmdType::SET_WAIT_MIN;
      if (cmd == 'm') type = CmdType::SET_WAIT_MAX;
      if (cmd == 'a') type = CmdType::SET_CLOSED_ANGLE;
      if (cmd == 'o') type = CmdType::SET_OPEN_ANGLE;
      if (cmd == 'x') type = CmdType::FACTORY_RESET;
      return make(type, (float)v);
    }
  }
  return make(CmdType::UNKNOWN);
}