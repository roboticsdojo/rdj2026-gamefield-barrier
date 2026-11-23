#pragma once
#include <Arduino.h>

enum class CmdType : uint8_t {
  NONE, OPEN, CLOSE, RESET, STATUS, HEARTBEAT, FAULT_TEST, HELP,
  SET_DETECT_CM, SET_WAIT_MIN, SET_WAIT_MAX, SET_CLOSED_ANGLE, SET_OPEN_ANGLE,
  SHOW_SETTINGS, FACTORY_RESET, BAD_VALUE, UNKNOWN
};

struct Command {
  CmdType type;
  float   value;
};

// Single-letter commands run immediately:  s c r p h f ? v
// Commands with a number end with Enter (or 100 ms of silence):
//   d<cm>  t<sec>  m<sec>  a<deg>  o<deg>  x1
class SerialCommand {
public:
  Command poll();

private:
  Command finish();
  char          _buf[16];
  uint8_t       _len = 0;
  unsigned long _lastRx = 0;
};