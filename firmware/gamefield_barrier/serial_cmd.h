#pragma once
#include <Arduino.h>

enum class Command : uint8_t {
  NONE, OPEN, CLOSE, RESET, STATUS, HEARTBEAT, FAULT_TEST, HELP, UNKNOWN
};

// Single-character commands (easy from the Arduino IDE serial monitor):
//   s = open   c = close   r = reset   p = status
//   h = heartbeat   f = inject test fault   ? = help
class SerialCommand {
public:
  Command poll();   // returns NONE if no byte waiting
};