#include "serial_cmd.h"

Command SerialCommand::poll() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ') continue;   // ignore line endings

    switch (tolower(ch)) {
      case 's': return Command::OPEN;
      case 'c': return Command::CLOSE;
      case 'r': return Command::RESET;
      case 'p': return Command::STATUS;
      case 'h': return Command::HEARTBEAT;
      case 'f': return Command::FAULT_TEST;
      case '?': return Command::HELP;
      default:  return Command::UNKNOWN;
    }
  }
  return Command::NONE;
}