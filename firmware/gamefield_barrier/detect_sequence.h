#pragma once
#include <Arduino.h>
#include "servo_barrier.h"

// Runs the timed sequence when the ultrasonic detects an object:
//   barrier CLOSED -> wait a random time -> open -> stay open (DONE)
// The barrier is closed again only by reset or a manual close (knob / serial / ROS).
// Non-blocking: call update() every loop().
class DetectSequence {
public:
  enum class Action : uint8_t { NONE, OPEN, CLOSE };
  struct Step {
    Action action;           // command issued at the start of the step
    unsigned long dwellMs;   // wait AFTER the barrier has reached the target
  };

  void begin(ServoBarrier* barrier);
  void update(bool detected);
  void cancel();             // call on any manual/serial override
  bool active() const { return _active; }

private:
  void tryStart();
  void startStep(uint8_t idx);
  void finish(bool ok);

  ServoBarrier* _barrier = nullptr;
  Step          _buf[2];     // steps built when a sequence starts
  const Step*   _steps = nullptr;
  uint8_t       _count = 0;
  uint8_t       _idx = 0;
  bool          _active = false;
  bool          _armed = true;
  bool          _reached = false;
  unsigned long _stepStart = 0;
  unsigned long _dwellStart = 0;
};