#pragma once
#include <Arduino.h>
#include <Servo.h>

enum class BarrierState : uint8_t {
  UNKNOWN,
  INITIALIZING,
  OPENING,
  OPEN,
  CLOSING,
  CLOSED,
  FAULT
};

enum class FaultCode : uint8_t {
  NONE,
  COMMS_LOSS,
  TEST        // injected from the serial monitor
};

class ServoBarrier {
public:
  void begin();
  void update();                 // call every loop()

  // force = manual knob override: also works while in FAULT (state stays FAULT)
  bool open(bool force = false);
  bool close(bool force = false);

  void reset();                  // clear fault, re-initialize to CLOSED
  void fault(FaultCode code);    // enter FAULT, move to SAFE_ANGLE

  BarrierState state() const { return _state; }
  FaultCode    faultCode() const { return _fault; }
  int          angle() const { return (int)(_angle + 0.5f); }

  static const char* stateName(BarrierState s);
  static const char* faultName(FaultCode f);

private:
  Servo        _servo;
  BarrierState _state = BarrierState::UNKNOWN;
  FaultCode    _fault = FaultCode::NONE;
  float        _angle = 0;
  int          _target = 0;
  bool         _arrived = false;
  unsigned long _arrivedAt = 0;
  unsigned long _lastStep = 0;
};

// servo - D9
// pot pin - A0
// status led - built in
// trig - D7
// echo -D8