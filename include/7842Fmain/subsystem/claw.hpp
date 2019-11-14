#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class clawStates {
  off, // motor 0 voltage
  close, // motor 100% voltage
  open, // motor -100% voltage
  clamp, // pid to close position
  release, // pid to open position
  brake, // apply motor brake
  calibrate, // apply motor brake
  hold,
  holdAtPos
};

class Claw : public StateMachine<clawStates, clawStates::brake> {

 public:
  Claw(const std::shared_ptr<Motor>& iclaw, const std::shared_ptr<IterativePosPIDController>& ipid);

  std::shared_ptr<Motor> getMotor() const;

 protected:
  void initialize() override;
  void loop() override;

  double getPosition();

  std::shared_ptr<Motor> claw {nullptr};
  std::shared_ptr<IterativePosPIDController> pid {nullptr};

  double startPos {0};
  double holdPos {0};
};