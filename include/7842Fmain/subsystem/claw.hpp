#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class clawStates {
  off, // motor 0 voltage
  brake, // motor 0 velocity
  close, // motor 100% voltage
  open // motor -100% voltage
};

class Claw : public StateMachine<clawStates, clawStates::brake> {

 public:
  Claw(const std::shared_ptr<Motor>& iclaw);

  std::shared_ptr<Motor> getMotor() const;

 protected:
  void initialize() override;
  void loop() override;

  std::shared_ptr<Motor> claw {nullptr};
};