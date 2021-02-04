#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class rollerStates {
  off, // all motors off
  loading, // run all util sensor, then bottom
  shoot, // run top until no sensor, then run top and bottom
  on, // all on
  poop, // poop
  out,
  deploy,
  purge,
  intakeOut,
  topOut,
};

class Roller : public StateMachine<rollerStates, rollerStates::off> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottomRoller,
         const std::shared_ptr<AbstractMotor>& itopRoller,
         const std::shared_ptr<pros::ADIAnalogIn>& itoplight,
         const std::shared_ptr<pros::ADIAnalogIn>& ibottomlight);

public:
  double getTopLight() const;
  double getBottomLight() const;

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottomRoller {nullptr};
  std::shared_ptr<AbstractMotor> topRoller {nullptr};
  std::shared_ptr<pros::ADIAnalogIn> topLight {nullptr};
  std::shared_ptr<pros::ADIAnalogIn> bottomLight {nullptr};
};