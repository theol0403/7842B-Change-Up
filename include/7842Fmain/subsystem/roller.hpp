#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class rollerStates {
  off, // all motors off
  loading, // run all util sensor, then bottom
  shoot, // run top until no sensor, then run top and bottom
  on, // all on
  poop, // poop
  preShoot,
  out,
  deploy,
  purge
};

class Roller : public StateMachine<rollerStates, rollerStates::off> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottomRoller,
         const std::shared_ptr<AbstractMotor>& itopRoller,
         const std::shared_ptr<pros::ADIAnalogIn>& ilight);

protected:
  double getSensor() const;

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottomRoller {nullptr};
  std::shared_ptr<AbstractMotor> topRoller {nullptr};
  std::shared_ptr<pros::ADIAnalogIn> light {nullptr};

  double lightDiff {0};
};