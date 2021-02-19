#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class rollerStates {
  off, // all off
  on, // all on
  onWithoutPoop, // all on
  out, // all backwards
  loading, // load balls into robot. Disable rollers one by one, and auto poop
  loadingWithoutPoop, // load balls into robot. Disable rollers one by one, and auto poop
  shoot, // all on but don't move intakes
  poop, // poop while intaking
  intakeOut, // poop while outtaking
  purge, // shoot while outtaking
  topOut, // only spin top roller
  deploy,
  timedPoop,
  poopOff, // poop while not intaking
};

class Roller : public StateMachine<rollerStates, rollerStates::off> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottomRoller,
         const std::shared_ptr<AbstractMotor>& itopRoller,
         const std::shared_ptr<pros::ADIAnalogIn>& itoplight,
         const std::shared_ptr<OpticalSensor>& icolor);

public:
  enum class colors { red, blue, none };

  double getTopLight() const;
  colors getColor() const;
  void shouldPoop();

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottomRoller {nullptr};
  std::shared_ptr<AbstractMotor> topRoller {nullptr};
  std::shared_ptr<pros::ADIAnalogIn> topLight {nullptr};
  std::shared_ptr<OpticalSensor> color {nullptr};
};