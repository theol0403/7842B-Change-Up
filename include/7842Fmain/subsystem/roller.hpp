#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class rollerStates {
  off, // all off
  offWithoutPoop, // all off
  out, // all backwards
  on, // all on
  onWithoutPoop, // all on without poop
  shoot, // all on but don't move intakes
  shootWithoutPoop, // all on but don't move intakes
  intake, // load balls into robot. Disable rollers one by one, and auto poop
  intakeWithoutPoop, // load balls into robot. Disable rollers one by one
  poopIn, // poop while intaking
  poopOut, // poop while outtaking
  purge, // shoot while outtaking
  topOut, // only spin top roller
  deploy,
  timedPoop,
  timedShootPoop,
  spacedShoot, // all on but space the ball and no intakes
  topPoop, // bring down then poop
};

class Roller : public StateMachine<rollerStates, rollerStates::off> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottomRoller,
         const std::shared_ptr<AbstractMotor>& itopRoller,
         const std::shared_ptr<OpticalSensor>& itoplight,
         const std::shared_ptr<OpticalSensor>& ibottomLight,
         const std::shared_ptr<GUI::Graph>& igraph);

public:
  enum class colors { none = 0, red, blue };

  colors getTopLight() const;
  colors getBottomLight() const;

  bool shouldPoop();
  bool shouldShootPoop();
  bool shouldSpacedShoot();

  int getIntake();

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottomRoller {nullptr};
  std::shared_ptr<AbstractMotor> topRoller {nullptr};
  std::shared_ptr<OpticalSensor> topLight {nullptr};
  std::shared_ptr<OpticalSensor> bottomLight {nullptr};
  std::shared_ptr<GUI::Graph> graph {nullptr};

  Timer macroTime;
  rollerStates macroReturnState = rollerStates::off;
  int macroIntakeVel = 12000;
};