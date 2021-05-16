#pragma once
#include "7842Fmain/util/enum.hpp"
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

// The first 3 bits toggle the state of each roller.
// The next few bits modify the behavior of the rollers.
// The last few bits enumerate additional actions.
// Bits can be combined to represent compound states.
enum class rollerStates {
  off = 0, // all off, no poop

  // toggle bits
  intake = 1 << 0, // move intakes
  bottom = 1 << 1, // move bottom roller
  top = 1 << 2, // move top roller

  // modifier bits
  out = 1 << 3, // reverse anything that is not enabled
  poop = 1 << 4, // enable auto poop

  // action bits
  deploy = 1 << 5,
  timedPoop = 2 << 5, // poop for time
  backPoop = 3 << 5, // bring blue down then poop
  shootRev = 4 << 5, // bring rollers back manually
  outSlow = 4 << 6, // outtake slowly
  forcePoop = 4 << 7, //
  intakeOnly = 4 << 8, //

  // state combinations
  loading = intake | bottom, // load balls into robot. Disable rollers one by one.
  shoot = bottom | top, // all on but don't move intakes
  on = intake | bottom | top, // all on
  topIntake = top | intake,

  compact = out | intake, // reverse top and botton rollers while intaking
  fill = out | shoot, // shoot top and bottom while outtaking
  purge = out | top, // shoot top while reversing rest

  loadingPoop = loading | poop,
  shootPoop = shoot | poop,
  onPoop = on | poop,
  outPoop = out | poop,

  toggles = 0b100000111, // bitmask for roller toggles
  modifiers = 0b100011000, // bitmask for roller modifiers
  flags = 0b100011111, // bitmask for flags
  actions = 0b1111100000, // bitmask for actions
};

ENUM_FLAG_OPERATORS(rollerStates)

class Roller : public StateMachine<rollerStates, rollerStates::poop> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakeMotor,
         const std::shared_ptr<AbstractMotor>& ibottomMotor,
         const std::shared_ptr<AbstractMotor>& itopMotor,
         const std::shared_ptr<OpticalSensor>& itopLight,
         const std::shared_ptr<OpticalSensor>& ibottomLight) :
    intakeMotor(iintakeMotor),
    bottomMotor(ibottomMotor),
    topMotor(itopMotor),
    topLight(itopLight),
    bottomLight(ibottomLight) {
    initialize();
  }

  void initialize();

  enum class colors { none, red, blue };
  colors targetColor {colors::red};
  colors garbageColor {colors::blue};

  colors getTopLight() const;
  colors getBottomLight() const;

  // set an action
  Timer macroTime;
  void runAction(const rollerStates& action);

  // modifier processors
  bool shouldPoop();
  int getIntake();

  void loop() override;

  std::shared_ptr<AbstractMotor> intakeMotor {nullptr};
  std::shared_ptr<AbstractMotor> bottomMotor {nullptr};
  std::shared_ptr<AbstractMotor> topMotor {nullptr};
  std::shared_ptr<OpticalSensor> topLight {nullptr};
  std::shared_ptr<OpticalSensor> bottomLight {nullptr};
};
