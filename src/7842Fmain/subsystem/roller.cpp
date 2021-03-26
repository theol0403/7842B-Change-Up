#include "roller.hpp"

#define intake(v) intakeMotor->moveVoltage(v);
#define bottom(v) bottomMotor->moveVoltage(v);
#define top(v) topMotor->moveVoltage(v);

void Roller::initialize() {

  topLight->setLedPWM(100);
  bottomLight->setLedPWM(100);
  // top->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  startTask("Roller");
}

Roller::colors Roller::getTopLight() const {
  // static Timer t;
  // if (t.repeat(300_ms))
  //   std::cout << "Hue: " << topLight->getHue() << ", Brightness: " << topLight->getBrightness()
  //             << ", Proximity: " << topLight->getProximity() << std::endl;

  if (topLight->getProximity() < 100) return colors::none;

  int hue = topLight->getHue();
  switch (hue) {
    case 160 ... 300: return colors::blue;
    case 0 ... 40:
    case 350 ... 360: return colors::red;
    default: return colors::none;
  }
}

Roller::colors Roller::getBottomLight() const {
  // static Timer t;
  // if (t.repeat(300_ms))
  //   std::cout << "Hue: " << bottomLight->getHue()
  //             << ", Brightness: " << bottomLight->getBrightness()
  //             << ", Proximity: " << bottomLight->getProximity() << std::endl;

  if (bottomLight->getProximity() < 110) return colors::none;

  int hue = bottomLight->getHue();
  switch (hue) {
    case 180 ... 300: return colors::blue;
    case 0 ... 40:
    case 350 ... 360: return colors::red;
    default: return colors::none;
  }
}

void Roller::runAction(const rollerStates& action) {
  // mark action start time
  macroTime.placeMark();
  // clear prev action
  state &= ~rollerStates::actions;
  // add new action
  state |= action;
}

bool Roller::shouldPoop() {
  // if poop is not enabled, don't do anything
  if (!(state & rollerStates::poop)) return false;

  // if a blue is in the top, lower it before pooping
  if (getTopLight() == colors::blue) {
    runAction(rollerStates::backPoop);
    return true;
  }

  // if blue ball in bottom but no red in top, poop it
  if (getBottomLight() == colors::blue && getTopLight() != colors::red) {
    runAction(rollerStates::timedPoop);
    return true;
  }

  return false;
}

int Roller::getIntake() {
  return !!(state & rollerStates::intake) ? 12000 : (!!(state & rollerStates::out) ? -12000 : 0);
}

void Roller::loop() {
  Rate rate;

  while (true) {

    // strip flags from action. If action, execute it, and skip flags.
    if (auto action = state & rollerStates::actions; action != rollerStates::off) {
      switch (action) {

        case rollerStates::deploy:
          top(12000);
          bottom(-800);
          intake(-12000);
          break;

        case rollerStates::timedPoop:
          top(-12000);
          bottom(12000);
          intake(getIntake());
          if (macroTime.getDtFromMark() >= 200_ms) {
            runAction(rollerStates::off);
            continue;
          }
          break;

        case rollerStates::backPoop:
          top(-12000);
          bottom(-12000);
          intake(getIntake());
          if (macroTime.getDtFromMark() >= 150_ms) {
            runAction(rollerStates::off);
            continue;
          }
          break;

        case rollerStates::shootRev:
          top(-12000);
          bottom(-12000);
          intake(getIntake());
          break;

        default: break;
      }
      continue;
    }

    // if out is enabled
    bool out = !!(state & rollerStates::out);

    switch (auto toggles = state & rollerStates::toggles; toggles) {

      case rollerStates::off:
        if (shouldPoop()) continue;
        top(out ? -12000 : 0);
        bottom(out ? -12000 : 0);
        intake(out ? -12000 : 0);
        break;

      case rollerStates::top:
        if (shouldPoop()) continue;
        top(12000);
        bottom(out ? -12000 : 0);
        intake(out ? -12000 : 0);
        break;

      case rollerStates::intake:
        if (shouldPoop()) continue;
        if (out) {
          top(-12000);
          bottom(-12000);
        } else if (getTopLight() != colors::none && getBottomLight() != colors::none) {
          top(0);
          if (getBottomLight() == colors::blue) {
            bottom(-2000);
          } else {
            bottom(0);
          }
        } else if (getTopLight() != colors::none) {
          // balance between raising ball to prevent rubbing and bringing ball too high
          top(1800);
          bottom(6000);
        } else {
          // balance between bringing ball too fast and accidentally pooping
          top(4000);
          // if there is a blue but it can't poop
          if (getBottomLight() == colors::blue) {
            bottom(0);
          } else {
            bottom(12000);
          }
        }
        intake(getIntake());
        break;

      case rollerStates::shoot:
      case rollerStates::on:
        if (shouldPoop()) continue;
        // don't shoot a blue ball
        if (getTopLight() != colors::blue) {
          top(12000);
        } else {
          top(-1000);
        }
        // if red on the top needs to be separated from bottom ball
        if (getTopLight() == colors::red && getBottomLight() != colors::none) {
          if (getBottomLight() == colors::blue) {
            bottom(0);
          } else {
            bottom(2000);
          }
        } else {
          bottom(12000);
        }
        intake(getIntake());
        break;

      default: break;
    }

    rate.delayUntil(5_ms);
  }
}