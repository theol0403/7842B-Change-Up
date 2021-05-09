#include "roller.hpp"
#include <bitset>
#include <iostream>

#define intake(v) intakeMotor->moveVoltage(v);
#define bottom(v) bottomMotor->moveVoltage(v);
#define top(v) topMotor->moveVoltage(v);
using rs = rollerStates;

void Roller::initialize() {
  topLight->setLedPWM(100);
  bottomLight->setLedPWM(100);
  // topMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
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

void Roller::runAction(const rs& action) {
  // mark action start time
  macroTime.placeMark();
  // clear prev action
  state &= ~rs::actions;
  // add new action
  state |= action;
}

bool Roller::shouldPoop() {
  // if poop is not enabled, don't do anything
  if (!(state & rs::poop)) return false;

  // if a blue is in the top, lower it before pooping
  if (getTopLight() == colors::blue && !(state & rs::top)) {
    runAction(rs::backPoop);
    return true;
  }

  // if blue ball in bottom but no red in top, poop it
  if (getBottomLight() == colors::blue && getTopLight() != colors::red && !(state & rs::top)) {
    runAction(rs::timedPoop);
    return true;
  }

  return false;
}

int Roller::getIntake() {
  return !!(state & rs::intake) ? 12000 : (!!(state & rs::out) ? -12000 : 0);
}

void Roller::loop() {
  Rate rate;

  while (true) {

    // strip flags from action. If action, execute it, and skip flags.
    if (auto action = state & rs::actions; action != rs::off) {
      switch (action) {

        case rs::deploy:
          top(12000);
          bottom(-800);
          intake(-12000);
          break;

        case rs::timedPoop:
          top(-8000);
          bottom(8000);
          intake(getIntake());
          if (macroTime.getDtFromMark() >= 100_ms) {
            runAction(rs::off);
            continue;
          }
          break;

        case rs::backPoop:
          top(-12000);
          bottom(-12000);
          intake(getIntake());
          if (macroTime.getDtFromMark() >= 100_ms) {
            runAction(rs::off);
            continue;
          }
          break;

        case rs::shootRev:
          top(-12000);
          bottom(-12000);
          intake(getIntake());
          break;

        default:
          std::cout << "Error: action not found: "
                    << std::bitset<sizeof(rs)>(static_cast<std::underlying_type_t<rs>>(state))
                    << std::endl;
          break;
      }
      continue;
    }

    // run auto poop if needed
    if (shouldPoop()) continue;

    // if out is enabled
    bool out = !!(state & rs::out);

    // strip toggles from state
    switch (auto toggles = state & rs::toggles; toggles) {

      case rs::off:
        top(out ? -12000 : 0);
        bottom(out ? -12000 : 0);
        intake(out ? -12000 : 0);
        break;

      case rs::top:
      case rs::topIntake:
        top(12000);
        bottom(out ? -12000 : 0);
        intake(getIntake());
        break;

      case rs::intake:
      case rs::loading:
        if (out) {
          top(-12000);
          bottom(-12000);
        } else if (getTopLight() != colors::none && getBottomLight() != colors::none) {
          top(0);
          bottom(-500);
        } else if (getTopLight() != colors::none) {
          // balance between raising ball to prevent rubbing and bringing ball too high
          top(0);
          bottom(5000);
        } else {
          // balance between bringing ball too fast and accidentally pooping
          top(7000);
          // if there is a blue but it can't poop
          bottom(12000);
        }
        intake(12000);
        break;

      case rs::shoot:
      case rs::on:
        // don't shoot a blue ball
        // if (getTopLight() == colors::blue) {
        //   //  move to intake mode
        //   state &= ~rs::shoot;
        //   top(-6000);
        //   pros::delay(100);
        //   continue;
        // } else {
        top(12000);
        // }
        // if red on the top needs to be separated from bottom ball
        if (getTopLight() == colors::red && getBottomLight() != colors::none) {
          if (getBottomLight() == colors::blue) {
            bottom(0);
          } else {
            bottom(2000);
          }
        } else {
          if (getBottomLight() == colors::blue) {
            bottom(2000);
          } else {
            bottom(12000);
          }
        }
        intake(getIntake());
        break;

      default:
        std::cout << "Error: toggle not found: "
                  << std::bitset<sizeof(rs)>(static_cast<std::underlying_type_t<rs>>(state))
                  << std::endl;
        break;
    }

    rate.delayUntil(5_ms);
  }
}
