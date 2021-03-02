#include "roller.hpp"

Roller::Roller(const std::shared_ptr<AbstractMotor>& iintakes,
               const std::shared_ptr<AbstractMotor>& ibottomRoller,
               const std::shared_ptr<AbstractMotor>& itopRoller,
               const std::shared_ptr<OpticalSensor>& itopLight,
               const std::shared_ptr<OpticalSensor>& ibottomLight,
               const std::shared_ptr<GUI::Graph>& igraph) :
  intakes(iintakes),
  bottomRoller(ibottomRoller),
  topRoller(itopRoller),
  topLight(itopLight),
  bottomLight(ibottomLight),
  graph(igraph) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Roller");
}

void Roller::initialize() {
  topLight->setLedPWM(100);
  bottomLight->setLedPWM(100);
  // topRoller->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  graph->withSeries("Bottom Sensor", LV_COLOR_BLACK, [&] { return bottomLight->getHue(); });
  graph->withSeries("Top Sensor", LV_COLOR_WHITE, [&] { return topLight->getHue(); });
}

Timer t;
Roller::colors Roller::getTopLight() const {
  // if (t.repeat(200_ms))
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

bool Roller::shouldPoop(int shouldIntake) {
  // if blue ball in bottom but no red in top
  if (getTopLight() != colors::red && getBottomLight() == colors::blue) {
    macroTime.placeMark();
    macroReturnState = state;
    macroIntakeVel = shouldIntake;
    state = rollerStates::timedPoop;
    return true;
  }

  if (getTopLight() == colors::blue) {
    macroTime.placeMark();
    macroReturnState = state;
    macroIntakeVel = shouldIntake;
    state = rollerStates::topPoop;
    return true;
  }
  return false;
}

bool Roller::shouldShootPoop(int shouldIntake) {
  // if red ball in top but blue in bottom
  if (getTopLight() == colors::red && getBottomLight() == colors::blue) {
    macroTime.placeMark();
    macroReturnState = state;
    macroIntakeVel = shouldIntake;
    state = rollerStates::timedShootPoop;
    return true;
  }
  return false;
}

bool Roller::shouldSpacedShoot(int shouldIntake) {
  // if double shot
  if (getTopLight() == colors::red && getBottomLight() == colors::red) {
    macroTime.placeMark();
    macroReturnState = state;
    macroIntakeVel = shouldIntake;
    state = rollerStates::spacedShoot;
    return true;
  }
  return false;
}

void Roller::loop() {
  Rate rate;

  while (true) {

    switch (state) {

      case rollerStates::off:
        if (shouldPoop(0)) continue;
        topRoller->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        intakes->moveVoltage(0);
        break;

      case rollerStates::out:
        // if (shouldPoop(-12000)) continue;
        topRoller->moveVoltage(-12000);
        bottomRoller->moveVoltage(-12000);
        intakes->moveVoltage(-12000);
        break;

      case rollerStates::on:
        if (shouldPoop()) continue;
        if (shouldShootPoop()) continue;
        [[fallthrough]];
      case rollerStates::onWithoutPoop:
        if (shouldSpacedShoot()) continue;
        topRoller->moveVoltage(12000);
        if (getBottomLight() == colors::blue && getTopLight() != colors::red) {
          bottomRoller->moveVoltage(-2000);
        } else {
          bottomRoller->moveVoltage(12000);
        }
        intakes->moveVoltage(12000);
        break;

      case rollerStates::shoot:
        if (shouldPoop(0)) continue;
        if (shouldShootPoop(0)) continue;
        [[fallthrough]];
      case rollerStates::shootWithoutPoop:
        if (shouldSpacedShoot(0)) continue;
        topRoller->moveVoltage(12000);
        if (getBottomLight() == colors::blue && getTopLight() != colors::red) {
          bottomRoller->moveVoltage(-2000);
        } else {
          bottomRoller->moveVoltage(12000);
        }
        intakes->moveVoltage(0);
        break;

      case rollerStates::intake:
        if (shouldPoop()) continue;
        [[fallthrough]];
      case rollerStates::intakeWithoutPoop:
        if (getTopLight() != colors::none && getBottomLight() != colors::none) {
          topRoller->moveVoltage(0);
          bottomRoller->moveVoltage(0);
          intakes->moveVoltage(12000);
        } else if (getTopLight() != colors::none) {
          // balance between raising ball to prevent rubbing and bringing ball too high
          topRoller->moveVoltage(2000);
          // slow down
          bottomRoller->moveVoltage(4000);
          intakes->moveVoltage(12000);
        } else {
          // balance between bringing ball too fast and accidentally pooping
          topRoller->moveVoltage(4000);
          if (getBottomLight() == colors::blue) {
            bottomRoller->moveVoltage(-3000);
          } else {
            bottomRoller->moveVoltage(12000);
          }
          intakes->moveVoltage(12000);
        }
        break;

      case rollerStates::poopIn:
        topRoller->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        intakes->moveVoltage(12000);
        break;

      case rollerStates::poopOut:
        topRoller->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        intakes->moveVoltage(-12000);
        break;

      case rollerStates::purge:
        topRoller->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        intakes->moveVoltage(-12000);
        break;

      case rollerStates::topOut:
        topRoller->moveVoltage(12000);
        bottomRoller->moveVoltage(5000);
        intakes->moveVoltage(0);
        break;

      case rollerStates::deploy:
        topRoller->moveVoltage(12000);
        bottomRoller->moveVoltage(-1000);
        intakes->moveVoltage(-12000);
        break;

      case rollerStates::timedPoop:
        topRoller->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        intakes->moveVoltage(macroIntakeVel);
        if (macroTime.getDtFromMark() >= 200_ms) {
          macroTime.clearMark();
          state = macroReturnState;
          continue;
        }
        break;

      case rollerStates::timedShootPoop:
        topRoller->moveVoltage(12000);
        bottomRoller->moveVoltage(0);
        intakes->moveVoltage(macroIntakeVel);
        if (macroTime.getDtFromMark() >= 400_ms) {
          macroTime.placeMark();
          state = rollerStates::timedPoop;
          continue;
        }
        break;

      case rollerStates::spacedShoot:
        topRoller->moveVoltage(12000);
        bottomRoller->moveVoltage(1000);
        intakes->moveVoltage(macroIntakeVel);
        if (macroTime.getDtFromMark() >= 10_ms) {
          macroTime.clearMark();
          state = macroReturnState;
          continue;
        }
        break;

      case rollerStates::topPoop:
        topRoller->moveVoltage(-12000);
        bottomRoller->moveVoltage(-12000);
        intakes->moveVoltage(macroIntakeVel);
        if (macroTime.getDtFromMark() >= 150_ms) {
          macroTime.placeMark();
          state = rollerStates::timedPoop;
          continue;
        }
        break;
    }

    rate.delayUntil(5_ms);
  }
}