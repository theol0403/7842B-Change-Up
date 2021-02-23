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
  topRoller->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  graph->withSeries("Bottom Sensor", LV_COLOR_BLACK, [&] { return bottomLight->getHue(); });
  graph->withSeries("Top Sensor", LV_COLOR_WHITE, [&] { return topLight->getHue(); });
}

Timer t;
Roller::colors Roller::getTopLight() const {
  if (t.repeat(200_ms))
    std::cout << "Hue: " << topLight->getHue() << ", Brightness: " << topLight->getBrightness()
              << ", Lightness: " << std::endl;

  if (topLight->getBrightness() > 0.02) return colors::none;

  int hue = topLight->getHue();
  switch (hue) {
    case 160 ... 250: return colors::blue;
    case 0 ... 40:
    case 340 ... 360: return colors::red;
    default: return colors::none;
  }
}

Roller::colors Roller::getBottomLight() const {
  if (bottomLight->getBrightness() > 0.02) return colors::none;

  int hue = bottomLight->getHue();
  switch (hue) {
    case 160 ... 250: return colors::blue;
    case 0 ... 40:
    case 340 ... 360: return colors::red;
    default: return colors::none;
  }
}

bool Roller::shouldPoop(int shouldIntake) {
  // if blue ball in bottom but no red in top
  if (getBottomLight() == colors::blue && getTopLight() != colors::red) {
    poopTime.placeMark();
    backState = state;
    shouldIntakePoopVel = shouldIntake;
    state = rollerStates::timedPoop;
    return true;
  }
  return false;
}

bool Roller::shouldShootPoop(int shouldIntake) {
  // if red ball in top but blue in bottom
  if (getBottomLight() == colors::blue && getTopLight() == colors::red) {
    poopTime.placeMark();
    backState = state;
    shouldIntakePoopVel = shouldIntake;
    state = rollerStates::timedShootPoop;
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
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(0);
        break;

      case rollerStates::out:
        if (shouldPoop(-12000)) continue;
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::on:
        if (shouldPoop()) continue;
        if (shouldShootPoop()) continue;
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::shoot:
        if (shouldPoop(0)) continue;
        if (shouldShootPoop(0)) continue;
        topRoller->moveVoltage(12000);
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(12000);
        break;

      case rollerStates::loading:
        if (shouldPoop()) continue;
        if (getTopLight() != colors::none && getBottomLight() != colors::none) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(0);
          topRoller->moveVoltage(0);
        } else if (getTopLight() == colors::red) {
          // will shoot blue if in bot
          intakes->moveVoltage(12000);
          // slow down
          bottomRoller->moveVoltage(8000);
          topRoller->moveVoltage(0);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(8000);
        }
        break;

      case rollerStates::poopIn:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::poopOut:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::purge:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::topOut:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(5000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::deploy:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-1000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::timedPoop:
        intakes->moveVoltage(shouldIntakePoopVel);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        if (poopTime.getDtFromMark() >= 0.3_s) {
          poopTime.clearHardMark();
          state = backState;
          continue;
        }
        break;

      case rollerStates::timedShootPoop:
        intakes->moveVoltage(shouldIntakePoopVel);
        bottomRoller->moveVoltage(-2000); // what happens if you full reverse
        topRoller->moveVoltage(12000);
        if (poopTime.getDtFromMark() >= 0.3_s) {
          poopTime.placeMark();
          state = rollerStates::timedPoop;
          continue;
        }
        break;
    }

    rate.delayUntil(5_ms);
  }
}