#include "roller.hpp"

Roller::Roller(const std::shared_ptr<AbstractMotor>& iintakes,
               const std::shared_ptr<AbstractMotor>& ibottomRoller,
               const std::shared_ptr<AbstractMotor>& itopRoller,
               const std::shared_ptr<pros::ADIAnalogIn>& itopLight,
               const std::shared_ptr<OpticalSensor>& icolor) :
  intakes(iintakes),
  bottomRoller(ibottomRoller),
  topRoller(itopRoller),
  topLight(itopLight),
  color(icolor) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Roller");
}

Timer r;
double Roller::getTopLight() const {
  if (r.repeat(200_ms)) std::cout << topLight->get_value() - 2894 << std::endl;
  return topLight->get_value() - 2954 + 40;
}

Roller::colors Roller::getColor() const {
  // if (r.repeat(200_ms)) std::cout << color->getBrightness() << std::endl;
  if (color->getBrightness() > 0.02) return colors::none;

  int hue = color->getHue();
  // if (rate.repeat(200_ms)) { std::cout << "Hue: " << hue << std::endl; }

  // switch (hue) {
  //   case 150 ... 260: std::cout << "blue" << std::endl; return colors::blue;
  //   case 10 ... 40: std::cout << "red" << std::endl; return colors::red;
  //   default: std::cout << "none" << std::endl; return colors::none;
  // }

  switch (hue) {
    case 160 ... 250: return colors::blue;
    case 0 ... 40:
    case 340 ... 360: return colors::red;
    default: return colors::none;
  }
}

void Roller::initialize() {
  topLight->calibrate();
  // topRoller->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}

rollerStates backState = rollerStates::off;

Timer poopTime;
bool Roller::shouldPoop() {
  // if blue ball but none in top
  if (getColor() == colors::blue && getTopLight() >= 30) {
    poopTime.placeMark();
    backState = state;
    state = rollerStates::timedPoop;
    return true;
  }
  return false;
}

void Roller::loop() {
  Rate rate;

  while (true) {

    switch (state) {

      case rollerStates::off:
        // if blue ball but none in top
        if (getColor() == colors::blue && getTopLight() >= 0) {
          poopTime.placeMark();
          state = rollerStates::poopOff;
          continue;
        }

        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(0);
        break;

      case rollerStates::on:
        if (shouldPoop()) continue;
        if (getColor() == colors::blue && getTopLight() < 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(-1000);
          topRoller->moveVoltage(12000);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(12000);
        }
        break;

      case rollerStates::onWithoutPoop:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::out:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::loading:
        if (shouldPoop()) continue;
        if (getTopLight() < 0 && getColor() != colors::none) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(0);
          topRoller->moveVoltage(0);
        } else if (getTopLight() < 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(1000);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(12000);
        }
        break;

      case rollerStates::loadingWithoutPoop:
        if (getTopLight() < 0 && getColor() != colors::none) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(0);
          topRoller->moveVoltage(0);
        } else if (getTopLight() < 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVelocity(0);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(12000);
        }
        break;

      case rollerStates::shoot:
        if (shouldPoop()) continue;
        topRoller->moveVoltage(12000);
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(12000);
        break;

      case rollerStates::poop:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::intakeOut:
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
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::deploy:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-1000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::timedPoop:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        if (poopTime.getDtFromMark() >= 0.5_s) {
          poopTime.clearHardMark();
          state = backState;
        }
        break;

      case rollerStates::poopOff:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        if (poopTime.getDtFromMark() >= 0.5_s) {
          poopTime.clearHardMark();
          state = rollerStates::off;
        }
        break;
    }

    rate.delayUntil(5_ms);
  }
}