#include "roller.hpp"

Roller::Roller(const std::shared_ptr<AbstractMotor>& iintakes,
               const std::shared_ptr<AbstractMotor>& ibottomRoller,
               const std::shared_ptr<AbstractMotor>& itopRoller,
               const std::shared_ptr<pros::ADIAnalogIn>& itopLight,
               const std::shared_ptr<pros::ADIAnalogIn>& ibottomLight) :
  intakes(iintakes),
  bottomRoller(ibottomRoller),
  topRoller(itopRoller),
  topLight(itopLight),
  bottomLight(ibottomLight) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Roller");
}

double Roller::getTopLight() const {
  return topLight->get_value() - 2954 + 50;
}

double Roller::getBottomLight() const {
  return bottomLight->get_value() - 2954 + 50;
}

void Roller::initialize() {
  topLight->calibrate();
  bottomLight->calibrate();
}

void Roller::loop() {
  Timer time;
  Rate rate;

  while (true) {

    switch (state) {

      case rollerStates::on:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(12000);
        break;
      case rollerStates::purge:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::deploy:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-2000);
        topRoller->moveVoltage(12000);
        break;

      case rollerStates::off:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(0);
        break;

      case rollerStates::loading:
        if (getTopLight() < 0 && getBottomLight() < 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(0);
          topRoller->moveVoltage(0);
        } else if (getTopLight() < 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(0);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(12000);
        }
        break;

      case rollerStates::shoot:
        topRoller->moveVoltage(12000);
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(12000);
        break;

      case rollerStates::poop:
        intakes->moveVoltage(12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::out:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(-12000);
        topRoller->moveVoltage(-12000);
        break;

      case rollerStates::intakeOut:
        intakes->moveVoltage(-12000);
        bottomRoller->moveVoltage(12000);
        topRoller->moveVoltage(-12000);
        break;
      case rollerStates::topOut:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(12000);
        break;
    }

    // if (time.repeat(100_ms)) { std::cout << getSensor() << std::endl; }

    rate.delayUntil(5_ms);
  }
}