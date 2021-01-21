#include "roller.hpp"

Roller::Roller(const std::shared_ptr<AbstractMotor>& iintakes,
               const std::shared_ptr<AbstractMotor>& ibottomRoller,
               const std::shared_ptr<AbstractMotor>& itopRoller,
               const std::shared_ptr<pros::ADIAnalogIn>& ilight) :
  intakes(iintakes), bottomRoller(ibottomRoller), topRoller(itopRoller), light(ilight) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Roller");
}

double Roller::getSensor() const {
  return light->get_value() - 2954 + 50;
}

void Roller::initialize() {
  light->calibrate();
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

      case rollerStates::off:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(0);
        break;

      case rollerStates::loading:
        if (getSensor() > 0) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(6000);
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(0);
        }
        break;

      case rollerStates::shoot:
        topRoller->moveVoltage(12000);
        intakes->moveVoltage(0);
        if (getSensor() > 0) {
          bottomRoller->moveVoltage(12000);
        } else {
          bottomRoller->moveVoltage(12000);
        }
        break;
    }

    if (time.repeat(100_ms)) { std::cout << getSensor() << std::endl; }

    rate.delayUntil(5_ms);
  }
}