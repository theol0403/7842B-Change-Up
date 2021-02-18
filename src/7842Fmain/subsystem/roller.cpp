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

double Roller::getTopLight() const {
  return topLight->get_value() - 2954 + 50;
}

Roller::colors Roller::getColor() const {
  auto rgb = color->getRGB();
  std::cout << "Red: " << rgb.red << std::endl;
  std::cout << "Green: " << rgb.green << std::endl;
  std::cout << "Blue: " << rgb.blue << std::endl;
  std::cout << "Alpha: " << rgb.brightness << std::endl;

  if (rgb.red > 100) {
    return colors::red;
  } else if (rgb.blue > 100) {
    return colors::blue;
  } else {
    return colors::none;
  }
}

void Roller::initialize() {
  topLight->calibrate();
}

void Roller::loop() {
  Rate rate;

  while (true) {

    switch (state) {

      case rollerStates::off:
        intakes->moveVoltage(0);
        bottomRoller->moveVoltage(0);
        topRoller->moveVoltage(0);
        break;

      case rollerStates::on:
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
        // if no balls in intake
        if (getTopLight() >= 0 && getColor() == colors::none) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(12000);
          // if only top ball in intake
        } else if (getTopLight() < 0 && getColor() == colors::none) {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(12000);
          topRoller->moveVoltage(0);
          // else there is ball in top and bottom
        } else {
          intakes->moveVoltage(12000);
          bottomRoller->moveVoltage(0);
          topRoller->moveVoltage(0);
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
        bottomRoller->moveVoltage(-2000);
        topRoller->moveVoltage(12000);
        break;
    }

    rate.delayUntil(5_ms);
  }
}