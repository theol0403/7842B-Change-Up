#include "claw.hpp"

Claw::Claw(const std::shared_ptr<AbstractMotor>& iclaw) : claw(iclaw) {
  initialize();
  startTask("Claw");
}

std::shared_ptr<AbstractMotor> Claw::getMotor() const {
  return claw;
}

void Claw::initialize() {
  claw->setBrakeMode(AbstractMotor::brakeMode::brake);
  claw->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

void Claw::loop() {

  Rate rate;

  while (true) {

    switch (state) {

      case clawStates::off:
        claw->setBrakeMode(AbstractMotor::brakeMode::brake);
        claw->moveVoltage(0);
        break;

      case clawStates::brake:
        claw->setBrakeMode(AbstractMotor::brakeMode::brake);
        claw->moveVelocity(0);
        break;

      case clawStates::close: claw->moveVoltage(12000); break;

      case clawStates::open: claw->moveVoltage(-12000); break;

      case clawStates::clamp:
        claw->setBrakeMode(AbstractMotor::brakeMode::coast);
        claw->moveVoltage(0);
        break;
    }

    rate.delayUntil(10);
  }
}
