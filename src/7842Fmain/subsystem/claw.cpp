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

  while (true) {

    switch (state) {

      case clawStates::off: claw->moveVoltage(0); break;

      case clawStates::brake: claw->moveVelocity(0); break;

      case clawStates::close: claw->moveVoltage(12000); break;

      case clawStates::open: claw->moveVoltage(-12000); break;

      case clawStates::clamp: claw->moveVoltage(8000); break;
    }

    pros::delay(10);
  }
}
