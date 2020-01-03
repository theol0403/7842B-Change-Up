#include "intake.hpp"

Intake::Intake(
  const std::shared_ptr<AbstractMotor>& ileft, const std::shared_ptr<AbstractMotor>& iright) :
  left(ileft), right(iright) {
  initialize();
  startTask("Intake");
}

std::shared_ptr<AbstractMotor> Intake::getLeft() const {
  return left;
}

std::shared_ptr<AbstractMotor> Intake::getRight() const {
  return right;
}

void Intake::initialize() {
  left->setBrakeMode(AbstractMotor::brakeMode::brake);
  right->setBrakeMode(AbstractMotor::brakeMode::brake);
}

void Intake::loop() {

  while (true) {

    switch (state) {

      case intakeStates::off:
        left->moveVoltage(0);
        right->moveVoltage(0);
        break;

      case intakeStates::brake:
        left->moveVelocity(0);
        right->moveVelocity(0);
        break;

      case intakeStates::intake:
        left->moveVoltage(12000);
        right->moveVoltage(12000);
        break;

      case intakeStates::open:
        left->moveVoltage(-12000);
        right->moveVoltage(-12000);
        break;

      case intakeStates::intakeSlow:
        left->moveVelocity(150);
        right->moveVelocity(150);
        break;
    }

    pros::delay(10);
  }
}