#include "claw.hpp"

Claw::Claw(
  const std::shared_ptr<Motor>& iclaw, const std::shared_ptr<IterativePosPIDController>& ipid) :
  claw(std::move(iclaw)), pid(std::move(ipid)) {
  initialize();
  startTask("claw");
}

std::shared_ptr<Motor> Claw::getMotor() const {
  return claw;
}

void Claw::initialize() {
  claw->setBrakeMode(AbstractMotor::brakeMode::brake);
  claw->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  startPos = claw->getPosition();
}

double Claw::getPosition() {
  return claw->getPosition() - startPos;
}

void Claw::loop() {

  while (true) {

    switch (state) {

      case clawStates::off: claw->moveVoltage(0); break;

      case clawStates::close: claw->moveVoltage(12000); break;

      case clawStates::open: claw->moveVoltage(-12000); break;

      case clawStates::clamp:
        pid->setTarget(-85);
        claw->moveVoltage(pid->step(getPosition()) * 12000);
        break;

      case clawStates::release:
        pid->setTarget(-340);
        claw->moveVoltage(pid->step(getPosition()) * 6000);
        break;

      case clawStates::brake: claw->moveVelocity(0); break;

      case clawStates::calibrate:
        do {
          claw->moveVoltage(8000);
          pros::delay(100);
        } while (claw->getActualVelocity() > 8);
        pros::delay(400);
        startPos = claw->getPosition();
        state = clawStates::clamp;
        break;

      case clawStates::hold:
        holdPos = getPosition();
        state = clawStates::holdAtPos;
        break;

      case clawStates::holdAtPos:
        pid->setTarget(holdPos);
        claw->moveVoltage(pid->step(getPosition()) * 12000);
        break;

      case clawStates::deploy:
        holdPos = -200;
        state = clawStates::holdAtPos;
        break;
    }

    // std::cout << "claw: " << claw->getPosition() << std::endl;
    pros::delay(10);
  }
}