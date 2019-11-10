#include "claw.hpp"

Claw::Claw(std::unique_ptr<Motor>&& iclaw, std::unique_ptr<IterativePosPIDController>&& ipid) :
  claw(std::move(iclaw)), pid(std::move(ipid)) {
  calibrate();
  startTask("claw");
}

void Claw::calibrate() {
  claw->setBrakeMode(AbstractMotor::brakeMode::brake);
  claw->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  startPos = claw->getPosition();
}

void Claw::loop() {

  while (true) {

    switch (state) {

      case clawStates::off: claw->moveVoltage(0); break;

      case clawStates::close: claw->moveVoltage(12000); break;

      case clawStates::open: claw->moveVoltage(-12000); break;

      case clawStates::clamp:
        pid->setTarget(50);
        claw->moveVoltage(pid->step(claw->getPosition()) * 12000);
        break;

      case clawStates::release:
        pid->setTarget(-200);
        claw->moveVoltage(pid->step(claw->getPosition()) * 12000);
        break;

      case clawStates::brake: claw->moveVelocity(0); break;
    }

    //std::cout << "claw: " << getArmAngle() << std::endl;
    pros::delay(10);
  }
}