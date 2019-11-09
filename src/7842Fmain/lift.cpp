#include "lift.hpp"

Lift::Lift(
  std::unique_ptr<Motor>&& ileftLift,
  std::unique_ptr<Motor>&& irightLift,
  std::unique_ptr<IterativePosPIDController>&& ilpid,
  std::unique_ptr<IterativePosPIDController>&& irpid) :
  leftLift(std::move(ileftLift)),
  rightLift(std::move(irightLift)),
  lpid(std::move(ilpid)),
  rpid(std::move(irpid)) {
  calibrate();
  startTask("Lift");
}

double Lift::getLAngle() const {
  return leftLift->getPosition() - lstartAngle;
}

double Lift::getRAngle() const {
  return rightLift->getPosition() - rstartAngle;
}

double Lift::setLHoldPos(double iholdPos) {
  lholdPos = iholdPos;
}

double Lift::setRHoldPos(double iholdPos) {
  rholdPos = iholdPos;
}

void Lift::calibrate() {
  leftLift->setBrakeMode(AbstractMotor::brakeMode::coast);
  leftLift->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  rightLift->setBrakeMode(AbstractMotor::brakeMode::coast);
  rightLift->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  lstartAngle = leftLift->getPosition();
  rstartAngle = rightLift->getPosition();
}

void Lift::loop() {

  while (true) {

    switch (state) {

      case liftStates::off:
        leftLift->moveVoltage(0);
        rightLift->moveVoltage(0);
        break;

      case liftStates::holdCurrentPos:
        lholdPos = getLAngle();
        rholdPos = getRAngle();
        state = liftStates::holdAtPos;
        break;

      case liftStates::holdAtPos:
        lpid->setTarget(lholdPos);
        rpid->setTarget(rholdPos);
        leftLift->moveVoltage(lpid->step(getLAngle()) * 12000);
        rightLift->moveVoltage(rpid->step(getRAngle()) * 12000);
        break;

      case liftStates::up:
        leftLift->moveVoltage(12000);
        rightLift->moveVoltage(12000);
        break;

      case liftStates::down:
        leftLift->moveVoltage(-12000);
        rightLift->moveVoltage(-12000);
        break;

      case liftStates::bottom:
        lpid->setTarget(0);
        rpid->setTarget(0);
        leftLift->moveVoltage(lpid->step(getLAngle()) * 12000);
        rightLift->moveVoltage(rpid->step(getRAngle()) * 12000);
        break;
    }

    //std::cout << "Lift: " << getArmAngle() << std::endl;
    pros::delay(10);
  }
}