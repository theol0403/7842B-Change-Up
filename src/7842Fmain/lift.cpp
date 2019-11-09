#include "lift.hpp"

Lift::Lift(
  std::unique_ptr<Motor>&& ileftLift,
  std::unique_ptr<Motor>&& irightLift,
  std::unique_ptr<IterativePosPIDController>&& ilpid,
  std::unique_ptr<IterativePosPIDController>&& irpid) :
  lift({std::move(ileftLift), std::move(irightLift)}), pid({std::move(ilpid), std::move(irpid)}) {
  calibrate();
  startTask("Lift");
}

void Lift::setPosition(const std::valarray<double>& ipos) {
  holdPos = ipos;
}

std::valarray<double> Lift::getPosition() const {
  return std::valarray<double>({lift[0]->getPosition(), lift[1]->getPosition()}) - startPos;
}

void Lift::calibrate() {
  lift[0]->setBrakeMode(AbstractMotor::brakeMode::coast);
  lift[0]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  lift[1]->setBrakeMode(AbstractMotor::brakeMode::coast);
  lift[1]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  startPos = {lift[0]->getPosition(), lift[1]->getPosition()};
}

void Lift::loop() {

  while (true) {

    switch (state) {

      case liftStates::off:
        lift[0]->moveVoltage(0);
        lift[1]->moveVoltage(0);
        break;

      case liftStates::hold:
        holdPos = getPosition();
        state = liftStates::holdAtPos;
        break;

      case liftStates::holdAtPos:
        pid[0]->setTarget(holdPos[0]);
        pid[1]->setTarget(holdPos[1]);
        lift[0]->moveVoltage(pid[0]->step(getPosition()[0]) * 12000);
        lift[1]->moveVoltage(pid[1]->step(getPosition()[1]) * 12000);
        break;

      case liftStates::up:
        lift[0]->moveVoltage(12000);
        lift[1]->moveVoltage(12000);
        break;

      case liftStates::down:
        lift[0]->moveVoltage(-12000);
        lift[1]->moveVoltage(-12000);
        break;

      case liftStates::bottom:
        pid[0]->setTarget(0);
        pid[1]->setTarget(0);
        lift[0]->moveVoltage(pid[0]->step(getPosition()[0]) * 12000);
        lift[1]->moveVoltage(pid[1]->step(getPosition()[1]) * 12000);
        break;
    }

    //std::cout << "Lift: " << getArmAngle() << std::endl;
    pros::delay(10);
  }
}