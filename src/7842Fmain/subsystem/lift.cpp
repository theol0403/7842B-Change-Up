#include "lift.hpp"

Lift::Lift(
  const std::shared_ptr<Motor>& ileftLift,
  const std::shared_ptr<Motor>& irightLift,
  const std::shared_ptr<Potentiometer>& ileftPot,
  const std::shared_ptr<Potentiometer>& irightPot,
  const std::shared_ptr<IterativePosPIDController>& ilpid,
  const std::shared_ptr<IterativePosPIDController>& irpid) :
  lift({std::move(ileftLift), std::move(irightLift)}),
  pid({std::move(ilpid), std::move(irpid)}),
  pot({std::move(ileftPot), std::move(irightPot)}) {
  pros::delay(100); // allow pots to initialize
  initialize();
  startTask("Lift");
}

void Lift::setPosition(const std::valarray<double>& ipos) {
  holdPos = ipos;
}

std::valarray<double> Lift::getPosition() const {
  return std::valarray<double>(getRawPosition()) - startPos;
}

double Lift::getError() const {
  return std::abs(holdPos - getPosition()).sum() / 2;
}

std::shared_ptr<Motor> Lift::getLeftMotor() const {
  return lift[0];
}

std::shared_ptr<Motor> Lift::getRightMotor() const {
  return lift[1];
}

std::valarray<double> Lift::getRawPosition() const {
  return {pot[0]->get(), pot[1]->get()};
}

void Lift::initialize() {
  lift[0]->setBrakeMode(AbstractMotor::brakeMode::brake);
  lift[0]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  lift[1]->setBrakeMode(AbstractMotor::brakeMode::brake);
  lift[1]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  startPos = getRawPosition();
}

void Lift::loop() {

  Timer timer;

  const QTime brakeTime = 300_ms;
  const int aboveCubePos = 300;

  while (true) {

    switch (state) {

      case liftStates::off:
        lift[0]->moveVoltage(0);
        lift[1]->moveVoltage(0);
        break;

      case liftStates::up:
        lift[0]->moveVoltage(12000);
        lift[1]->moveVoltage(12000);
        break;

      case liftStates::down:
        lift[0]->moveVoltage(-10000);
        lift[1]->moveVoltage(-10000);
        break;

      case liftStates::upSlow:
        lift[0]->moveVelocity(50);
        lift[1]->moveVelocity(50);
        break;

      case liftStates::downSlow:
        lift[0]->moveVelocity(-50);
        lift[1]->moveVelocity(-50);
        break;

      case liftStates::brake:
        lift[0]->moveVelocity(0);
        lift[1]->moveVelocity(0);
        timer.placeHardMark();
        if (timer.getDtFromHardMark() > brakeTime) {
          holdPos = getPosition().sum() / 2.0;
          state = liftStates::holdAtPos;
          timer.clearHardMark();
        }
        break;

      case liftStates::holdAtPos:
        pid[0]->setTarget(holdPos[0]);
        pid[1]->setTarget(holdPos[1]);
        lift[0]->moveVoltage(pid[0]->step(getPosition()[0]) * 12000);
        lift[1]->moveVoltage(pid[1]->step(getPosition()[1]) * 12000);
        break;

      case liftStates::aboveCube:
        holdPos = aboveCubePos;
        state = liftStates::holdAtPos;
        break;

      case liftStates::calibrate:
        do {
          lift[0]->moveVoltage(-12000);
          lift[1]->moveVoltage(-12000);
          pros::delay(400);
        } while (lift[0]->getActualVelocity() > 5 || lift[1]->getActualVelocity() > 5);
        pros::delay(400);
        startPos = getRawPosition();
        state = liftStates::off;
        break;
    }

    // if (timer.repeat(100_ms)) {
    //   std::cout << "L: " << getPosition()[0] << ", R: " << getPosition()[1] << std::endl;
    // }

    pros::delay(10);
  }
}