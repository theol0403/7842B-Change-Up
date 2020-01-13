#include "lift.hpp"

const double Lift::aboveCubePos {50};
const double Lift::fourStackPos {100};
const double Lift::smallTowerPos {100};
const double Lift::mediumTowerPos {150};
const double Lift::middleTowerPos {200};

Lift::Lift(const std::shared_ptr<AbstractMotor>& ileftMotor,
           const std::shared_ptr<AbstractMotor>& irightMotor,
           const std::shared_ptr<RotarySensor>& ileftSensor,
           const std::shared_ptr<RotarySensor>& irightSensor,
           const std::shared_ptr<IterativePosPIDController>& ilpid,
           const std::shared_ptr<IterativePosPIDController>& irpid) :
  motors({ileftMotor, irightMotor}), sensors({ileftSensor, irightSensor}), pids({ilpid, irpid}) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Lift");
}

void Lift::goToPosition(double ipos) {
  setPosition({ipos, ipos});
  state = liftStates::holdAtPos;
}

void Lift::setPosition(const std::valarray<double>& ipos) {
  holdPos = ipos;
}

std::valarray<double> Lift::getPosition() const {
  return std::valarray<double>(getRawPosition()) - startPos;
}

double Lift::getHeight() const {
  return getPosition().sum() / 2;
}

double Lift::getError() const {
  return std::abs(holdPos - getPosition()).sum() / 2;
}

std::shared_ptr<AbstractMotor> Lift::getLeftMotor() const {
  return motors[0];
}

std::shared_ptr<AbstractMotor> Lift::getRightMotor() const {
  return motors[1];
}

void Lift::setPowerWithBrake(const std::valarray<double>& power) {
  if (std::abs(power[0]) <= 0.05) {
    motors[0]->moveVelocity(0);
  } else {
    motors[0]->moveVoltage(power[0] * 12000);
  }

  if (std::abs(power[1]) <= 0.05) {
    motors[1]->moveVelocity(0);
  } else {
    motors[1]->moveVoltage(power[1] * 12000);
  }
}

std::valarray<double> Lift::getRawPosition() const {
  return {sensors[0]->get(), sensors[1]->get()};
}

void Lift::initialize() {
  motors[0]->setBrakeMode(AbstractMotor::brakeMode::brake);
  motors[0]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  motors[1]->setBrakeMode(AbstractMotor::brakeMode::brake);
  motors[1]->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  startPos = getRawPosition();
}

void Lift::loop() {

  Rate rate;

  while (true) {

    switch (state) {

      case liftStates::off:
        motors[0]->moveVoltage(0);
        motors[1]->moveVoltage(0);
        break;

      case liftStates::up:
        motors[0]->moveVoltage(12000);
        motors[1]->moveVoltage(12000);
        break;

      case liftStates::down:
        motors[0]->moveVoltage(-10000);
        motors[1]->moveVoltage(-10000);
        break;

      case liftStates::upSlow:
        motors[0]->moveVelocity(80);
        motors[1]->moveVelocity(80);
        break;

      case liftStates::downSlow:
        motors[0]->moveVelocity(-50);
        motors[1]->moveVelocity(-50);
        break;

      case liftStates::brake:
        motors[0]->moveVelocity(0);
        motors[1]->moveVelocity(0);
        holdPos = getPosition().sum() / 2.0;
        state = liftStates::holdAtPos;
        break;

      case liftStates::holdAtPos:
        pids[0]->setTarget(holdPos[0]);
        pids[1]->setTarget(holdPos[1]);
        setPowerWithBrake({pids[0]->step(getPosition()[0]), pids[1]->step(getPosition()[1])});
        break;

      case liftStates::aboveCube:
        holdPos = aboveCubePos;
        state = liftStates::holdAtPos;
        break;

      case liftStates::calibrate:
        do {
          motors[0]->moveVoltage(-4000);
          motors[1]->moveVoltage(-4000);
          pros::delay(200);
        } while (std::abs(motors[0]->getActualVelocity()) > 30 ||
                 std::abs(motors[1]->getActualVelocity()) > 30);
        pros::delay(200);
        startPos = getRawPosition();
        state = liftStates::off;
        break;
    }

    // if (timer.repeat(100_ms)) {
    //   std::cout << "L: " << getPosition()[0] << ", R: " << getPosition()[1] << std::endl;
    // }

    rate.delayUntil(3);
  }
}