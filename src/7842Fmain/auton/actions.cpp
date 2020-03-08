#include "7842Fmain/auton.hpp"

void slowDown() {
  Robot::model()->setMaxVoltage(9000);
  Robot::model()->setMaxVelocity(120);
}

void speedUp() {
  Robot::model()->setMaxVoltage(12000);
  Robot::model()->setMaxVelocity(200);
}

void spikeCube() {
  Robot::lift()->setState(liftStates::down);
  while (Robot::lift()->getHeight() > 45) {
    pros::delay(10);
  }
  Robot::lift()->setState(liftStates::brakeAndHold);
}

void spikeFourStack() {
  spikeFourStack(Timer());
}

void spikeFourStack(const Timer& timer) {
  // start lower
  Robot::claw()->setState(clawStates::off);
  Robot::lift()->setState(liftStates::down);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  // stagger
  while (Robot::lift()->getHeight() > 55 && timer.getDtFromMark() < 11_s) {
    if (std::abs(Robot::lift()->getLeftMotor()->getActualVelocity()) > 30) {
      Robot::lift()->setState(liftStates::down);
      Robot::model()->arcade(0.24, 0);
      pros::delay(500);
    } else {
      Robot::lift()->setState(liftStates::up);
      Robot::model()->arcade(-0.35, 0);
      pros::delay(500);
    }

    pros::delay(10);
  }

  Robot::lift()->setState(liftStates::off);
}