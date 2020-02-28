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
  while (Robot::lift()->getHeight() > 46) {
    pros::delay(10);
  }
  Robot::lift()->setState(liftStates::brakeAndHold);
}

void spikeFourStack() {
  // start lower
  Robot::claw()->setState(clawStates::off);
  Robot::lift()->setState(liftStates::down);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  // stagger
  while (Robot::lift()->getHeight() > 50) {
    if (std::abs(Robot::lift()->getLeftMotor()->getActualVelocity()) > 30) {
      Robot::lift()->setState(liftStates::down);
      Robot::model()->arcade(0.24, 0);
      pros::delay(500);
    } else {
      Robot::lift()->setState(liftStates::up);
      Robot::model()->arcade(-0.3, 0);
      pros::delay(500);
    }

    pros::delay(10);
  }

  Robot::lift()->setState(liftStates::off);
}