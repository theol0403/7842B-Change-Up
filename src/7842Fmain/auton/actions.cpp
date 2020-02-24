#include "7842Fmain/auton.hpp"

void slowDown() {
  Robot::model()->setMaxVoltage(8000);
  Robot::model()->setMaxVelocity(100);
}

void speedUp() {
  Robot::model()->setMaxVoltage(12000);
  Robot::model()->setMaxVelocity(200);
}

void spikeCube() {
  Robot::lift()->setState(liftStates::down);
  while (Robot::lift()->getHeight() > 40) {
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
  while (Robot::lift()->getHeight() > 100) {
    if (std::abs(Robot::lift()->getLeftMotor()->getActualVelocity()) > 5) {
      Robot::lift()->setState(liftStates::down);
      Robot::model()->arcade(0.2, 0);
    } else {
      Robot::lift()->setState(liftStates::up);
      Robot::model()->arcade(-0.4, 0);
      pros::delay(500);
    }

    pros::delay(10);
  }

  Robot::lift()->setState(liftStates::off);
}