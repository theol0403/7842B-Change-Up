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
  while (Robot::lift()->getHeight() > 15) {
    pros::delay(20);
  }
  Robot::lift()->setState(liftStates::brake);
}