#include "7842Fmain/auton.hpp"

void slowDown() {
  Robot::model()->setMaxVoltage(8000);
  Robot::model()->setMaxVelocity(100);
}

void speedUp() {
  Robot::model()->setMaxVoltage(12000);
  Robot::model()->setMaxVelocity(200);
}