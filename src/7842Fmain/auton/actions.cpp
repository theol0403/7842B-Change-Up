#include "7842Fmain/auton.hpp"

void slowDown() {
  Robot::model()->setMaxVoltage(9000);
  Robot::model()->setMaxVelocity(120);
}

void speedUp() {
  Robot::model()->setMaxVoltage(12000);
  Robot::model()->setMaxVelocity(200);
}
