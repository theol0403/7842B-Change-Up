#include "7842Fmain/config.hpp"

void testAuton() {
  auto&& chassis = *Robot::chassis();
  Robot::odom()->setState({11_ft, 10_ft, -180_deg});

  // deploy the claw
  Robot::deploy();
  // lower lift and open claw
  Robot::lift()->setPosition({0, 0});
  Robot::lift()->setState(liftStates::holdAtPos);
  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(400);
}