#include "7842Fmain/config.hpp"

void testAuton() {
  auto&& chassis = *Robot::chassis();
  // Robot::odom()->setState({11_ft, 10_ft, -180_deg});

  // deploy the claw
  // Robot::deploy();

  // lower lift and open claw
  Robot::lift()->setPosition({250, 250});
  Robot::lift()->setState(liftStates::holdAtPos);
  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(1200);

  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(400);

  Robot::lift()->setPosition({400, 400});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(400);
  chassis.strafeDistanceAtDirection(8_in, 90_deg);

  Robot::lift()->setPosition({-100, -100});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(1200);

  Robot::clawLeft()->setState(clawStates::clamp);
  pros::delay(400);

  Robot::lift()->setPosition({300, 300});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(1000);
}