#include "7842Fmain/config.hpp"

void farStackBlueAuton() {
  auto&& chassis = *Robot::chassis();
  Robot::odom()->setState({11_ft, 10_ft, -180_deg});

  // deploy lift
  pros::Task deploy(
    [](void*) {
      Robot::deploy();
      Robot::lift()->setPosition({250, 250});
      Robot::lift()->setState(liftStates::holdAtPos);

      Robot::clawLeft()->setState(clawStates::release);
      Robot::clawRight()->setState(clawStates::release);
    },
    nullptr, "deploy");

  // drive out
  chassis.strafeToPoint(
    {8.5_ft, 10.5_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveAngleSettler);

  // drive to stack
  chassis.strafeToPoint(
    {8.5_ft, 9.5_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveAngleSettler);

  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(400);

  Robot::lift()->setPosition({400, 400});
  pros::delay(400);
  chassis.strafeDistanceAtDirection(8_in, 90_deg);

  Robot::lift()->setPosition({-100, -100});
  pros::delay(1000);

  Robot::clawLeft()->setState(clawStates::clamp);
  pros::delay(400);

  Robot::lift()->setPosition({300, 300});
  pros::delay(500);

  // pdrive into corner
  chassis.strafeToPoint(
    {10.5_ft, 9.8_ft}, OdomController::makeAngleCalculator(45_deg), 1,
    OdomController::defaultDriveSettler);

  // open claw
  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(500);
}