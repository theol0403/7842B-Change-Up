#include "7842Fmain/config.hpp"

void farRedAuton() {
  auto&& chassis = *Robot::chassis();
  Robot::odom()->setState({1_ft, 10_ft, -180_deg});

  // deploy the claw
  Robot::deploy();
  // lower lift and open claw
  Robot::lift()->setPosition({0, 0});
  Robot::lift()->setState(liftStates::holdAtPos);
  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(400);

  // allign claw to cube
  chassis.strafeToPoint(
    {2_ft, 9.7_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveSettler);
  // close claw
  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(300);

  // raise lift
  Robot::lift()->setPosition({550, 550});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(600);

  // drive to tower
  chassis.strafeToPoint(
    {1.4_ft, 7.5_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveAngleSettler);

  // open claw
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(400);

  // back up
  chassis.strafeToPoint(
    {4_ft, 8_ft}, OdomController::makeAngleCalculator(), 1, OdomController::defaultDriveSettler);

  // lower lift
  Robot::lift()->setPosition({0, 0});
  Robot::lift()->setState(liftStates::holdAtPos);

  // strafe to behind cube while lowering lift
  chassis.strafeToPoint(
    {4.2_ft, 8.2_ft}, OdomController::makeAngleCalculator(-45_deg), 2,
    OdomController::defaultDriveAngleSettler);

  // push cube into corner
  chassis.strafeToPoint(
    {1.5_ft, 9.8_ft}, OdomController::makeAngleCalculator(-45_deg), 1,
    OdomController::defaultDriveSettler);

  // back up and turn
  chassis.strafeToPoint(
    {3_ft, 10_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveSettler);
}