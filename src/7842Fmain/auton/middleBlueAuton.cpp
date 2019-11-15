#include "7842Fmain/config.hpp"

void middleBlueAuton() {
  auto&& chassis = *Robot::chassis();
  Robot::odom()->setState({11_ft, 5_ft, -90_deg});

  // deploy lift
  pros::Task deploy(
    [](void*) {
      Robot::deploy();
      Robot::lift()->setPosition({0, 0});
      Robot::lift()->setState(liftStates::holdAtPos);

      Robot::clawLeft()->setState(clawStates::release);
      Robot::clawRight()->setState(clawStates::release);
    },
    nullptr, "deploy");

  // position to push the cubes
  Vector pushPoint {8_ft, 4_ft};
  // drive while turning to push cubes
  chassis.strafeToPoint(
    pushPoint,
    [&](const OdomController& odom) {
      // drive straight until halfway to point, then turn to face stack
      if (odom.distanceToPoint(pushPoint) < 2_ft) {
        return lib7842::OdomMath::rollAngle180(0_deg - Robot::odom()->getState().theta);
      } else {
        return 0_deg;
      }
    },
    1, OdomController::makeSettler(4_in));

  // allign with stack
  chassis.strafeToPoint(
    {8_ft, 5_ft}, OdomController::makeAngleCalculator(0_deg), 1,
    OdomController::defaultDriveAngleSettler);

  // close claw
  Robot::clawLeft()->setState(clawStates::clamp);
  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(500);

  // raise lift
  Robot::lift()->setPosition({300, 300});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(500);

  // drive in front of single cube
  chassis.strafeToPoint({9.5_ft, 7.5_ft}, OdomController::makeAngleCalculator(0_deg), 1);

  // push cube
  chassis.strafeToPoint({9_ft, 9.5_ft}, OdomController::makeAngleCalculator(0_deg), 1);

  // get different angle on cube
  chassis.strafeToPoint({8_ft, 10_ft}, OdomController::makeAngleCalculator(45_deg), 1);

  // push cube into corner
  chassis.strafeToPoint(
    {10.5_ft, 9.8_ft}, OdomController::makeAngleCalculator(45_deg), 1,
    OdomController::defaultDriveSettler);

  // open claw
  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);
  pros::delay(500);

  // back up and turn
  chassis.strafeToPoint(
    {9_ft, 10_ft}, OdomController::makeAngleCalculator(-180_deg), 1,
    OdomController::defaultDriveSettler);
}