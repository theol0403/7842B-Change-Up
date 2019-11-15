#include "7842Fmain/config.hpp"

void testAuton() {
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

  chassis.strafeToPoint(
    {8_ft, 4_ft},
    [](const OdomController& odom) {
      if (odom.distanceToPoint({8_ft, 4_ft}) < 2_ft) {
        return lib7842::OdomMath::rollAngle180(0_deg - Robot::odom()->getState().theta);
      } else {
        return 0_deg;
      }
    },
    1, OdomController::makeSettler(6_in));

  chassis.strafeToPoint(
    {8_ft, 5_ft}, OdomController::makeAngleCalculator(0_deg), 1,
    OdomController::defaultDriveAngleSettler);

  Robot::clawLeft()->setState(clawStates::clamp);
  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(500);

  Robot::lift()->setPosition({300, 300});
  Robot::lift()->setState(liftStates::holdAtPos);
  pros::delay(500);

  chassis.strafeToPoint({9.5_ft, 7.5_ft}, OdomController::makeAngleCalculator(0_deg), 1);
}