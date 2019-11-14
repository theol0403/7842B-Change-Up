#include "7842Fmain/config.hpp"

void testAuton() {
  auto&& chassis = *Robot::chassis();

  // deploy lift
  pros::Task deploy(
    [](void*) {
      Robot::deploy();
    },
    nullptr, "deploy");

  chassis.strafeToPoint(
    {4_ft, 4.5_ft},
    [](const OdomController& odom) {
      if (odom.distanceToPoint({4_ft, 4.5_ft}) < 2_ft) {
        return lib7842::OdomMath::rollAngle180(-90_deg - Robot::odom()->getState().theta);
      } else {
        return 0_deg;
      }
    },
    1, OdomController::makeSettler(6_in));

  Robot::clawLeft()->setState(clawStates::release);
  Robot::clawRight()->setState(clawStates::release);

  chassis.strafeToPoint(
    {4_ft, 5.5_ft}, OdomController::makeAngleCalculator({4_in, 7_in}), 1,
    OdomController::defaultDriveAngleSettler);

  Robot::clawLeft()->setState(clawStates::clamp);
  Robot::clawRight()->setState(clawStates::clamp);
  pros::delay(300);

  Robot::lift()->setPosition({100, 100});
  Robot::lift()->setState(liftStates::hold);
  pros::delay(500);

  chassis.strafeToPoint({2.5_ft, 7.5_ft}, OdomController::makeAngleCalculator(-90_deg), 1);
}