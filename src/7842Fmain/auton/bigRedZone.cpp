#include "7842Fmain/auton.hpp"

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto& chassis = *controller;
  auto& side = chassis.getSide();

  // TODO: measure position
  Robot::odom()->setState(mirror({1_ft, 9.8_ft, 90_deg}, side));

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, 0_deg, makeAngle(90_deg));

  // drive to first cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 90_deg}), makeAngle(90_deg));

  // spike cube and raise lift
  pros::delay(1000);

  // set max voltage while lift is up
  Robot::model()->setMaxVoltage(8000);
  // drive to cube stack
  chassis.strafeToPoint(toClaw({fourStackCube, 90_deg}), makeAngle(90_deg));
  Robot::model()->setMaxVoltage(12000);

  // spike stack and raise lift a bit
  pros::delay(1000);

  // drive to inner protected cube
  chassis.strafeToPoint(toClaw({innerProtectedCube, 0_deg}), makeAngle(0_deg), 1,
                        OdomController::defaultDriveAngleSettler);

  // spike cube and raise lift
  pros::delay(1000);

  // score cube
  chassis.strafeToPoint(toClaw({{0.5_ft, 11.5_ft}, -45_deg}), makeAngle(-45_deg), 1,
                        OdomController::defaultDriveAngleSettler);
}