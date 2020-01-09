#include "7842Fmain/auton.hpp"

void bigZoneGrabStack(const std::shared_ptr<SideController>& controller) {
  extractChassis(controller); // add chassis and side to scope

  // TODO: measure position
  Robot::odom()->setState(mirror({1_ft, 9.8_ft, 90_deg}, side));

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, 0_deg, makeAngle(90_deg));

  // drive to first cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 90_deg}), makeAngle(90_deg));

  // spike cube and raise lift
  pros::delay(1000);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({fourStackCube, 90_deg}), makeAngle(90_deg));
  speedUp();

  // spike stack and raise lift a bit
  pros::delay(1000);
}

void bigZoneGrabProtectedAndScore(const std::shared_ptr<SideController>& controller) {
  extractChassis(controller); // add chassis and side to scope

  // drive to inner protected cube
  chassis.strafeToPoint(toClaw({innerProtectedCube, 0_deg}), makeAngle(0_deg));

  // spike cube and raise lift
  pros::delay(1000);

  // score cube
  chassis.strafeToPoint(toClaw({{0.5_ft, 11.5_ft}, -45_deg}), makeAngle(-45_deg));
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  extractChassis(controller); // add chassis and side to scope

  bigZoneGrabStack(controller);

  bigZoneGrabProtectedAndScore(controller);
}