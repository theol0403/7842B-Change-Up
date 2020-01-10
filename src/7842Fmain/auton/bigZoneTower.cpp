#include "7842Fmain/auton.hpp"

void bigZoneTower(const std::shared_ptr<SideController>& controller) {
  extractChassis(controller); // add chassis and side to scope

  bigZoneGrabStack(controller);

  Vector towerCube {9_ft, 6_ft - (towerBaseWidth + cubeHalf)};

  // drive to cube in front of tower
  chassis.strafeToPoint(toClaw({towerCube, 90_deg}), makeAngle(90_deg));

  // spike cube
  pros::delay(1000);

  // back up
  chassis.strafeToPoint({3.5_ft, 9_ft}, makeAngle(90_deg));

  // raise lift
  pros::delay(1000);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({leftTower, 90_deg}), makeAngle(90_deg));

  // open and close claw
  pros::delay(1000);

  // back up
  chassis.strafeToPoint({3.5_ft, 9_ft}, makeAngle(90_deg));

  speedUp();

  bigZoneGrabProtectedAndScore(controller);
}