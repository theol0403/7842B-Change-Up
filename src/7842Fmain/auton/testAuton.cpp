#include "7842Fmain/auton.hpp"

void testAuton(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  Robot::odom()->setState(mirror({0_ft, 0_ft, 0_deg}, side));

  Robot::lift()->goToPosition(Lift::fourStackPos);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({0_ft, 3_ft, 0_deg}), makeAngle(0_deg));
  speedUp();

  // spike stack and raise lift a bit
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}