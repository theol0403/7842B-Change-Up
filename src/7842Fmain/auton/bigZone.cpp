#include "7842Fmain/auton.hpp"

void bigZoneGrabStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = extractChassis(controller);

  // TODO: measure position
  Robot::odom()->setState(mirror({1_ft, 9.8_ft, 90_deg}, side));

  // deploy robot
  asyncTask(Robot::get().deploy(); Robot::lift()->goToPosition(Lift::aboveCubePos););
  Robot::claw()->setState(clawStates::clamp);

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, 0_deg, makeAngle(90_deg));

  // drive to first cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 90_deg}), makeAngle(90_deg));

  // spike cube and raise lift
  spikeCube();
  Robot::lift()->goToPosition(Lift::fourStackPos);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({fourStackCube, 90_deg}), makeAngle(90_deg));
  speedUp();

  // spike stack and raise lift a bit
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigZoneGrabProtectedAndScore(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = extractChassis(controller);

  // drive to inner protected cube
  chassis.strafeToPoint(toClaw({innerProtectedCube, 0_deg}), makeAngle(0_deg));

  // spike cube and raise lift
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);

  // score cube
  chassis.strafeToPoint(toClaw({{1_ft, 11.5_ft}, -45_deg}), makeAngle(-45_deg));
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = extractChassis(controller);

  bigZoneGrabStack(controller);

  bigZoneGrabProtectedAndScore(controller);
}