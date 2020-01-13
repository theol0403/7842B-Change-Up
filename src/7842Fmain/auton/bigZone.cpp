#include "7842Fmain/auton.hpp"

void bigPreloadOuterProtected(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

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
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigGrabStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  Robot::lift()->goToPosition(Lift::fourStackPos);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({fourStackCube, 90_deg}), makeAngle(90_deg));
  speedUp();

  // spike stack and raise lift a bit
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigInnerProtectedScore(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // drive to inner protected cube
  chassis.strafeToPoint(toClaw({innerProtectedCube, 0_deg}), makeAngle(0_deg));

  // spike cube and raise lift
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);

  // score cube
  chassis.strafeToPoint(toClaw({{1_ft, 11.5_ft}, -45_deg}), makeAngle(-45_deg));
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadOuterProtected(controller);

  bigGrabStack(controller);

  bigInnerProtectedScore(controller);
}