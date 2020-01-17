#include "7842Fmain/auton.hpp"

void bigPreloadOuterProtected(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // TODO: measure position
  Robot::odom()->setState(mirror({9_in, 9.8_ft, 90_deg}, side));
  chassis.strafeAbsoluteDirection(5_in, 95_deg, makeAngle(90_deg));

  // deploy robot
  asyncTask(Robot::get().deploy(); Robot::lift()->goToPosition(Lift::aboveCubePos););
  Robot::claw()->setState(clawStates::clamp);

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, -20_deg, makeAngle(90_deg));

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
  chassis.strafeToPoint(toClaw({{1_ft - 1_in, 11.5_ft}, -45_deg}), makeAngle(-45_deg));
  spikeCube();
  Robot::claw()->setState(clawStates::open);
  pros::delay(800);
  Robot::claw()->setState(clawStates::brake);
  Robot::lift()->goToPosition(Lift::aboveCubePos + 100);
  pros::delay(300);
  Robot::claw()->setState(clawStates::open);

  chassis.strafeAbsoluteDirection(1_ft, 135_deg, makeAngle(-45_deg));
  Robot::claw()->setState(clawStates::brake);
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadOuterProtected(controller);

  bigGrabStack(controller);

  bigInnerProtectedScore(controller);
}