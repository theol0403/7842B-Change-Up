#include "7842Fmain/auton.hpp"

void bigPreloadProtected(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // TODO: measure position
  Robot::odom()->setState(mirror({9_in, 9.8_ft, 90_deg}, side));

  // drive forward a bit
  chassis.strafeAbsoluteDirection(4_in, 100_deg, makeAngle(90_deg));

  // push cube into goal
  chassis.strafeAbsoluteDirection(6_in, 0_deg, makeAngle(90_deg));

  // go above inner protected
  chassis.strafeToPoint(toClaw({innerProtectedCube, 90_deg}), makeAngle(90_deg));

  // deploy robot
  asyncTask(Robot::get().deploy(); Robot::lift()->goToPosition(Lift::aboveCubePos););
  Robot::claw()->setState(clawStates::clamp);

  // wait for robot to deploy
  pros::delay(500);

  // grab inner cube
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);

  // drive to outer cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 90_deg}), makeAngle(90_deg));

  // spike cube
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
  Robot::lift()->goToPosition(Lift::aboveCubePos + 50);
}

void bigScore(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // score cube
  chassis.strafeToPoint(toClaw({{1_ft - 2_in, 11.5_ft}, -45_deg}), makeAngle(-45_deg));
  spikeCube();
  Robot::claw()->setState(clawStates::open);
  pros::delay(800);
  Robot::claw()->setState(clawStates::brake);
  Robot::lift()->goToPosition(Lift::aboveCubePos + 200);
  pros::delay(400);
  Robot::claw()->setState(clawStates::open);
  pros::delay(400);
  Robot::claw()->setState(clawStates::brake);

  chassis.strafeAbsoluteDirection(1_ft, 135_deg, makeAngle(-45_deg));
  Robot::claw()->setState(clawStates::brake);
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadProtected(controller);

  bigGrabStack(controller);

  bigScore(controller);
}