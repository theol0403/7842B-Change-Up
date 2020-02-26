#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

Timer timer;

void bigPreloadProtected(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  timer.placeMark();

  // TODO: measure position
  Robot::odom()->setState(mirror({9_in, 9.75_ft, 90_deg}, side));

  // drive forward a bit
  chassis.strafeAbsoluteDirection(4.5_in, 100_deg, makeAngle(90_deg), 1,
                                  Settler().distanceErr(2_in));

  // push cube into goal
  chassis.strafeAbsoluteDirection(9_in, -5_deg, makeAngle(90_deg), 1, Settler().distanceErr(2_in));

  // go above inner protected
  chassis.strafeToPoint(toClaw({innerProtectedCube, 90_deg}), makeAngle(90_deg));

  // deploy robot
  Robot::get().deploy();
  Robot::claw()->setState(clawStates::clamp);

  // grab inner protected cube
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
  pros::delay(1000);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({{2_tile + cubeHalf, 4_tile + cubeHalf}, 90_deg}),
                        makeAngle(90_deg));
  speedUp();

  // spike stack
  spikeFourStack();

  // ignore
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigScore(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // lift
  Robot::lift()->goToPosition(500);

  // move to goal
  speedUp();
  chassis.strafeToPoint(toClaw({{1_ft - 2.5_in, 11.5_ft + 0.5_in}, -45_deg}), makeAngle(-45_deg));
  pros::delay(500);

  Robot::claw()->setState(clawStates::open);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  // raise lift
  Robot::lift()->goToPosition(1000);
  pros::delay(100);

  // back up
  chassis.strafeAbsoluteDirection(1_ft, 135_deg, makeAngle(-45_deg));
  Robot::lift()->setState(liftStates::off);
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadProtected(controller);

  bigGrabStack(controller);

  bigScore(controller);
}

void bigZoneNoFourStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadProtected(controller);

  bigScore(controller);
}