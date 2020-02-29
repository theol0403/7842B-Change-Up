#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

Timer timer;

void bigPreloadInner(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // TODO: measure position
  Robot::odom()->setState(mirror({10_in, 9.8_ft, 90_deg}, side));

  bool deployed = false;
  // deploy robot
  asyncTask(pros::delay(1000); Robot::get().deploy(); Robot::claw()->setState(clawStates::clamp);
            deployed = true;);

  // push cube into goal
  util::strafeVector(Robot::model(), 1, 0, mirror(-90_deg, side));
  pros::delay(400);
  Robot::model()->tank(0, 0);

  // go above inner protected
  chassis.strafeToPoint(toClaw({innerProtectedCube, 90_deg}), makeAngle(90_deg));

  // wait for deploy
  while (!deployed)
    pros::delay(20);

  // grab inner protected cube
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigInnerToOuter(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // drive to outer cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 180_deg}), makeAngle(180_deg));

  // spike cube
  spikeCube();
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigInnerToStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // drive towards stack
  Robot::lift()->goToPosition(Lift::fourStackPos);
  chassis.strafeToPoint({3_ft, 10_ft}, makeAngle(180_deg), 1, Settler().distanceErr(4_in));

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({{2_tile + cubeHalf, 4_tile + cubeHalf + 4_in}, 180_deg}),
                        makeAngle(180_deg));
  speedUp();

  // spike stack
  spikeFourStack(timer);

  Robot::claw()->setState(clawStates::closeMedium);

  // ignore
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigOuterToStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // drive towards stack
  Robot::lift()->goToPosition(Lift::fourStackPos);
  pros::delay(1000);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({{2_tile + cubeHalf, 4_tile + cubeHalf + 4_in}, 180_deg}),
                        makeAngle(180_deg));
  speedUp();

  // spike stack
  spikeFourStack();

  Robot::claw()->setState(clawStates::closeMedium);

  // ignore
  Robot::lift()->goToPosition(Lift::aboveCubePos);
}

void bigScore(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // lift
  Robot::lift()->goToPosition(755);

  // move to goal
  speedUp();
  chassis.strafeToPoint({4_ft, 10_ft}, makeAngle(-90_deg), 1, Settler().distanceErr(4_in));
  chassis.strafeToPoint(toClaw({{1_ft - 1_in, 11.5_ft + 1_in}, -40_deg}), makeAngle(-40_deg));
  // pros::delay(500);

  Robot::claw()->setState(clawStates::openMedium);
  pros::delay(1000);
  Robot::claw()->setState(clawStates::brake);

  // // raise lift
  // Robot::lift()->goToPosition(1000);
  // pros::delay(100);

  // // back up
  chassis.strafeAbsoluteDirection(1_ft, 135_deg, makeAngle(-40_deg));
  Robot::lift()->setState(liftStates::off);
}

void bigZone6(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  timer.placeMark();

  bigPreloadInner(controller);

  bigInnerToStack(controller);

  bigScore(controller);
}

void bigZone7(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadInner(controller);

  bigInnerToOuter(controller);

  bigOuterToStack(controller);

  bigScore(controller);
}

void bigZoneNoFourStack(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadInner(controller);

  bigInnerToOuter(controller);

  bigScore(controller);
}