#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

void bigPreloadProtected(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  // TODO: measure position
  Robot::odom()->setState(mirror({9_in, 9.75_ft, 90_deg}, side));

  // drive forward a bit
  chassis.strafeAbsoluteDirection(4.5_in, 100_deg, makeAngle(90_deg), 1, makeSettle(2_in));

  // push cube into goal
  chassis.strafeAbsoluteDirection(9_in, -10_deg, makeAngle(90_deg), 1, makeSettle(2_in));

  // go above inner protected
  chassis.strafeToPoint(toClaw({innerProtectedCube, 90_deg}), makeAngle(90_deg));

  // deploy robot
  asyncTask(Robot::get().deploy(); Robot::claw()->setState(clawStates::close););

  // wait for robot to deploy
  pros::delay(500);

  // grab inner protected cube
  spikeCube();
  Robot::claw()->setState(clawStates::clamp);
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
  chassis.strafeToPoint(toClaw({{2_tile + cubeHalf - 0.8_in, 4_tile + cubeHalf}, 90_deg}),
                        makeAngle(90_deg));
  speedUp();

  // spike stack and raise lift a bit

  // start lower
  Robot::claw()->setState(clawStates::off);
  Robot::lift()->setState(liftStates::down);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  // stagger
  while (Robot::lift()->getHeight() > 50) {
    if (std::abs(Robot::lift()->getLeftMotor()->getActualVelocity()) > 5) {
      Robot::lift()->setState(liftStates::down);
    } else {
      Robot::lift()->setState(liftStates::up);
      pros::delay(400);
    }
    pros::delay(10);
  }

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
  pros::delay(400);

  Robot::claw()->setState(clawStates::open);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  // raise lift
  Robot::lift()->goToPosition(2000);
  pros::delay(100);

  // back up
  chassis.strafeAbsoluteDirection(1_ft, 135_deg, makeAngle(-45_deg));
  Robot::lift()->setState(liftStates::brakeAndHold);
}

void bigZone(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadProtected(controller);

  bigGrabStack(controller);

  bigScore(controller);
}