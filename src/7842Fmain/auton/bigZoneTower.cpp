#include "7842Fmain/auton.hpp"

void bigZoneTower(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadProtected(controller);

  bigGrabStack(controller);

  Vector towerCube {6_ft - (towerBaseWidth / 2 + cubeHalf), 9_ft};

  // drive to cube in front of tower
  chassis.strafeToPoint(toClaw({towerCube, 90_deg}), makeAngle(90_deg));

  // spike cube
  spikeCube();
  Robot::lift()->goToPosition(Lift::smallTowerPos);

  // back up
  chassis.strafeToPoint({3_ft, 9_ft}, makeAngle(90_deg));

  // wait for lift to raise
  pros::delay(1000);

  slowDown(); // set max voltage while lift is up
  // drive to cube stack
  chassis.strafeToPoint(toClaw({leftTower, 90_deg}), makeAngle(90_deg));

  // open and close claw
  Robot::claw()->setState(clawStates::open);
  pros::delay(500);
  Robot::claw()->setState(clawStates::close);

  // raise lift a bit
  Robot::lift()->goToPosition(Lift::smallTowerPos + 200);
  pros::delay(500);
  Robot::claw()->setState(clawStates::clamp);

  // back up
  chassis.strafeToPoint({3_ft, 9_ft}, makeAngle(0_deg));

  // lower lift
  Robot::lift()->goToPosition(Lift::aboveCubePos);

  speedUp();

  bigScore(controller);
}