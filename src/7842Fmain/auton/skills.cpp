#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

void skills(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigZone7(controller);

  Robot::claw()->setState(clawStates::closeMedium);
  Robot::lift()->goToPosition(Lift::aboveCubePos + 100);
  pros::delay(500);

  chassis.strafeToPoint(toClaw({closeTowerCube, 180_deg}), makeAngle(180_deg));

  spikeCube();

  chassis.strafeAbsoluteDirection(1_ft, 0_deg, makeAngle(180_deg));

  Robot::lift()->goToPosition(3000);

  pros::delay(3000);

  chassis.strafeToPoint(toClaw({closeTower + Vector {0_ft, 6_in}, 180_deg}), makeAngle(180_deg));

  Robot::claw()->setState(clawStates::open);
  pros::delay(2000);
  Robot::claw()->setState(clawStates::off);
}