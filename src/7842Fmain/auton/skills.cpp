#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

void skills(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigZone7(controller);

  Robot::claw()->setState(clawStates::closeMedium);

  chassis.strafeToPoint(toClaw({closeTowerCube, 180_deg}), makeAngle(180_deg));

  spikeCube();

  chassis.strafeAbsoluteDirection(1_ft, 0_deg, makeAngle(180_deg));

  Robot::lift()->goToPosition(3000);

  pros::delay(3000);

  chassis.strafeToPoint(toClaw({closeTower, 180_deg}), makeAngle(180_deg));
}