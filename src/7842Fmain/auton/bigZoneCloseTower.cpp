#include "7842Fmain/auton.hpp"

void bigZoneCloseTower(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigPreloadOuterProtected(controller);

  Robot::lift()->goToPosition(Lift::mediumTowerPos);
  slowDown();

  // drive to cube in front of tower
  chassis.strafeToPoint(toClaw({closeTower, -180_deg}), makeAngle(180_deg));

  // open and close claw
  Robot::claw()->setState(clawStates::open);
  pros::delay(500);
  Robot::claw()->setState(clawStates::close);

  // raise lift a bit
  Robot::lift()->goToPosition(Lift::mediumTowerPos + 200);
  pros::delay(500);
  Robot::claw()->setState(clawStates::clamp);

  bigGrabStack(controller);

  bigInnerProtectedScore(controller);
}