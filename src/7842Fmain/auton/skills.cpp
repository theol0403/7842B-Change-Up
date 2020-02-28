#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

void skills(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigZone7(controller);

  Robot::claw()->setState(clawStates::brake);
  Robot::lift()->goToPosition(Lift::aboveCubePos - 500);

  chassis.strafeToPoint(toClaw({closeTowerCube + Vector {3_in, 2_in}, 180_deg}),
                        makeAngle(180_deg));

  Robot::claw()->setState(clawStates::close);
  pros::delay(500);
  Robot::claw()->setState(clawStates::off);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  Robot::lift()->goToPosition(Lift::aboveCubePos);

  chassis.strafeAbsoluteDirection(1_ft, 0_deg, makeAngle(180_deg));

  Robot::lift()->goToPosition(2900);

  pros::delay(1800);

  slowDown();
  chassis.strafeToPoint(toClaw({closeTower + Vector {3_in, 6_in}, 180_deg}), makeAngle(180_deg));

  Robot::claw()->setState(clawStates::open);
  pros::delay(1000);
  Robot::claw()->setState(clawStates::off);

  speedUp();
}