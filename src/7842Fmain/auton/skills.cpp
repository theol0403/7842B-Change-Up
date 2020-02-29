#include "7842Fmain/auton.hpp"
using namespace lib7842::units;

void skills(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  bigZone7(controller);

  Robot::claw()->setState(clawStates::brake);
  Robot::lift()->goToPosition(Lift::aboveCubePos - 500);

  chassis.strafeToPoint(toClaw({closeTowerCube + Vector {3_in, 3_in}, 180_deg}),
                        makeAngle(180_deg));

  Robot::claw()->setState(clawStates::close);
  pros::delay(500);
  Robot::claw()->setState(clawStates::off);
  pros::delay(500);
  Robot::claw()->setState(clawStates::closeMedium);

  Robot::lift()->goToPosition(Lift::aboveCubePos);

  chassis.strafeAbsoluteDirection(1_ft, 0_deg, makeAngle(180_deg));

  Robot::lift()->goToPosition(2920);

  pros::delay(1800);

  slowDown();
  chassis.strafeToPoint(toClaw({closeTower + Vector {3_in, 6_in}, 180_deg}), makeAngle(180_deg));

  Robot::claw()->setState(clawStates::open);
  pros::delay(500);
  Robot::claw()->setState(clawStates::brake);

  chassis.strafeAbsoluteDirection(1_ft, 0_deg, makeAngle(180_deg));
  Robot::claw()->setState(clawStates::brake);

  speedUp();

  // tower 2

  Robot::lift()->goToPosition(Lift::aboveCubePos - 500);

  chassis.strafeToPoint(toClaw({leftTowerCube + Vector {-6_in, 3_in}, 90_deg}), makeAngle(90_deg));

  Robot::claw()->setState(clawStates::close);
  pros::delay(600);
  Robot::claw()->setState(clawStates::off);
  pros::delay(500);
  Robot::claw()->setState(clawStates::closeMedium);

  Robot::lift()->goToPosition(Lift::aboveCubePos);

  chassis.strafeAbsoluteDirection(1_ft, -90_deg, makeAngle(90_deg));

  Robot::lift()->goToPosition(2000);

  pros::delay(1800);

  slowDown();
  chassis.strafeToPoint(toClaw({leftTower + Vector {-1.5_ft, 1_ft}, 90_deg}), makeAngle(90_deg));

  Robot::claw()->setState(clawStates::open);
  pros::delay(1000);
  Robot::claw()->setState(clawStates::off);

  speedUp();
}