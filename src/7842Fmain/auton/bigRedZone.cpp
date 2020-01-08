#include "7842Fmain/auton.hpp"

void bigRedZone() {
  auto& chassis = *Robot::chassis();
  // TODO: measure position
  Robot::odom()->setState({2.2_ft, 1.8_ft, 0_deg});

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, -90_deg, makeAngle(0_deg));

  // drive to first cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 0_deg}), makeAngle(0_deg));

  // spike cube and raise lift
  pros::delay(1000);

  // set max voltage while lift is up
  Robot::model()->setMaxVoltage(8000);
  // drive to cube stack
  chassis.strafeToPoint(toClaw({fourStackCube, 0_deg}), makeAngle(0_deg));
  Robot::model()->setMaxVoltage(12000);

  // spike stack and raise lift a bit
  pros::delay(1000);

  // drive to inner protected cube
  chassis.strafeToPoint(toClaw({innerProtectedCube, -90_deg}), makeAngle(-90_deg), 1,
                        OdomController::defaultDriveAngleSettler);

  // spike cube and raise lift
  pros::delay(1000);

  // score cube
  chassis.strafeToPoint(toClaw({{0.5_ft, 1_ft}, -135_deg}), makeAngle(-135_deg), 1,
                        OdomController::defaultDriveAngleSettler);
}