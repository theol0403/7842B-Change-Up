#include "7842Fmain/auton.hpp"

void bigRedZone() {
  auto& chassis = *Robot::chassis();
  // TODO: measure position
  Robot::odom()->setState({2.2_ft, 1.8_ft, 0_deg});

  // push cube into corner
  chassis.strafeAbsoluteDirection(5_in, -90_deg, makeAngle(0_deg));

  // drive to first cube
  chassis.strafeToPoint(toClaw({outerProtectedCube, 0_deg}), makeAngle(0_deg));

  // spike cube
  pros::delay(1000);
}