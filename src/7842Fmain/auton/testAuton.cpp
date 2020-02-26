#include "7842Fmain/auton.hpp"

void testAuton(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  Settler::setDefaultAbort(TimeUtilFactory::withSettledUtilParams(2, 1, 1000_ms));

  chassis.strafeToPoint({1_ft, 2_ft});
  chassis.turnToAngle(90_deg);
  chassis.strafeToPoint({0_ft, 0_ft}, makeAngle(0_deg));

  // spikeFourStack();
}