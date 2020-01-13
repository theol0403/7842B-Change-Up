#include "7842Fmain/auton.hpp"

void testAuton(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  chassis.strafeToPoint({0_ft, 5_ft}, [&](const OdomController& odom) {
    if (odom.distanceToPoint({0_ft, 5_ft}) < 2.5_ft) {
      return util::rollAngle180(45_deg - Robot::odom()->getState().theta);
    } else {
      return util::rollAngle180(0_deg - Robot::odom()->getState().theta);
    }
  });

  chassis.strafeToPoint(toClaw({2_ft, 6_ft, 45_deg}), makeAngle(45_deg));

  auto path = SimplePath({Robot::odom()->getState(), {0_ft, 4_ft}, {0_ft, 0_ft}})
                .generate(1_cm)
                .smoothen(.01, 1e-8 * meter);

  Robot::follower()->followPath(PathGenerator::generate(path, defaultLimits), true);
}