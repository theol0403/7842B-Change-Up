#include "main.h"
#include "config.hpp"
#include "driver.hpp"

void initialize() {
  pros::delay(200);
  Robot::initialize();
}

void disabled() {}
void competition_initialize() {}

ChassisScales scales({2.75_in, 21_in}, 360);
Limits limits(scales, 200_rpm, 0.5_s, 1, 1);
template <typename S> void follow(S&& path) {
  auto trajectory = TrajectoryGenerator::generate(path, limits, 10_ms);
  TrajectoryGenerator::follow(*Robot::model(), trajectory, scales, 200_rpm);
}

void distance(QLength m) {
  follow(Line({0_m, 0_m}, {0_m, m}));
}

void autonomous() {

  // auto path = CubicBezier({{0_ft, 0_ft}, {1_ft, 0_ft}, {1_ft, 2_ft}, {2_ft, 2_ft}});
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
