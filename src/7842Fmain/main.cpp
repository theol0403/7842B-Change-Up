#include "main.h"
#include "config.hpp"
#include "driver.hpp"

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

IterativePosPIDController pid(0.02, 0, 0, 0, TimeUtilFactory().create());
IMU s(4);
void turn(QAngle a) {
  QAngle error = 0_deg;
  pid.controllerSet(0);
  do {
    error = util::rollAngle180(a - s.getRemapped(180, -180) * degree);
    double out = pid.step(-error.convert(degree));
    Robot::model()->tank(out, -out);
  } while (abs(error) >= 5_deg);
}
void initialize() {
  pros::delay(200);
  Robot::initialize();
  s.calibrate();
}

void autonomous() {
  // auto path = CubicBezier({{0_ft, 0_ft}, {1_ft, 0_ft}, {1_ft, 2_ft}, {2_ft, 2_ft}});
  turn(90_deg);
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
