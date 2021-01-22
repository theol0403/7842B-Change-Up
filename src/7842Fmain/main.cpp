#include "main.h"
#include "config.hpp"
#include "driver.hpp"

void disabled() {}
void competition_initialize() {}

ChassisScales scales({2.75_in, 21_in}, 360);
Limits limits(scales, 200_rpm, 0.6_s, 1, 1);
template <typename S> void follow(S&& path, bool forward) {
  auto trajectory = TrajectoryGenerator::generate(path, limits, 10_ms);
  TrajectoryGenerator::follow(*Robot::model(), trajectory, scales, 200_rpm, forward);
}

void drive(QLength m) {
  follow(Line({0_m, 0_m}, {0_m, abs(m)}), m >= 0_m);
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
  Robot::model()->tank(0, 0);
}
void initialize() {
  pros::delay(200);
  Robot::initialize();
  s.calibrate();
}

#define roller(x) Robot::roller()->setNewState(rollerStates::x)

void autonomous() {
  s.reset();

  roller(deploy);
  pros::delay(500);
  roller(loading);

  turn(55_deg);
  drive(1_ft);
  Robot::model()->xArcade(-1, 0, 0);
  pros::delay(100);
  Robot::model()->xArcade(0, 0, 0);

  roller(on);
  pros::delay(500);
  roller(loading);

  drive(-2_ft);
  turn(-100_deg);
  roller(poop);
  drive(4.2_ft);
  roller(loading);
  turn(130_deg);

  drive(4_ft);
  drive(-0.2_ft);

  roller(on);
  pros::delay(500);
  roller(loading);
  drive(-1_ft);

  roller(poop);
  turn(-135_deg);
  roller(loading);
  drive(4_ft);
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
