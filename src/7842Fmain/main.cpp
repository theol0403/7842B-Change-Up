#include "main.h"
#include "config.hpp"
#include "driver.hpp"

#define mDigital(x)                                                                                \
  (pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_##x) ||   \
   pros::c::controller_get_digital(pros::E_CONTROLLER_PARTNER, pros::E_CONTROLLER_DIGITAL_##x))

void disabled() {}
void competition_initialize() {}

ChassisScales scales({2.75_in, 21_in}, 360);
Limits limits(scales, 200_rpm, 0.6_s, 1, 1);
template <typename S> void follow(S&& path, bool forward = true) {
  auto trajectory = TrajectoryGenerator::generate(path, limits, 10_ms);
  TrajectoryGenerator::follow(*Robot::model(), trajectory, scales, 200_rpm, forward);
}

#define asyncTask(x) pros::Task([&]() { x });

void drive(QLength m) {
  follow(Line({0_m, 0_m}, {0_m, abs(m)}), m >= 0_m);
}

IterativePosPIDController pid(0.024, 0, 0, 0, TimeUtilFactory().create());
IMU s(4);
void turn(QAngle a) {
  QAngle error = 0_deg;
  pid.controllerSet(0);
  do {
    error = util::rollAngle180(a - s.getRemapped(180, -180) * degree);
    double out = pid.step(-error.convert(degree));
    Robot::model()->tank(out, -out);
  } while (abs(error) >= 3_deg);
  Robot::model()->tank(0, 0);
}
void initialize() {
  pros::delay(200);
  Robot::initialize();
  s.calibrate();
  pros::delay(1000);
}

#define roll(x) Robot::roller()->setNewState(rollerStates::x)

void cornerGoal() {
  roll(loading);
  pros::delay(600);
  roll(on);
  pros::delay(400);
  roll(poop);
}

void autonomous() {
  s.reset();

  // deploy
  roll(deploy);
  pros::delay(500);
  roll(loading);

  // 1 to first corner goal
  turn(58_deg);
  drive(1.1_ft);
  Robot::model()->xArcade(-1, 0, 0.5);
  pros::delay(100);
  Robot::model()->xArcade(0, 0, 0);
  // 1 shoot first corner goal
  cornerGoal();
  roll(loading);

  // back up
  Robot::model()->setMaxVoltage(5500);
  drive(-2.5_ft);
  Robot::model()->setMaxVoltage(12000);
  roll(purge);
  pros::delay(500);
  // to ball
  turn(-103_deg);
  roll(loading);
  drive(4.90_ft);

  // 2 to first edge goal
  turn(134_deg);
  drive(3.8_ft);
  // 2 shoot first edge goal
  roll(on);
  pros::delay(500);
  roll(loading);

  // back up
  drive(-1_ft);
  roll(poop);
  // to ball
  turn(-139_deg);
  roll(loading);
  Robot::model()->setMaxVoltage(9000);
  drive(7_ft);
  Robot::model()->setMaxVoltage(12000);

  // 3 to second corner goal
  turn(-175_deg);
  drive(1.85_ft);
  // 3 shoot second corner goal
  drive(-0.1_ft);
  cornerGoal();

  // back up
  drive(-1.2_ft);
  roll(poop);
  // to ball
  turn(-45_deg);
  asyncTask(pros::delay(500); roll(loading););
  drive(4.45_ft);

  // 4 to second edge goal
  turn(-135_deg);
  drive(0.9_ft);
  // 4 shoot second edge goal
  roll(on);
  pros::delay(500);

  // back up
  drive(-1.7_ft);
  roll(purge);
  pros::delay(600);
  // to ball
  turn(-45_deg);
  roll(loading);
  drive(4.8_ft);

  // 5 to third corner goal
  turn(-105_deg);
  drive(3.6_ft);
  // 5 shoot third corner goal
  cornerGoal();
  pros::delay(200);

  // back up
  drive(-1.3_ft);
  pros::delay(1000);
}

void opcontrol() {

  while (true) {

    if (mDigital(Y) && !pros::competition::is_connected()) {
      s.calibrate();
      Robot::roller()->initialize();
    }

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
