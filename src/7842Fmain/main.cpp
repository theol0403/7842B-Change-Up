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
  pros::delay(1000);
}

#define roll(x) Robot::roller()->setNewState(rollerStates::x)

void autonomous() {
  s.reset();

  roll(deploy);
  pros::delay(500);
  roll(loading);

  turn(55_deg);
  drive(1.1_ft);
  Robot::model()->xArcade(-1, 0, 0);
  pros::delay(100);
  Robot::model()->xArcade(0, 0, 0);

  // goal 1
  roll(on);
  pros::delay(900);

  Robot::model()->setMaxVoltage(6000);
  drive(-2.5_ft);
  Robot::model()->setMaxVoltage(12000);
  roll(purge);
  pros::delay(500);
  turn(-105_deg);
  roll(loading);
  drive(5.15_ft);
  turn(135_deg);

  drive(4_ft);
  drive(-0.2_ft);

  // goal 2
  roll(on);
  pros::delay(500);
  roll(loading);
  drive(-1_ft);

  roll(poop);
  turn(-139_deg);
  roll(loading);
  Robot::model()->setMaxVoltage(9000);
  drive(6.1_ft);
  Robot::model()->setMaxVoltage(12000);
  roll(loading);

  turn(-175_deg);
  drive(1.8_ft);

  // corner goal
  roll(on);
  pros::delay(500);
  roll(poop);
  pros::delay(500);

  drive(-1.3_ft);
  roll(poop);

  turn(-45_deg);
  drive(4.35_ft);
  roll(loading);
  turn(-135_deg);

  drive(0.7_ft);
  // edge goal
  roll(on);
  pros::delay(700);
  drive(-1.5_ft);
  roll(purge);
  pros::delay(700);
  turn(-45_deg);

  roll(loading);
  drive(4.7_ft);
  turn(-105_deg);
  drive(2.2_ft);
  // far corner goal
  roll(on);
  pros::delay(500);
  roll(poop);
  pros::delay(500);

  drive(-1.3_ft);
}

void opcontrol() {

  while (true) {

    if (mDigital(Y) && !pros::competition::is_connected()) {
      Robot::roller()->initialize();
      s.calibrate();
    }

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
