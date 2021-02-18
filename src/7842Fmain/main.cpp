#include "main.h"
#include "config.hpp"
#include "driver.hpp"

void disabled() {}
void competition_initialize() {}

void initialize() {
  pros::delay(200);
  Robot::initialize();
}

#define asyncTask(x) pros::Task([&]() { x });

void drive(const QLength& m) {
  Robot::generator()->follow(Line({0_m, 0_m}, {0_m, abs(m)}), m >= 0_m);
}

IterativePosPIDController pid(0.024, 0, 0, 0, TimeUtilFactory().create());
void turn(QAngle a) {
  QAngle error = 0_deg;
  pid.controllerSet(0);
  do {
    error = util::rollAngle180(a - Robot::imu()->getRemapped(180, -180) * degree);
    double out = pid.step(-error.convert(degree));
    Robot::model()->tank(out, -out);
  } while (abs(error) >= 3_deg);
  Robot::model()->tank(0, 0);
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
  Robot::imu().reset();

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
  turn(-105_deg);
  roll(loading);
  drive(4.95_ft);

  // 2 to first edge goal
  turn(134_deg);
  drive(3.8_ft);
  // 2 shoot first edge goal
  roll(on);
  pros::delay(500);
  roll(poop);

  // back up
  drive(-1_ft);
  // to ball
  turn(-138_deg);
  roll(loading);
  Robot::model()->setMaxVoltage(9000);
  drive(7_ft);
  Robot::model()->setMaxVoltage(12000);

  // 3 to second corner goal
  turn(-175_deg);
  drive(1.85_ft);
  // 3 shoot second corner goal
  cornerGoal();

  // back up
  drive(-1_ft);
  roll(poop);
  // to ball
  turn(-45_deg);
  asyncTask(pros::delay(1000); roll(loading););
  drive(4.8_ft);

  // 4 to second edge goal
  turn(-135_deg);
  drive(0.9_ft);
  // 4 shoot second edge goal
  roll(on);
  pros::delay(500);

  // back up
  drive(-1.6_ft);
  roll(purge);
  pros::delay(600);
  // to ball
  turn(-45_deg);
  roll(loading);
  drive(4.85_ft);

  // 5 to third corner goal
  turn(-105_deg);
  drive(3.6_ft);
  // 5 shoot third corner goal
  cornerGoal();
  pros::delay(200);

  // back up
  drive(-1.3_ft);
  turn(75_deg);
  roll(loading);
  drive(4.5_ft);
  turn(-45_deg);
  drive(4_ft);

  // 2 shoot first edge goal
  roll(on);
  pros::delay(500);
  roll(poop);

  // back up
  drive(-1_ft);
  pros::delay(2000);
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
