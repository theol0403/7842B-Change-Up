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

auto ballDistance = 1_ft;
auto ballVel = 0.5;

void driveBall(const QLength& m, double ballPct) {
  auto generator = Robot::generator();

  auto startDist = m * ballPct;
  if ((m - startDist - ballDistance) < 0_m)
    throw std::runtime_error("Distance not enough to accelerate " +
                             std::to_string(startDist.convert(foot)) + "_ft and seek ball");

  // std::cout << "Accel: " << startDist.convert(foot) << std::endl;

  generator->follow(Line({0_m, 0_m}, {0_m, startDist}), true, 0, ballVel);

  auto seek = Trapezoidal(generator->limits, ballDistance, ballVel, ballVel, ballVel + 0.01);
  std::cout << seek.time.convert(second) << std::endl;

  // std::cout << "Seek: " << seek.length.convert(foot) << std::endl;

  Timer t;
  while (t.getDtFromStart() < seek.time) {
    QAngularSpeed leftWheel = (seek.vel / (1_pi * generator->scales.wheelDiameter)) * 360_deg;
    QAngularSpeed rightWheel = (seek.vel / (1_pi * generator->scales.wheelDiameter)) * 360_deg;

    double offset = Robot::vision()->getOffset() * 0.005;

    auto leftSpeed = (leftWheel / generator->gearset).convert(number) + offset;
    auto rightSpeed = (rightWheel / generator->gearset).convert(number) - offset;

    Robot::model()->tank(leftSpeed, rightSpeed);
    pros::delay(10);
  }
  Robot::model()->tank(0, 0);

  auto endDist = m - startDist - ballDistance;
  // std::cout << "Decel: " << endDist.convert(foot) << std::endl;
  generator->follow(Line({0_m, 0_m}, {0_m, endDist}), true, ballVel, 0);
}

IterativePosPIDController pid(0.024, 0, 0, 0, TimeUtilFactory().create());
void turn(QAngle a) {
  QAngle error = 0_deg;
  pid.controllerSet(0);
  Timer t;
  do {
    error = util::rollAngle180(a - Robot::imu()->getRemapped(180, -180) * degree);
    double out = pid.step(-error.convert(degree));
    Robot::model()->tank(out, -out);
  } while (abs(error) >= 3_deg && t.getDtFromStart() < 3_s);
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

  drive(2_ft);
  turn(135_deg);
  drive(2_ft);

  // 1 shoot first corner goal
  roll(loading);
  pros::delay(600);
  roll(on);
  pros::delay(800);
  roll(loading);

  // back up
  Robot::model()->setMaxVoltage(5500);
  drive(-2.5_ft);
  Robot::model()->setMaxVoltage(12000);
  roll(purge);
  pros::delay(500);
  // to ball
  turn(-60_deg);
  roll(loading);
  driveBall(4.95_ft, 0.6);

  // 2 to first edge goal
  turn(179_deg);
  drive(3.8_ft);
  // 2 shoot first edge goal
  roll(on);
  pros::delay(500);
  roll(poop);

  // back up
  drive(-1_ft);
  // to ball
  turn(-93_deg);
  roll(loading);
  driveBall(6_ft, 0.6);

  // 3 to second corner goal
  turn(-130_deg);
  drive(1.85_ft);
  // 3 shoot second corner goal
  cornerGoal();

  // back up
  drive(-1_ft);
  roll(poop);
  // to ball
  turn(0_deg);
  asyncTask(pros::delay(600); roll(loading););
  driveBall(4.8_ft, 0.6);

  // 4 to second edge goal
  turn(-90_deg);
  drive(0.9_ft);
  // 4 shoot second edge goal
  roll(on);
  pros::delay(500);

  // back up
  drive(-1.6_ft);
  roll(purge);
  pros::delay(600);
  // to ball
  turn(0_deg);
  roll(loading);
  driveBall(4.85_ft, 0.6);

  // 5 to third corner goal
  turn(-60_deg);
  drive(3.6_ft);
  // 5 shoot third corner goal
  cornerGoal();
  pros::delay(200);

  // back up
  drive(-1.3_ft);
  // to ball
  turn(120_deg);
  roll(loading);
  driveBall(4.5_ft, 0.6);
  // 5 to third edge goal
  turn(0_deg);
  drive(4_ft);

  // 5 shoot third edge goal
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
