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

auto ballDistance = 0.7_ft;
auto ballVel = 0.3;

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

    double offset = Robot::vision()->getOffset() * 0.0055;

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

IterativePosPIDController pid(0.028, 0, 0.0004, 0, TimeUtilFactory().create());

void turn(QAngle a) {
  QAngle error = 0_deg;
  pid.controllerSet(0);
  Timer t;
  Timer settleTime;
  do {
    error = util::rollAngle180(a - Robot::imu()->getRemapped(180, -180) * degree);
    double out = pid.step(-error.convert(degree));
    Robot::model()->tank(out, -out);
    if (abs(error) < 3_deg) settleTime.placeHardMark();
    pros::delay(10);
  } while ((abs(error) >= 3_deg && t.getDtFromStart() < 3_s) ||
           settleTime.getDtFromHardMark() < 0.2_s);
  Robot::model()->tank(0, 0);
}

#define roll(x) Robot::roller()->setNewState(rollerStates::x)

void cornerGoal() {
  roll(loading);
  pros::delay(500);
  roll(onWithoutPoop);
  pros::delay(1200);
  roll(poop);
}

void autonomous() {

  Robot::imu().reset();

  // deploy
  roll(deploy);
  pros::delay(500);
  roll(loading);

  drive(2.3_ft);
  turn(135_deg);
  drive(2.7_ft);

  // 1 shoot first corner goal
  roll(loading);
  pros::delay(400);
  roll(topOut);
  pros::delay(400);
  roll(onWithoutPoop);
  pros::delay(1000);

  // back up
  drive(-2.5_ft);
  roll(purge);
  pros::delay(500);
  // to ball
  turn(-79_deg);
  roll(loading);
  driveBall(3.2_ft, 0.6);

  // 2 to first edge goal
  turn(172_deg);
  drive(3.7_ft);
  // 2 shoot first edge goal
  roll(on);
  pros::delay(600);
  roll(loadingWithoutPoop);

  // back up
  drive(-0.8_ft);
  roll(poop);
  // to ball
  turn(-92_deg);
  roll(loading);
  driveBall(3.45_ft, 0.2);

  // 3 to second corner goal
  turn(-135_deg);
  drive(2_ft);
  // 3 shoot second corner goal
  cornerGoal();

  // back up
  drive(-1_ft);
  roll(poop);
  // to ball
  turn(-9_deg);
  roll(loading);
  driveBall(3.8_ft, 0.6);

  // 4 to second edge goal
  turn(-94_deg);
  drive(1_ft);
  // 4 shoot second edge goal
  roll(on);
  pros::delay(500);

  // back up
  drive(-1.6_ft);
  roll(purge);
  pros::delay(600);
  // to ball
  turn(-2_deg);
  roll(loading);
  driveBall(3.3_ft, 0.5);

  // 5 to third corner goal
  turn(-55_deg);
  drive(3.6_ft);
  // 5 shoot third corner goal
  cornerGoal();
  roll(loadingWithoutPoop);
  pros::delay(300);

  // back up
  drive(-1.3_ft);
  roll(poop);
  // to ball
  turn(110_deg);
  roll(loading);
  driveBall(3.8_ft, 0.6);
  // 6 to third edge goal
  turn(-7_deg);
  drive(3_ft);

  // 6 shoot third edge goal
  roll(on);
  pros::delay(500);
  roll(poop);

  // back up
  drive(-1_ft);
  // to ball
  turn(75_deg);
  roll(loading);
  driveBall(3.8_ft, 0.3);
  // 7 to fourth corner goal
  turn(38_deg);
  drive(1.5_ft);

  // 7 shoot fourth corner goal
  cornerGoal();

  // back up
  drive(-1.5_ft);
  // to ball
  turn(170_deg);
  roll(loading);
  driveBall(3.6_ft, 0.6);

  // 8 to fourth edge goal
  turn(80_deg);
  drive(1_ft);
  // 8 shoot fourth edge goal
  roll(on);
  pros::delay(600);

  // back up
  drive(-0.5_ft);
  roll(onWithoutPoop);
  pros::delay(500);
  // to ball
  turn(-97_deg);
  roll(loading);
  driveBall(4_ft, 0.3);

  drive(-1_ft);
  Robot::model()->xArcade(-1, 0, 0);
  pros::delay(300);
  Robot::model()->xArcade(0, 0, 0);
  drive(2_ft);
  drive(-1_ft);
  Robot::model()->xArcade(1, 0, 0);
  pros::delay(300);
  Robot::model()->xArcade(0, 0, 0);
  drive(1.5_ft);
  roll(on);

  pros::delay(1000);
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
