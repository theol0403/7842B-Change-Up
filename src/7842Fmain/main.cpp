#include "main.h"
#include "auton.hpp"
#include "driver.hpp"

void disabled() {}
void competition_initialize() {}

void initialize() {
  pros::delay(200);
  Robot::initialize();
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}

void autonomous() {
  // start facing right
  Robot::imu()->reset(90_deg);

  // deploy after strafing from edge
  asyncTask({
    pros::delay(500);
    roll(deploy);
  });
  move->strafe(Line({0_ft, 0_ft}, {-1.5_ft, 0_ft}));
  roll(intake);

  /* ------------------------------- first corner goal ------------------------------- */

  // to ball and goal
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {1.5_ft, 4_ft, 60_deg}), {.ball_seek = 0_pct});

  // shoot
  roll(onWithoutPoop);
  pros::delay(1000);

  // back up
  drive(-2.5_ft);

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  roll(purge);
  pros::delay(200);
  turn(-79_deg);

  // to ball
  roll(intake);
  drive(3.2_ft, {.ball_seek = 60_pct});

  // to goal
  turn(172_deg);
  drive(3.75_ft);

  // shoot
  roll(onWithoutPoop);
  pros::delay(500);
  roll(intakeWithoutPoop);

  // back up
  drive(-0.8_ft);

  /* --------------------------- second corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(200);
    roll(poopIn);
  });
  turn(-92_deg);

  // to ball and goal
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-1.5_ft, 5_ft, -60_deg}), {.ball_seek = 20_pct});

  // shoot
  roll(onWithoutPoop);
  pros::delay(1000);

  // back up
  drive(-1_ft);

  /* ---------------------------- second edge goal ---------------------------- */

  // purge and turn
  roll(purge);
  turn(-9_deg);

  // to ball
  roll(intake);
  drive(3.85_ft, {.ball_seek = 60_pct});

  // to goal
  turn(-94_deg);
  drive(1.2_ft);

  // shoot
  roll(onWithoutPoop);
  pros::delay(500);
  roll(intakeWithoutPoop);

  // back up
  drive(-1.6_ft);

  /* ---------------------------- third corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(200);
    roll(poopIn);
  });
  turn(-2_deg);

  // to ball
  roll(intake);
  drive(3.4_ft, {.ball_seek = 50_pct});

  // to goal
  turn(-56_deg);
  drive(3.6_ft);

  // shoot
  roll(onWithoutPoop);
  pros::delay(1000);

  // back up
  drive(-1.3_ft);

  /* ----------------------------- third edge goal ---------------------------- */

  // purge and turn
  roll(poopIn);
  turn(110_deg);

  // to ball
  roll(intake);
  drive(3.8_ft, {.ball_seek = 60_pct});

  // to goal
  turn(-8_deg);
  drive(3_ft);

  // shoot
  roll(onWithoutPoop);
  pros::delay(500);
  roll(poopIn);

  // back up
  drive(-1_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  roll(poopIn);
  turn(75_deg);

  // to ball and goal
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-1.5_ft, 5_ft, -60_deg}), {.ball_seek = 20_pct});

  // shoot
  roll(onWithoutPoop);
  pros::delay(1000);

  // back up
  drive(-1.5_ft);

  /* ---------------------------- fourth edge goal ---------------------------- */

  // purge and turn
  roll(poopIn);
  turn(170_deg);

  // to ball
  roll(intake);
  drive(3.8_ft, {.ball_seek = 60_pct});

  // to goal
  turn(80_deg);
  drive(1_ft);

  // shoot
  roll(onWithoutPoop);
  pros::delay(500);

  // back up
  drive(-0.5_ft);

  /* ------------------------------- center goal ------------------------------ */

  // purge and turn
  roll(purge);
  pros::delay(800);
  turn(-101_deg);

  // to ball
  roll(intake);
  drive(2_ft, {.ball_seek = 40_pct});

  // recenter
  turn(-101_deg);
  drive(2_ft);

  drive(-1_ft);
  drive(2_ft);

  roll(onWithoutPoop);
  pros::delay(600);
  drive(-2_ft);
}
