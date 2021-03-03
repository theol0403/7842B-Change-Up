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

void shootCorner() {
  roll(topOut);
  pros::delay(400);
  roll(intakeWithoutPoop);
  pros::delay(200);
  asyncTask({
    pros::delay(400);
    roll(out);
  });
}

void shootEdge() {
  roll(shootWithoutPoop);
  pros::delay(400);
  roll(intakeWithoutPoop);
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(90_deg);

  // deploy after strafing from edge
  move->strafe(Line({0_ft, 0_ft}, {-1.5_ft, 0_ft}));

  roll(deploy);
  pros::delay(500);
  roll(intakeWithoutPoop);

  /* ------------------------------- first corner goal ------------------------------- */

  // to ball and goal
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {1.9_ft, 3.8_ft, 70_deg}), {});

  // shoot
  roll(topOut);
  pros::delay(400);
  roll(onWithoutPoop);
  pros::delay(400);
  roll(intakeWithoutPoop);

  // back up
  asyncTask({
    pros::delay(200);
    roll(out);
  });
  drive(-2.5_ft);

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  turn(-75_deg);

  // to ball
  roll(intake);
  drive(3.75_ft, {.ball_seek = 30_pct});

  // to goal
  turn(177_deg);
  drive(3_ft, {.goal_seek = 50_pct});

  // shoot
  shootEdge();

  // back up
  drive(-0.85_ft);

  /* --------------------------- second corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(poopIn);
  });
  turn(-92_deg);

  // to ball and goal
  pros::delay(200);
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-3.5_ft, 5.7_ft, -85_deg}), {.ball_seek = 10_pct});

  // shoot
  shootCorner();

  // back up
  drive(-3_ft);

  /* ---------------------------- second edge goal ---------------------------- */

  // purge and turn
  roll(out);
  pros::delay(200);
  turn(18_deg);

  // to ball
  roll(intake);
  drive(3.1_ft, {.ball_seek = 40_pct});

  // to goal
  turn(-92_deg);
  drive(3.5_ft, {.goal_seek = 50_pct});

  // shoot
  roll(topOut);
  pros::delay(400);
  roll(shootWithoutPoop);
  pros::delay(300);
  roll(intakeWithoutPoop);

  // back up
  drive(-1.6_ft);

  /* ---------------------------- third corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(poopIn);
  });
  turn(-2_deg);

  // to ball
  roll(intake);
  drive(4.3_ft, {.ball_seek = 50_pct});

  // to goal
  turn(-61_deg);
  drive(3.5_ft, {.goal_seek = 50_pct});

  // shoot
  shootCorner();

  // back up
  drive(-2_ft);

  /* ----------------------------- third edge goal ---------------------------- */

  // purge and turn
  roll(out);
  pros::delay(200);
  turn(112_deg);

  // to ball
  roll(intake);
  drive(4.2_ft, {.ball_seek = 60_pct});

  // to goal
  turn(-5_deg);
  drive(3.5_ft, {.goal_seek = 50_pct});

  // shoot
  shootEdge();

  // back up
  drive(-1.2_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(poopIn);
  });
  turn(77_deg);

  // to ball and goal
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-3_ft, 6.2_ft, -80_deg}), {.ball_seek = 20_pct});

  // shoot
  shootCorner();

  // back up
  drive(-1.6_ft);

  /* ---------------------------- fourth edge goal ---------------------------- */

  // purge and turn
  roll(out);
  pros::delay(200);
  turn(171_deg);

  // to ball
  roll(intake);
  drive(4.2_ft, {.ball_seek = 60_pct});

  // to goal
  turn(80_deg);
  drive(1_ft);

  // shoot
  shootEdge();

  // back up
  asyncTask({
    pros::delay(300);
    roll(out);
  });
  drive(-0.8_ft);

  /* ------------------------------- center goal ------------------------------ */

  // purge and turn
  roll(out);
  pros::delay(200);
  turn(-100_deg);

  // to ball
  roll(intake);
  drive(1.5_ft, {.ball_seek = 0_pct});

  // recenter
  turn(-101_deg);
  Timer t;
  t.placeMark();
  while (t.getDtFromMark() < 0.3_s) {
    Robot::model()->xArcade((Robot::vision()->getBlueOffset() + 60) * 0.022, 0, 0);
  }

  // move to poke
  drive(2.5_ft);
  drive(-0.8_ft);
  drive(1.8_ft);
  roll(on);
  pros::delay(600);
  drive(-2_ft);
}
