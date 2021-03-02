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
  roll(shootWithoutPoop);
  pros::delay(500);
  roll(intakeWithoutPoop);
}

void shootEdge() {
  roll(shootWithoutPoop);
  pros::delay(400);
  roll(intakeWithoutPoop);
}

void align(const QTime& time) {
  Timer t;
  while (t.getDtFromStart() < time) {
    Robot::model()->xArcade(Robot::vision()->getBlueOffset() * 0.02, 0, 0);
  }
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
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {2_ft, 4_ft, 60_deg}), {});

  // shoot
  roll(onWithoutPoop);
  pros::delay(100);
  roll(shootWithoutPoop);
  pros::delay(800);
  roll(onWithoutPoop);
  pros::delay(200);

  // back up
  asyncTask({
    pros::delay(200);
    roll(out);
  });
  drive(-2.5_ft);

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  pros::delay(200);
  turn(-75_deg);

  // to ball
  roll(intake);
  drive(3.8_ft, {.ball_seek = 30_pct});

  // to goal
  turn(177_deg);
  drive(3_ft, {.goal_seek = 50_pct});

  // shoot
  shootEdge();

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
  pros::delay(200);
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-3.5_ft, 6.2_ft, -80_deg}), {.ball_seek = 10_pct});

  // shoot
  shootCorner();

  // back up
  drive(-1_ft);

  /* ---------------------------- second edge goal ---------------------------- */

  // purge and turn
  roll(out);
  pros::delay(600);
  turn(21_deg);

  // to ball
  roll(intake);
  drive(5.2_ft, {.ball_seek = 60_pct});

  // to goal
  turn(-91_deg);
  drive(3.8_ft, {.goal_seek = 50_pct});

  // shoot
  roll(onWithoutPoop);
  pros::delay(100);
  roll(shootWithoutPoop);
  pros::delay(800);
  roll(out);

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
  drive(4.5_ft, {.ball_seek = 50_pct});

  // to goal
  turn(-60_deg);
  drive(4.5_ft, {.goal_seek = 50_pct});

  // shoot
  shootCorner();

  // back up
  drive(-1.3_ft);

  /* ----------------------------- third edge goal ---------------------------- */

  // purge and turn
  roll(out);
  turn(113_deg);

  // to ball
  roll(intake);
  drive(5.2_ft, {.ball_seek = 60_pct});

  // to goal
  turn(-5_deg);
  drive(3.5_ft, {.goal_seek = 50_pct});

  // shoot
  shootEdge();
  roll(out);

  // back up
  drive(-1.2_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  roll(poopIn);
  turn(77_deg);

  // to ball and goal
  roll(intake);
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-2_ft, 6_ft, -75_deg}), {.ball_seek = 20_pct});

  // shoot
  shootCorner();

  // back up
  drive(-1.5_ft);

  /* ---------------------------- fourth edge goal ---------------------------- */

  // purge and turn
  roll(out);
  pros::delay(200);
  turn(173_deg);

  // to ball
  roll(intake);
  drive(4_ft, {.ball_seek = 60_pct});

  // to goal
  turn(80_deg);
  drive(1_ft);

  // shoot
  shootEdge();

  // back up
  drive(-0.8_ft);

  /* ------------------------------- center goal ------------------------------ */

  // purge and turn
  roll(out);
  pros::delay(400);
  turn(-104_deg);

  // to ball
  roll(intake);
  drive(2_ft, {.ball_seek = 0_pct});

  // recenter
  turn(-101_deg);
  align(0.3_s);

  // move to poke
  move->strafe(Line({0_ft, 0_ft}, {0.4_ft, 0_ft}));
  drive(1.5_ft);

  // back up and allign
  drive(-1_ft);
  align(0.4_s);

  // shoot
  drive(2_ft);
  roll(on);
  pros::delay(600);
  drive(-2_ft);
}
