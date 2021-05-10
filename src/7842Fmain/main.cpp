#include "main.h"
#include "auton.hpp"
#include "driver.hpp"

void disabled() {}
void competition_initialize() {}

void initialize() {
  pros::delay(100);
  Robot::initialize();
}

void opcontrol() {
  while (true) {
    driverBaseControl();
    driverDeviceControl();
    pros::delay(10);
  }
}

void shootEdge() {
  roll(shoot);
  pros::delay(900);
  roll(off);
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(29_deg);

  roll(intake);
  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(Mesh(29_deg, {2.6_ft, 1.3_ft, -3_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true});
  Robot::model()->stop();

  move(QuinticHermite(-180_deg, {-0.1_ft, -2.0_ft, -45_deg}, 1.1),
       {.rotator = makeAngler(-42_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot
  roll(on);
  pros::delay(500);
  roll(intake);
  pros::delay(50);
  roll(off);

  asyncTask(pros::delay(700); roll(out););

  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.6_ft, 3.7_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  turn(87_deg);

  roll(intake);
  move(QuinticHermite(110_deg, {3.2_ft, 2.0_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(80_pct).addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -300_ms)});
  Robot::model()->stop();

  roll(shoot);
  pros::delay(600);
  roll(off);
  asyncTask(pros::delay(600); roll(out););

  drive(-2.8_ft, {.steerer = AB().addImu(0_deg)});

  turn(234_deg);

  roll(loadingPoop);
  move(
    QuinticHermite(200_deg, {-3.7_ft, -5_ft, -90_deg}, 2, 4.5),
    {.curve = true, .steerer = AB().addBallVision(0_s, 2_s).addRoller(rollerStates::top, -200_ms)});

  pros::delay(200);
  drive(-0.7_ft);

  /* ---------------------------- third corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(loadingPoop);
  });
  turn(183_deg);

  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({3.8_ft, -1.6_ft, 0_deg}, 0.8, 0.9),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true});
  Robot::model()->stop();

  move(QuinticHermite(-180_deg, {0.2_ft, 2.2_ft, 45_deg}, 1.1),
       {.rotator = makeAngler(41_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  roll(on);
  pros::delay(500);
  roll(topIntake);
  pros::delay(100);
  roll(off);

  // back up
  move(QuinticHermite(45_deg, {3.6_ft, 3.7_ft, 0_deg}), {.curve = true, .start = -135_deg});

  turn(93_deg);

  roll(intake);
  move(QuinticHermite(70_deg, {-3.2_ft, 2.0_ft, 180_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(80_pct).addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -300_ms)});
  Robot::model()->stop();

  // /* ----------------------------- third edge goal ---------------------------- */

  // // purge and turn
  // roll(out);
  // pros::delay(200);
  // turn(111_deg);

  // // to ball
  // roll(intake);
  // drive(4.35_ft, {.ball_seek = 30_pct});

  // // to goal
  // turn(-5_deg);
  // drive(3.1_ft, {.goal_seek = 50_pct});

  // // shoot
  // shootEdge();

  // // back up
  // drive(-1.2_ft);

  // /* --------------------------- fourth corner goal --------------------------- */

  // // purge and turn
  // asyncTask({
  //   pros::delay(300);
  //   roll(poopIn);
  // });
  // turn(77_deg);

  // // to ball and goal
  // roll(intake);
  // move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-3_ft, 6.25_ft, -80_deg}), {.ball_seek = 10_pct});

  // // shoot
  // shootCorner();

  // // back up
  // drive(-1.6_ft);

  // /* ---------------------------- fourth edge goal ---------------------------- */

  // // purge and turn
  // roll(out);
  // pros::delay(200);
  // turn(172_deg);

  // // to ball
  // roll(intake);
  // drive(4.27_ft, {.ball_seek = 40_pct});

  // // to goal
  // turn(81_deg);
  // drive(0.9_ft);

  // // shoot
  // shootEdge();
  // roll(onWithoutPoop);
  // pros::delay(300);

  // // back up
  // asyncTask({
  //   pros::delay(400);
  //   roll(out);
  // });
  // drive(-0.8_ft);

  // /* ------------------------------- center goal ------------------------------ */

  // // purge and turn
  // roll(out);
  // pros::delay(200);
  // turn(-94_deg);

  // // to ball
  // roll(intake);
  // drive(2_ft, {.ball_seek = 0_pct});

  // turn(-97_deg);
  // Timer t;
  // while (t.getDtFromStart() < 0.2_s) {
  //   Robot::model()->xArcade((Robot::vision()->getBlueOffset()) * 0.025, 0, 0);
  //   pros::delay(10);
  // }
  // move(Line({0_ft, 0_ft}, {0.55_ft, 0_ft}));

  // // move to poke
  // drive(1.42_ft);
  // drive(-1_ft);
  // turn(-97_deg);
  // move(Line({0_ft, 0_ft}, {-0.4_ft, 0_ft}));
  // drive(1.4_ft);
  // roll(onWithoutPoop);
  // pros::delay(600);
  // drive(-1_ft);
  // turn(-97_deg);
  // move(Line({0_ft, 0_ft}, {0.75_ft, 0_ft}));
  // drive(1.3_ft);
  // drive(-2_ft);
}
