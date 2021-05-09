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

// void shootCorner() {
//   roll(topOut);
//   pros::delay(500);
//   roll(intakeWithoutPoop);
//   pros::delay(60);
//   asyncTask({
//     pros::delay(400);
//     roll(out);
//   });
// }

void shootEdge() {
  // roll(topOut);
  // pros::delay(500);
  // roll(shootWithoutPoop);
  // pros::delay(500);
  // asyncTask({
  //   pros::delay(350);
  //   roll(intakeWithoutPoop);
  // });
  roll(shoot);
  pros::delay(900);
  roll(off);
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(29_deg);

  roll(intake);
  move(Line({0_ft, 0_ft}, {2_ft, 0_ft}), {.end_v = 90_pct});
  move(Mesh({0_ft, 0_ft, 29_deg}, {2.6_ft, 1.3_ft, -3_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true, .start = 29_deg});
  Robot::model()->stop();

  move(QuinticHermite({0_ft, 0_ft, -180_deg}, {-0.0_ft, -1.9_ft, -45_deg}, 1.1),
       {.rotator = makeRotator(-42_deg, Limits<QAngle>(0.5_s, 60_deg / second))});

  // shoot
  roll(on);
  pros::delay(800);
  roll(topIntake);
  pros::delay(100);
  roll(off);

  asyncTask(pros::delay(700); roll(out););

  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.6_ft, 3.4_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  turn(88_deg);

  roll(intake);
  move(QuinticHermite({0_ft, 0_ft, 110_deg}, {3.3_ft, 2.2_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .start = 110_deg,
        .rotator = Injector().addBallVision(80_pct).addImu(0_deg, 1.6_s).build()});
  Robot::model()->stop();

  shootEdge();
  asyncTask(pros::delay(400); roll(out););

  drive(-2.8_ft, {.rotator = Injector().addImu(0_deg)});

  turn(225_deg);

  roll(loadingPoop);
  move(QuinticHermite({0_ft, 0_ft, 205_deg}, {-3_ft, -5.5_ft, -90_deg}, 2, 4.5),
       {.curve = true, .start = 205_deg});

  roll(topIntake);
  pros::delay(500);
  roll(off);

  drive(-0.7_ft);

  /* ---------------------------- third corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(loadingPoop);
  });
  turn(190_deg);

  move(QuarticBezier({{0_ft, 0_ft}, {0_ft, 1_ft}, {-2_ft, 2_ft}, {3_ft, 4_ft}, {3_ft, 5.7_ft}}),
       {.curve = true, .start = 90_deg});

  // shoot
  // shootCorner();

  // // back up
  // drive(-2_ft);

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
