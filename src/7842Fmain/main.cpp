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
  roll(shoot);
  pros::delay(1000);
  asyncTask({
    pros::delay(500);
    roll(intake);
  });
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(29_deg);

  roll(intake);
  move(Line({0_ft, 0_ft}, {2_ft, 0_ft}), {.end_v = 90_pct});
  move(Mesh({0_ft, 0_ft, 29_deg}, {2.6_ft, 1.3_ft, -3_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true, .start = 29_deg});
  Robot::model()->stop();

  move(QuinticHermite({0_ft, 0_ft, -180_deg}, {-0.1_ft, -1.9_ft, -45_deg}, 1.1),
       {.rotator = makeRotator(-45_deg, Limits<QAngle>(0.5_s, 60_deg / second))});

  // shoot
  roll(on);
  pros::delay(600);
  roll(topIntake);
  pros::delay(600);
  roll(off);

  asyncTask(pros::delay(700); roll(out););

  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.5_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  turn(90_deg);

  roll(intake);
  move(QuinticHermite({0_ft, 0_ft, 105_deg}, {3.5_ft, 1.7_ft, 0_deg}, 1, 3),
       {.curve = true,
        .start = 90_deg,
        .rotator = makeVision({.ball = 0_pct}, [](const Profile<>::State& state) {
          if (state.t > 1.6_s) {
            auto error = util::rollAngle180(
              0_deg - (-1 * Robot::imu()->imu->get_rotation() * degree - Robot::imu()->offset));
            return Robot::imu()->pid->step(-error.convert(degree)) * rpm * 20;
          }
          return 0_rpm;
        })});
  Robot::model()->stop();

  shootEdge();

  drive(-1_ft);

  // asyncTask(pros::delay(1200); roll(out););
  // move(QuinticHermite({0_ft, 0_ft, 0_deg}, {2_ft, 3_ft, 120_deg}),
  //      {.curve = true, .start = 180_deg});

  // roll(intake);

  // move(
  //   make_piecewise<QuinticHermite>({{0_ft, 0_ft, 0_deg}, {3_ft, 4_ft, 0_deg}, {2_ft, 6_ft, 0_deg}}),
  //   {.curve = true});

  // // deploy after strafing from edge
  // move(Line({0_ft, 0_ft}, {-1.5_ft, 0_ft}));

  // roll(deploy);
  // pros::delay(500);
  // roll(intakeWithoutPoop);

  // /* ------------------------------- first corner goal ------------------------------- */

  // // to ball and goal
  // move->curve(Mesh({0_ft, 0_ft, 0_deg}, {1.9_ft, 3.8_ft, 70_deg}), {});

  // // shoot
  // roll(intakeWithoutPoop);
  // pros::delay(100);
  // roll(topOut);
  // pros::delay(400);
  // roll(onWithoutPoop);
  // pros::delay(500);
  // roll(off);

  // // back up
  // asyncTask({
  //   pros::delay(200);
  //   roll(out);
  // });
  // drive(-2.5_ft);

  // /* ----------------------------- first edge goal ---------------------------- */

  // // purge and turn
  // turn(-75_deg);

  // // to ball
  // roll(intake);
  // drive(3.75_ft, {.ball_seek = 30_pct});

  // // to goal
  // turn(177_deg);
  // drive(3_ft, {.goal_seek = 50_pct});

  // // shoot
  // shootEdge();

  // // back up
  // drive(-0.85_ft);

  // /* --------------------------- second corner goal --------------------------- */

  // // purge and turn
  // asyncTask({
  //   pros::delay(400);
  //   roll(poopIn);
  // });
  // turn(-92_deg);

  // // to ball and goal
  // pros::delay(200);
  // roll(intake);
  // move->curve(Mesh({0_ft, 0_ft, 0_deg}, {-3.4_ft, 5.75_ft, -85_deg}), {.ball_seek = 10_pct});

  // // shoot
  // shootCorner();

  // // back up
  // drive(-3_ft);

  // /* ---------------------------- second edge goal ---------------------------- */

  // // purge and turn
  // roll(out);
  // pros::delay(200);
  // turn(16_deg);

  // // to ball
  // roll(intake);
  // drive(3.2_ft, {.ball_seek = 20_pct});

  // // to goal
  // turn(-92_deg);
  // drive(3_ft, {.goal_seek = 50_pct});

  // // shoot
  // roll(topOut);
  // pros::delay(500);
  // roll(shootWithoutPoop);
  // pros::delay(500);
  // asyncTask({
  //   pros::delay(350);
  //   roll(intakeWithoutPoop);
  // });

  // // back up
  // drive(-1.6_ft);

  // /* ---------------------------- third corner goal --------------------------- */

  // // purge and turn
  // asyncTask({
  //   pros::delay(400);
  //   roll(poopIn);
  // });
  // turn(-2_deg);

  // // to ball
  // roll(intake);
  // drive(4.3_ft, {.ball_seek = 50_pct});

  // // to goal
  // turn(-61_deg);
  // drive(3.3_ft, {.goal_seek = 50_pct});

  // // shoot
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
