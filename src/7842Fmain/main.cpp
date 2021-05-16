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

void shootCorner() {
  roll(on);
  pros::delay(500);
  roll(intake);
  pros::delay(60);
  roll(outSlow);
}

void shootSingleCorner() {
  roll(on);
  pros::delay(100);
  roll(intake);
  pros::delay(100);
  roll(outSlow);
}

void shootEdge() {
  roll(on);
  pros::delay(300);
  roll(shoot);
  pros::delay(400);
  roll(off);
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(29_deg);
  // deploy
  roll(intake);

  /* ---------------------------- first corner goal --------------------------- */

  // to ball
  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(Mesh(29_deg, {2.4_ft, 1.5_ft, 4_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true, .start = 29_deg});
  Robot::model()->stop();

  // strafe to goal
  move(QuinticHermite(-180_deg, {0_ft, -1.9_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-37_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot two red in corner
  shootCorner();

  // back up
  asyncTask(pros::delay(700); roll(out););
  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3_ft, 3.8_ft, 180_deg}, 1.3, 1),
       {.curve = true, .start = -45_deg});

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  roll(intake);
  turn(87_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(110_deg, {3.1_ft, 2.5_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -200_ms),
        .strafer = AB().addGoalVision(1.6_s)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  asyncTask(pros::delay(600); roll(out););

  // back up
  drive(-2.5_ft, {.rotator = AB().addImu(231_deg, 0.5_s)});

  /* ---------------------------- second edge goal ---------------------------- */

  // turn and purge
  roll(loadingPoop);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-3.8_ft, -5_ft, -110_deg}, 2.8, 3.5),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 00_pct).addRoller(rollerStates::top, -200_ms),
        .strafer = AB().addGoalVision(-2.5_ft, -1.5_ft)});

  // shoot one red in edge and remove one blue
  pros::delay(200);

  // back up
  drive(-0.6_ft);
  turn(186_deg);

  /* ---------------------------- second corner goal --------------------------- */

  // purge and turn
  roll(loadingPoop);

  // drive to first ball and strafe to second ball
  move(Line({2.2_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({3.5_ft, -1.0_ft, 0_deg}, 0.8, 1.2), {.start_v = 90_pct, .end_v = 0.1_pct});
  Robot::model()->stop();

  // strafe to goal
  move(QuinticHermite(-180_deg, {0.2_ft, 2.6_ft, 45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  drive(-0.1_ft);

  // shoot two red in corner
  shootCorner();

  // purge
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.9_ft, 2.7_ft, 0_deg}), {.curve = true, .start = -135_deg});

  /* ----------------------------- third edge goal ---------------------------- */

  // intake and turn
  roll(loadingPoop);
  turn(96_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(70_deg, {-3.4_ft, 2.4_ft, 180_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  asyncTask(pros::delay(500); roll(loadingPoop););

  // back up while turning right
  drive(-1.5_ft, {.rotator = AB()
                               .add(makeAngler(-90_deg, Limits<QAngle>(0.3_s, 60_deg / second)),
                                    0.3_s, 50_pct)
                               .addImu(93_deg, 50_pct)});

  /* ---------------------------- third corner goal --------------------------- */

  // to ball
  drive(4.1_ft, {.strafer = AB().addBallVision(0_s, 70_pct)});

  // strafe and turn to goal
  move(Line({-3.4_ft, 1.8_ft}), {.start = 90_deg,
                                 .rotator = AB().addImu(145_deg, 10_pct),
                                 .steerer = AB().addRoller(rollerStates::on, -300_ms)});
  drive(-0.1_ft);
  Robot::model()->stop();

  // shoot two red in corner
  shootSingleCorner();

  /* ------------------------------- center goal ------------------------------ */
  roll(loading);

  // strafe to ball
  move(
    QuinticHermite(225_deg, {-1.5_ft, 0.5_ft, 90_deg}, 1.2, 1.4),
    {
      .start = 45_deg,
      .rotator = AB()
                   .add(makeAngler(47_deg, Limits<QAngle>(0.5_s, 60_deg / second)), 10_pct, 70_pct)
                   .addImu(185_deg, 70_pct),
    });

  // strafe to center goal
  asyncTask(pros::delay(400); roll(shoot); pros::delay(500); roll(loadingPoop););
  move(Line({2.4_ft, -2.6_ft}), {.start = 180_deg});
  turn(10_deg);

  roll(loadingPoop);

  // drive to goal
  drive(2.4_ft, {.steerer = AB().addGoalVision(0_pct)});
  drive(-0.2_ft);

  pros::delay(500);
  roll(onPoop);
  pros::delay(400);
  roll(forcePoop);
  pros::delay(600);
  roll(off);

  /* ---------------------------- fourth edge goal ---------------------------- */

  // back up and turn
  drive(-1.5_ft);
  turn(45_deg);

  roll(loadingPoop);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-3_ft, -5.5_ft, -93_deg}, 2.5, 2.5),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 40_pct).addRoller(rollerStates::top, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});

  pros::delay(200);

  drive(-0.8_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(300);
    roll(loadingPoop);
  });
  turn(7_deg);

  // drive to first ball and strafe to second ball
  move(Line({3_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({2.8_ft, -0.9_ft, 0_deg}, 0.8, 1.2), {.start_v = 90_pct, .end_v = 0.1_pct});
  Robot::model()->stop();

  // strafe to goal
  move(QuinticHermite(-180_deg, {0.2_ft, 2.2_ft, 45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  drive(-0.1_ft);
  // shoot two red in corner
  shootCorner();

  // back up
  drive(-1_ft);
}
