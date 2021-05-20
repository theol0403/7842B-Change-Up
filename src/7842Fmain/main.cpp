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
  pros::delay(310);
  roll(intake);
  pros::delay(120);
  roll(outSlow);
}

void shootSingleCorner() {
  roll(intake);
  pros::delay(100);
  roll(outSlow);
}

void shootEdge() {
  roll(on);
  pros::delay(300);
  roll(shoot);
  pros::delay(300);
  roll(outSlow);
}

void autonomous() {

  // start facing right
  Robot::imu()->reset(29_deg);
  roll(deploy);
  pros::delay(500);
  roll(intake);

  /* ---------------------------- first corner goal --------------------------- */

  // to ball
  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(Mesh(29_deg, {2.6_ft, 1.4_ft, 0_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true, .start = 29_deg});
  Robot::model()->stop();

  roll(off);

  // strafe to goal
  move(
    QuinticHermite(-180_deg, {0_ft, -1.93_ft, -45_deg}, 1.1),
    {.start = 0_deg,
     .rotator = makeAngler(-37_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
     .steerer =
       AB().addRoller(rollerStates::on, -300_ms).addRoller(rollerStates::intake, 50_pct, 60_pct)});
  drive(-0.1_ft);

  // shoot two red in corner
  shootCorner();

  // back up
  asyncTask(roll(out); pros::delay(200); roll(off); pros::delay(500); roll(out););
  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.1_ft, 3.7_ft, 180_deg}, 1.3, 1),
       {.curve = true, .start = -45_deg});

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  roll(intake);
  turn(87_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(110_deg, {3.1_ft, 2.6_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -400_ms),
        .strafer = AB().addGoalVision(1.6_s)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  asyncTask(pros::delay(500); roll(out););

  // back up
  drive(-2.75_ft, {.rotator = AB().addImu(230_deg, 0.5_s)});

  /* ---------------------------- second edge goal ---------------------------- */

  // turn and purge
  roll(loadingPoop);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-3.7_ft, -5.5_ft, -110_deg}, 2.8, 3.5),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 00_pct).addRoller(rollerStates::top, -400_ms)});

  // shoot one red in edge and remove one blue
  // pros::delay(100);

  asyncTask(pros::delay(100); roll(outSlow););

  // back up
  drive(-0.6_ft);
  asyncTask(pros::delay(200); roll(forcePoop); pros::delay(400); roll(loadingPoop););
  turn(186_deg);

  /* ---------------------------- second corner goal --------------------------- */

  // drive to first ball and strafe to second ball
  move(Line({2.2_ft, 0_ft}), {.end_v = 90_pct});
  move(
    QuinticHermite({3.5_ft, -1.1_ft, 0_deg}, 0.8, 1.2),
    {.start_v = 90_pct, .end_v = 0.1_pct, .rotator = AB().addRoller(rollerStates::off, -200_ms)});
  Robot::model()->stop();

  // strafe to goal
  move(
    QuinticHermite(-180_deg, {0.25_ft, 2.55_ft, 45_deg}, 1.1),
    {.start = 0_deg,
     .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
     .steerer =
       AB().addRoller(rollerStates::on, -400_ms).addRoller(rollerStates::loading, 40_pct, 50_pct)});

  drive(-0.1_ft);

  // shoot two red in corner
  shootCorner();

  // purge
  asyncTask(roll(out); pros::delay(200); roll(off); pros::delay(500); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.7_ft, 2.85_ft, 0_deg}), {.curve = true, .start = -135_deg});

  /* ----------------------------- third edge goal ---------------------------- */

  // intake and turn
  roll(loadingPoop);
  turn(96_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(70_deg, {-3.3_ft, 2.2_ft, 183_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -400_ms),
        .strafer = AB().addGoalVision(70_pct)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  asyncTask(pros::delay(500); roll(loadingPoop););

  // back up while turning right
  drive(-1.6_ft, {.rotator = AB()
                               .add(makeAngler(-90_deg, Limits<QAngle>(0.3_s, 60_deg / second)),
                                    0.3_s, 50_pct)
                               .addImu(93_deg, 50_pct)});

  /* ---------------------------- third corner goal --------------------------- */

  // to ball
  drive(4.05_ft, {.strafer = AB().addBallVision(0_s, 70_pct)});

  // strafe and turn to goal
  move(Line({-3.4_ft, 1.85_ft}), {.start = 90_deg,
                                  .rotator = AB().addImu(152_deg, 10_pct),
                                  .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  asyncTask(pros::delay(200); roll(out); pros::delay(200); roll(loading););
  drive(-0.1_ft);
  Robot::model()->stop();

  // shoot two red in corner

  /* ------------------------------- center goal ------------------------------ */

  // strafe to ball
  move(
    QuinticHermite(225_deg, {-1.6_ft, 0.6_ft, 90_deg}, 1.5, 1.5),
    {
      .start = 45_deg,
      .rotator = AB()
                   .add(makeAngler(47_deg, Limits<QAngle>(0.5_s, 60_deg / second)), 10_pct, 70_pct)
                   .addImu(185_deg, 70_pct),
    });

  // strafe to center goal
  asyncTask(pros::delay(400); roll(forceShoot); pros::delay(300); roll(loading););
  move(Line({2.4_ft, -2.8_ft}), {.start = 180_deg});

  asyncTask(pros::delay(300); roll(loadingPoop););
  turn(10_deg);

  // drive to goal
  drive(2.5_ft, {.end_v = 40_pct, .top_v = 70_pct, .steerer = AB().addGoalVision(0_pct, 20_pct)});
  drive(-0.25_ft);

  pros::delay(300);
  roll(onPoop);
  pros::delay(400);
  roll(forcePoop);
  pros::delay(630);
  roll(off);

  /* ---------------------------- fourth edge goal ---------------------------- */

  // back up and turn
  drive(-1.6_ft);
  roll(loadingPoop);
  turn(47_deg);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-2.8_ft, -5.5_ft, -93_deg}, 2.5, 2.5),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 00_pct).addRoller(rollerStates::top, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});

  pros::delay(200);

  drive(-0.8_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(300);
    roll(loadingPoop);
  });
  turn(6_deg);

  // drive to first ball and strafe to second ball
  move(Line({3_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({2.8_ft, -0.9_ft, 0_deg}, 0.8, 1.2), {.start_v = 90_pct, .end_v = 0.1_pct});
  Robot::model()->stop();

  roll(off);

  // strafe to goal
  move(
    QuinticHermite(-180_deg, {0.3_ft, 2.4_ft, 45_deg}, 1.1),
    {.start = 0_deg,
     .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
     .steerer =
       AB().addRoller(rollerStates::on, -400_ms).addRoller(rollerStates::loading, 40_pct, 50_pct)});

  drive(-0.1_ft);
  shootCorner();
  asyncTask(pros::delay(100); roll(out); pros::delay(200); roll(off););

  // back up
  drive(-1_ft);
  roll(off);

  pros::delay(5000);
}
