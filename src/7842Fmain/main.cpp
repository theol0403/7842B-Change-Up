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
  pros::delay(50);
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
  move(Mesh(29_deg, {2.6_ft, 1.5_ft, -3_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true, .start = 29_deg});
  Robot::model()->stop();

  // strafe to goal
  move(QuinticHermite(-180_deg, {-0.1_ft, -2.1_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot two red in corner
  shootCorner();

  // back up
  asyncTask(pros::delay(700); roll(out););
  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.6_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  roll(intake);
  turn(87_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(110_deg, {3.1_ft, 2.2_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -200_ms),
        .strafer = AB().addGoalVision(60_pct)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  asyncTask(pros::delay(600); roll(out););

  // back up
  drive(-2.8_ft, {.steerer = AB().addImu(0_deg)});

  /* ---------------------------- second edge goal ---------------------------- */

  // turn and purge
  turn(233_deg);
  roll(loadingPoop);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-4_ft, -5_ft, -100_deg}, 2.8, 3.5),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 20_pct).addRoller(rollerStates::top, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});

  // shoot one red in edge and remove one blue
  pros::delay(200);

  // back up
  drive(-0.7_ft);

  /* ---------------------------- second corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(400);
    roll(loadingPoop);
  });
  turn(184_deg);

  // drive to first ball and strafe to second ball
  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({3.6_ft, -1.1_ft, 0_deg}, 0.8, 1.2), {.start_v = 90_pct, .end_v = 0.1_pct});
  Robot::model()->stop();

  // strafe to goal
  move(QuinticHermite(-180_deg, {0.1_ft, 2.4_ft, 45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot two red in corner
  roll(on);
  pros::delay(200);
  shootCorner();
  roll(intake);
  pros::delay(50);
  roll(outSlow);

  // purge
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.3_ft, 3.8_ft, 0_deg}), {.curve = true, .start = -135_deg});

  /* ----------------------------- third edge goal ---------------------------- */

  // intake and turn
  roll(loadingPoop);
  turn(96_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(70_deg, {-3.4_ft, 2.2_ft, 180_deg}, 1.7, 3),
       {.curve = true,
        .steerer = AB().addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  // back up while turning right
  drive(-1.9_ft,
        {.rotator = AB()
                      .add(makeAngler(-90_deg, Limits<QAngle>(0.3_s, 60_deg / second)), 0_s, 70_pct)
                      .addImu(90_deg, 70_pct)});

  /* ---------------------------- third corner goal --------------------------- */

  // to ball
  roll(loadingPoop);
  drive(4.1_ft, {.steerer = AB().addBallVision(0_s, 70_pct)});

  // strafe and turn to side ball
  move(Line({-2.4_ft, -0.9_ft}), {.start = 90_deg, .rotator = AB().addImu(183_deg, 0_pct)});

  // strafe to goal
  move(QuinticHermite(-180_deg, {-0.4_ft, -2.5_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot two red in corner
  shootCorner();

  /* ---------------------------- fourth edge goal ---------------------------- */

  // purge
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.8_ft, 3.8_ft, 0_deg}), {.curve = true, .start = -135_deg});

  // turn and purge
  turn(35_deg);
  roll(loadingPoop);

  // to ball and curve left to goal
  move(QuinticHermite(200_deg, {-2_ft, -5_ft, -93_deg}, 2.5, 3),
       {.curve = true,
        .steerer = AB().addBallVision(0.0_s, 20_pct).addRoller(rollerStates::top, -200_ms),
        .strafer = AB().addGoalVision(70_pct)});

  pros::delay(200);

  drive(-1.1_ft);

  /* --------------------------- fourth corner goal --------------------------- */

  // purge and turn
  asyncTask({
    pros::delay(300);
    roll(loadingPoop);
  });
  turn(7_deg);

  // to ball and goal
  roll(intake);
  move(Mesh({0_ft, 0_ft, 90_deg}, {-3_ft, 6.25_ft, -10_deg}),
       {.curve = true, .strafer = AB().addBallVision(10_pct, 50_pct)});

  // shoot
  shootCorner();

  // back up
  drive(-1.6_ft);
}
