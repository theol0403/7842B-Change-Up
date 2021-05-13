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
  roll(topIntake);
  pros::delay(50);
  roll(off);
}

void shootEdge() {
  roll(shoot);
  pros::delay(600);
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
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true});
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
  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.5_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  /* ----------------------------- first edge goal ---------------------------- */

  // purge and turn
  roll(intake);
  turn(87_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(110_deg, {3.1_ft, 2.1_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(60_pct).addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -200_ms)});
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
  move(QuinticHermite(200_deg, {-3.8_ft, -5_ft, -90_deg}, 2.8, 3.5),
       {.curve = true,
        .steerer = AB()
                     .addBallVision(0.0_s, 1.5_s)
                     .addGoalVision(60_pct)
                     .addRoller(rollerStates::top, -200_ms)});

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
  turn(183_deg);

  // drive to first ball and strafe to second ball
  move(Line({2_ft, 0_ft}), {.end_v = 90_pct});
  move(QuinticHermite({3.6_ft, -1.2_ft, 0_deg}, 0.8, 1.2), {.start_v = 90_pct, .end_v = 0.1_pct});
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

  // purge
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.5_ft, 3.9_ft, 0_deg}), {.curve = true, .start = -135_deg});

  /* ----------------------------- third edge goal ---------------------------- */

  // intake and turn
  roll(loadingPoop);
  turn(96_deg);

  // drive two balls and curve right to goal
  move(QuinticHermite(70_deg, {-3.2_ft, 2.1_ft, 180_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(80_pct).addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -200_ms)});
  Robot::model()->stop();

  // shoot two red in edge
  shootEdge();

  // back up while turning right
  drive(-2_ft,
        {.rotator = AB()
                      .add(makeAngler(-90_deg, Limits<QAngle>(0.3_s, 60_deg / second)), 0_s, 70_pct)
                      .addImu(90_deg, 70_pct)});

  /* ---------------------------- third corner goal --------------------------- */

  // to ball
  roll(loadingPoop);
  drive(4_ft, {.steerer = AB().addBallVision(0_s, 70_pct)});

  // strafe and turn to side ball
  move(Line({-2.1_ft, -0.7_ft}), {.start = 90_deg, .rotator = AB().addImu(180_deg, 0_pct)});

  // strafe to goal
  move(QuinticHermite(-180_deg, {-0.1_ft, -2.2_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot two red in corner
  shootCorner();

  /* ---------------------------- fourth edge goal ---------------------------- */

  // purge
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.5_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});
}
