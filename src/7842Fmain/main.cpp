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
  move(Mesh(29_deg, {2.6_ft, 1.5_ft, -3_deg}),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true});
  Robot::model()->stop();

  move(QuinticHermite(-180_deg, {-0.1_ft, -2.1_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  // shoot
  roll(on);
  pros::delay(500);
  roll(topIntake);
  pros::delay(50);
  roll(off);

  asyncTask(pros::delay(700); roll(out););

  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.5_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});

  turn(87_deg);

  roll(intake);
  move(QuinticHermite(110_deg, {3.1_ft, 2.1_ft, 0_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(60_pct).addImu(-5_deg, 1.6_s).addRoller(rollerStates::on, -200_ms)});
  Robot::model()->stop();

  roll(shoot);
  pros::delay(600);
  roll(off);
  asyncTask(pros::delay(600); roll(out););

  drive(-2.8_ft, {.steerer = AB().addImu(0_deg)});

  turn(233_deg);

  roll(loadingPoop);
  move(QuinticHermite(200_deg, {-3.8_ft, -5_ft, -90_deg}, 2.8, 3.5),
       {.curve = true,
        .steerer = AB()
                     .addBallVision(0.0_s, 1.5_s)
                     .addGoalVision(60_pct)
                     .addRoller(rollerStates::top, -200_ms)});

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
  move(QuinticHermite({3.6_ft, -1.2_ft, 0_deg}, 0.8, 1.2),
       {.start_v = 90_pct, .end_v = 0.1_pct, .curve = true});
  Robot::model()->stop();

  move(QuinticHermite(-180_deg, {0.1_ft, 2.4_ft, 45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  roll(on);
  pros::delay(600);
  roll(intake);
  pros::delay(300);
  roll(off);
  asyncTask(pros::delay(700); roll(out););

  // back up
  move(QuinticHermite(45_deg, {3.5_ft, 3.9_ft, 0_deg}), {.curve = true, .start = -135_deg});

  turn(96_deg);

  roll(intake);
  move(QuinticHermite(70_deg, {-3.2_ft, 2.1_ft, 180_deg}, 1.7, 3),
       {.curve = true,
        .steerer =
          AB().addGoalVision(80_pct).addImu(180_deg, 1.6_s).addRoller(rollerStates::on, -200_ms)});
  Robot::model()->stop();

  roll(shoot);
  pros::delay(600);
  roll(off);

  drive(-2_ft,
        {.rotator = AB()
                      .add(makeAngler(-90_deg, Limits<QAngle>(0.3_s, 60_deg / second)), 0_s, 70_pct)
                      .addImu(90_deg, 70_pct)});

  roll(loadingPoop);
  drive(4_ft, {.steerer = AB().addBallVision(0_s, 70_pct)});

  move(Line({-2.1_ft, -0.7_ft}), {.start = 90_deg, .rotator = AB().addImu(180_deg, 0_pct)});

  move(QuinticHermite(-180_deg, {-0.1_ft, -2.2_ft, -45_deg}, 1.1),
       {.start = 0_deg,
        .rotator = makeAngler(-39_deg, Limits<QAngle>(0.5_s, 60_deg / second)),
        .steerer = AB().addRoller(rollerStates::on, -300_ms)});

  roll(on);
  pros::delay(500);
  roll(topIntake);
  pros::delay(50);
  roll(off);

  asyncTask(pros::delay(700); roll(out););

  move(QuinticHermite({0_ft, 0_ft, 135_deg}, {-3.5_ft, 3.5_ft, 180_deg}),
       {.curve = true, .start = -45_deg});
}
