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

void autonomous() {
  Robot::imu()->reset(90_deg);

  roll(intake);
  move->strafe(Line({0_ft, 0_ft}, {-2_ft, 0_ft}));
  move->curve(Mesh({0_ft, 0_ft, 0_deg}, {1.5_ft, 4_ft, 60_deg}),
              {.ball_seek = 0_pct, .goal_seek = 50_pct});
  drive(-2_ft);

  turn(-79_deg);
  roll(intake);

  /* move->strafe(Line({0_ft, 0_ft}, {0_ft, 4_ft}), {.goal_seek = 40_pct}); */
}

// void cornerGoal() {
//   roll(intake);
//   pros::delay(200);
//   roll(onWithoutPoop);
//   pros::delay(700);
//   roll(poop);
// }

// void autonomous() {

//   Robot::imu().reset();

//   // deploy
//   roll(deploy);
//   pros::delay(500);
//   roll(intake);

//   drive(2.3_ft);
//   turn(135_deg);
//   drive(2.7_ft);

//   // 1 shoot first corner goal
//   roll(intake);
//   pros::delay(200);
//   roll(topOut);
//   pros::delay(400);
//   roll(onWithoutPoop);
//   pros::delay(800);

//   // back up
//   drive(-2.5_ft);
//   roll(purge);
//   pros::delay(200);
//   // to ball
//   turn(-79_deg);
//   roll(intake);
//   driveBall(3.2_ft, 0.6);

//   // 2 to first edge goal
//   turn(172_deg);
//   drive(3.75_ft);
//   // 2 shoot first edge goal
//   roll(on);
//   pros::delay(500);
//   roll(loadingWithoutPoop);

//   // back up
//   drive(-0.8_ft);
//   asyncTask({
//     pros::delay(200);
//     roll(poop);
//   });
//   // to ball
//   turn(-92_deg);
//   roll(intake);
//   driveBall(3.45_ft, 0.2);

//   // 3 to second corner goal
//   turn(-135_deg);
//   drive(1.9_ft);
//   // 3 shoot second corner goal
//   cornerGoal();
//   roll(purge);

//   // back up
//   drive(-1_ft);
//   // to ball
//   turn(-9_deg);
//   roll(intake);
//   driveBall(3.85_ft, 0.6);

//   // 4 to second edge goal
//   turn(-94_deg);
//   drive(1.2_ft);
//   // 4 shoot second edge goal
//   roll(onWithoutPoop);
//   pros::delay(500);
//   roll(poop);

//   // back up
//   drive(-1.6_ft);
//   roll(purge);
//   // to ball
//   turn(-2_deg);
//   roll(intake);
//   driveBall(3.4_ft, 0.5);

//   // 5 to third corner goal
//   turn(-56_deg);
//   drive(3.6_ft);
//   // 5 shoot third corner goal
//   cornerGoal();
//   roll(purge);
//   pros::delay(300);

//   // back up
//   drive(-1.3_ft);
//   roll(poop);
//   // to ball
//   turn(110_deg);
//   roll(intake);
//   driveBall(3.8_ft, 0.6);
//   // 6 to third edge goal
//   turn(-8_deg);
//   drive(3_ft);

//   // 6 shoot third edge goal
//   roll(onWithoutPoop);
//   pros::delay(500);
//   roll(poop);

//   // back up
//   drive(-1_ft);
//   // to ball
//   turn(75_deg);
//   roll(intake);
//   driveBall(3.8_ft, 0.3);
//   // 7 to fourth corner goal
//   turn(37_deg);
//   drive(1.5_ft);

//   // 7 shoot fourth corner goal
//   cornerGoal();

//   // back up
//   drive(-1.5_ft);
//   // to ball
//   turn(170_deg);
//   roll(intake);
//   driveBall(3.8_ft, 0.6);

//   // 8 to fourth edge goal
//   turn(80_deg);
//   drive(1_ft);
//   // 8 shoot fourth edge goal
//   roll(onWithoutPoop);
//   pros::delay(500);

//   // back up
//   drive(-0.5_ft);
//   roll(purge);
//   pros::delay(800);
//   // to ball
//   turn(-101_deg);
//   roll(intake);
//   driveBall(2_ft, 0.4);
//   turn(-101_deg);
//   Robot::model()->xArcade(1, 0, 0);
//   pros::delay(200);
//   Robot::model()->xArcade(0, 0, 0);
//   drive(2_ft);

//   drive(-1_ft);
//   // Robot::model()->xArcade(-1, 0, 0);
//   // pros::delay(300);
//   // Robot::model()->xArcade(0, 0, 0);
//   drive(2_ft);

//   roll(onWithoutPoop);
//   pros::delay(600);
//   drive(-2_ft);
// }
