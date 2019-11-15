#include "main.h"
#include "config.hpp"
#include "driver.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::delay(200);
  Robot::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void testAuton();

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  // Robot::chassis()->driveDistance(1_ft);
  // Robot::chassis()->turnToAngle(90_deg);
  // Robot::chassis()->driveDistance(1_ft);
  // Robot::chassis()->turnToAngle(180_deg);
  // Robot::chassis()->driveDistance(1_ft);
  // Robot::chassis()->turnToAngle(270_deg);
  // Robot::chassis()->driveDistance(1_ft);
  // Robot::chassis()->turnToAngle(90_deg);
  // Robot::odom()->reset();
  // Robot::chassis()->driveToPoint({2_ft, 2_ft}, 2);
  testAuton();
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
