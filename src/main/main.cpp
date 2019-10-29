#include "main.h"
#include "main/config.hpp"

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
  // Robot::getController()->driveDistance(1_ft);
  // Robot::getController()->turnToAngle(90_deg);
  // Robot::getController()->driveDistance(1_ft);
  // Robot::getController()->turnToAngle(180_deg);
  // Robot::getController()->driveDistance(1_ft);
  // Robot::getController()->turnToAngle(270_deg);
  // Robot::getController()->driveDistance(1_ft);
  // Robot::getController()->turnToAngle(90_deg);
  // Robot::getOdom()->reset();
  Robot::getController()->driveToPoint({2_ft, 2_ft}, 2);
}

void opcontrol() {

  Controller controller(ControllerId::master);
  Motor lift(11);
  Motor lift2(12);

  while (true) {
    Robot::update();

    Robot::getModel()->xArcade(
      controller.getAnalog(ControllerAnalog::rightX),
      controller.getAnalog(ControllerAnalog::rightY),
      controller.getAnalog(ControllerAnalog::leftX));

    if (controller.getDigital(ControllerDigital::up)) {
      lift.moveVoltage(-1 * 12000);
      lift2.moveVoltage(1 * 12000);
    } else if (controller.getDigital(ControllerDigital::down)) {
      lift.moveVoltage(1 * 12000);
      lift2.moveVoltage(-1 * 12000);
    } else {
      lift.moveVoltage(0);
      lift2.moveVoltage(0);
    }

    if (controller.getDigital(ControllerDigital::A)) { autonomous(); }

    pros::delay(5);
  }
}
