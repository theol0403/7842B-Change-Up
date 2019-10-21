#include "main.h"
#include "odomDebug/odomDebug.hpp"
#include "utility/threeEncXDriveModel.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

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
void autonomous() {}

void opcontrol() {

  auto model = std::make_shared<ThreeEncXDriveModel>(
    // motors
    std::make_shared<Motor>(1), //
    std::make_shared<Motor>(2), //
    std::make_shared<Motor>(3), //
    std::make_shared<Motor>(4), //
    // sensors
    std::make_shared<ADIEncoder>(3, 4, true), //
    std::make_shared<ADIEncoder>(5, 6), //
    std::make_shared<ADIEncoder>(1, 2, true), //
    // limits
    200, 12000);

  auto odometry = std::make_shared<ThreeEncoderOdometry>(
    TimeUtilFactory::create(), model, ChassisScales({2.75_in, 13.2_in, 0.001_in}, 360));

  Controller controller(ControllerId::master);

  OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
  display.setStateCallback([&](OdomDebug::state_t state) {
    odometry->setState({state.x, state.y, state.theta}, StateMode::CARTESIAN);
  });

  display.setResetCallback([&]() {
    model->resetSensors();
    odometry->setState({0_in, 0_in, 0_deg}, StateMode::CARTESIAN);
  });

  while (true) {
    auto state = odometry->getState(StateMode::CARTESIAN);
    auto sensors = model->getSensorVals();
    display.setData(
      {state.x, state.y, state.theta},
      {(double)sensors[0], (double)sensors[1], (double)sensors[2]});

    model->xArcade(
      controller.getAnalog(ControllerAnalog::rightX),
      controller.getAnalog(ControllerAnalog::rightY),
      controller.getAnalog(ControllerAnalog::leftX));

    pros::delay(20);
  }
}
