#include "main.h"
#include "odomDebug/odomDebug.hpp"

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

  auto chassis = ChassisControllerBuilder()
                   // .withMotors(1, 2, 3, 4)
                   .withMotors({1, 2}, {3, 4})
                   .withSensors(ADIEncoder(3, 4, true), ADIEncoder(5, 6), ADIEncoder(1, 2, true))
                   .withDimensions({{2.75_in, 13.2_in, 0.001_in}, 360})
                   .withOdometry(StateMode::CARTESIAN)
                   .buildOdometry();

  // auto xModel = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
  // auto oModel = std::dynamic_pointer_cast<ThreeEncoderSkidSteerModel>(chassis->getModel());
  Controller controller(ControllerId::master);

  OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
  display.setStateCallback([&](OdomDebug::state_t state) {
    chassis->setState({state.x, state.y, state.theta});
  });
  display.setResetCallback([&]() {
    chassis->setState({0_in, 0_in, 0_deg});
  });

  while (true) {

    auto state = chassis->getState();
    auto sensors = chassis->getModel()->getSensorVals();
    display.setData({state.x, state.y, state.theta}, {sensors[0], sensors[1], sensors[2]});

    // xModel->xArcade(
    //   controller.getAnalog(ControllerAnalog::rightX),
    //   controller.getAnalog(ControllerAnalog::rightY),
    //   controller.getAnalog(ControllerAnalog::leftX));

    pros::delay(20);
  }
}
