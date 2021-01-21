#include "driver.hpp"
#include "lib7842/api/other/utility.hpp"
using namespace lib7842;

#define mDigital(x)                                                                                \
  pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_##x)

#define mDigitalPressed(x)                                                                         \
  pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER,                             \
                                            pros::E_CONTROLLER_DIGITAL_##x)

#define mAnalog(x)                                                                                 \
  static_cast<double>(                                                                             \
    pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_##x)) /    \
    127.0

#define setNewDeviceState(device, state) Robot::device()->setNewState(device##States::state);

/***
 *    ______                  _____             _             _
 *    | ___ \                /  __ \           | |           | |
 *    | |_/ / __ _ ___  ___  | /  \/ ___  _ __ | |_ _ __ ___ | |
 *    | ___ \/ _` / __|/ _ \ | |    / _ \| '_ \| __| '__/ _ \| |
 *    | |_/ / (_| \__ \  __/ | \__/\ (_) | | | | |_| | | (_) | |
 *    \____/ \__,_|___/\___|  \____/\___/|_| |_|\__|_|  \___/|_|
 */
void driverBaseControl() {
  double rightX = mAnalog(RIGHT_X);
  double rightY = mAnalog(RIGHT_Y);
  double leftX = mAnalog(LEFT_X);

  if (!mDigital(DOWN)) {
    Robot::model()->xArcade(std::pow(rightX, 2) * util::sgn(rightX),
                            std::pow(rightY, 2) * util::sgn(rightY), std::pow(leftX, 3));
  }

  if (mDigital(X) && !pros::competition::is_connected()) autonomous();
}

/***
 *    ______           _            _____             _             _
 *    |  _  \         (_)          /  __ \           | |           | |
 *    | | | |_____   ___  ___ ___  | /  \/ ___  _ __ | |_ _ __ ___ | |
 *    | | | / _ \ \ / / |/ __/ _ \ | |    / _ \| '_ \| __| '__/ _ \| |
 *    | |/ /  __/\ V /| | (_|  __/ | \__/\ (_) | | | | |_| | | (_) | |
 *    |___/ \___| \_/ |_|\___\___|  \____/\___/|_| |_|\__|_|  \___/|_|
 */

void driverDeviceControl() {

  /***
   *    _____               
   *   |_   _|              
   *     | |_ __ __ _ _   _ 
   *     | | '__/ _` | | | |
   *     | | | | (_| | |_| |
   *     \_/_|  \__,_|\__, |
   *                   __/ |
   *                  |___/ 
   */
  // // roller control
  if (mDigital(L2)) {
    Robot::rollers()->moveVoltage(12000); // max voltage 12000 milliamps
  } else if (mDigital(L1)) {
    Robot::rollers()->moveVoltage(-12000);
  } else {
    Robot::rollers()->moveVoltage(0);
  }

  // arm control
  if (mDigital(R2)) {
    Robot::topRoller()->moveVoltage(12000);
  } else if (mDigital(R1)) {
    Robot::topRoller()->moveVoltage(-12000);
  } else {
    Robot::topRoller()->moveVelocity(0);
  }
}
