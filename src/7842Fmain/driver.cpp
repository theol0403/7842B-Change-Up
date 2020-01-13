#include "driver.hpp"

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

  Robot::model()->xArcade(std::pow(rightX, 2) * util::sgn(rightX),
                          std::pow(rightY, 2) * util::sgn(rightY),
                          std::pow(leftX, 2) * util::sgn(leftX));

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
   *     _     _  __ _   
   *    | |   (_)/ _| |  
   *    | |    _| |_| |_ 
   *    | |   | |  _| __|
   *    | |___| | | | |_ 
   *    \_____/_|_|  \__|
   */
  if (mDigital(R1) && mDigital(R2)) {
    setNewDeviceState(lift, aboveCube);
  } else if (mDigital(R2)) {
    setNewDeviceState(lift, down);
  } else if (mDigital(R1)) {
    setNewDeviceState(lift, up);
  } else if (mDigital(Y)) {
    setNewDeviceState(lift, upSlow);
  } else if (mDigital(B)) {
    setNewDeviceState(lift, downSlow);
  } else {
    setNewDeviceState(lift, brakeAndHold);
  }

  /***
   *     _____ _
   *    /  __ \ |
   *    | /  \/ | __ ___      __
   *    | |   | |/ _` \ \ /\ / /
   *    | \__/\ | (_| |\ V  V /
   *     \____/_|\__,_| \_/\_/
   */
  if (mDigital(L2)) {
    setNewDeviceState(claw, close);
  } else if (mDigital(L1)) {
    setNewDeviceState(claw, open);
  } else {
    auto state = Robot::claw()->getState();
    if (state == clawStates::open) {
      setNewDeviceState(claw, brake);
    } else if (state == clawStates::close) {
      setNewDeviceState(claw, clamp);
    }
  }
}