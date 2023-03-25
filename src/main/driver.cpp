#include "driver.hpp"
#include "lib7842/api/other/utility.hpp"
using namespace lib7842;

#define master(x)                                                                                  \
  pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_##x)

#define partner(x)                                                                                 \
  pros::c::controller_get_digital(pros::E_CONTROLLER_PARTNER, pros::E_CONTROLLER_DIGITAL_##x)

#define either(x) (master(x) || partner(x))

#define mDigitalPressed(x)                                                                         \
  pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER,                             \
                                            pros::E_CONTROLLER_DIGITAL_##x)

#define mAnalog(x)                                                                                 \
  static_cast<double>(                                                                             \
    pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_##x)) /    \
    127.0

#define system(device, state) Robot::device()->setNewState(device##States::state);

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

  // if (master(DOWN)) {
  //   rightX += Robot::vision()->getOffset() * 0.015;
  //   rightX = rightX * rightX * util::sgn(rightX);
  // }
  // if (master(DOWN)) {
  //   rightX += Robot::line()->getStrafe() * 0.001;
  //   rightX = rightX * rightX * util::sgn(rightX);

  //   // leftX += Robot::line()->getRotation() * 0.0017;
  //   // leftX = leftX * leftX * util::sgn(leftX);
  // }

  // Robot::model()->xArcade(std::pow(rightX, 2) * util::sgn(rightX),
  //                         std::pow(rightY, 2) * util::sgn(rightY), std::pow(leftX, 3));

  Robot::model()->xArcade(rightX, rightY, leftX);

  if (master(X) && master(UP) && !pros::competition::is_connected()) autonomous();
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

  // initial state
  rollerStates state {rollerStates::off};

  // if L2, add outtake
  if (either(L2)) { state |= rollerStates::out; }

  // if R2, add intake
  if (master(R2)) { state |= rollerStates::intake; }

  // if any of the partner buttons are pressed, the poop is removed. Shoot can still be added later.
  if (partner(R2) && partner(R1)) {
    state = rollerStates::off;
  } else if (partner(R2)) {
    state = rollerStates::intake;
  } else if (partner(L2) || master(RIGHT) || partner(R1)) {
    // don't turn on poop
  } else {
    state |= rollerStates::poop;
  }

  // if R1, add shoot
  if (master(R1)) { state |= rollerStates::shoot; }

  // add actions
  if (master(B) || master(DOWN)) {
    state |= rollerStates::deploy;
  } else if (master(L1)) {
    state |= rollerStates::shootRev;
  } else if (partner(B)) {
    state |= rollerStates::forcePoop;
  }

  Robot::roller()->setNewState(state);

  if (either(Y) && !pros::competition::is_connected()) { Robot::imu()->calibrate(); }
}