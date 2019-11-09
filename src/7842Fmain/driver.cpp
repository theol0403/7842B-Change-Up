#include "driver.hpp"

#define mDigital(x)                                                                                \
  pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_##x)

#define mDigitalPressed(x)                                                                         \
  pros::c::controller_get_digital_new_press(                                                       \
    pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_##x)

#define mAnalog(x)                                                                                 \
  static_cast<double>(                                                                             \
    pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_##x))      \
    / 127.0

void driverBaseControl() {
  Robot::model()->xArcade(mAnalog(RIGHT_X), mAnalog(RIGHT_Y), mAnalog(LEFT_X));

  if (mDigital(A)) autonomous();
}

static liftStates liftState = liftStates::hold;
static liftStates lastLiftState = liftStates::hold;

void driverDeviceControl() {

  bool moveSlow = mDigital(R1);

  if (mDigital(L2) && mDigital(L1)) {
    liftState = liftStates::bottom;
  } else if (mDigital(L1)) {
    liftState = moveSlow ? liftStates::upSlow : liftStates::up;
  } else if (mDigital(L2)) {
    liftState = moveSlow ? liftStates::downSlow : liftStates::down;
  } else {
    liftState = liftStates::hold;
  }

  if (liftState != lastLiftState) {
    Robot::lift()->setState(liftState);
    lastLiftState = liftState;
  }
}