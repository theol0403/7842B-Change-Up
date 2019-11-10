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

/***
 *    ______                  _____             _             _ 
 *    | ___ \                /  __ \           | |           | |
 *    | |_/ / __ _ ___  ___  | /  \/ ___  _ __ | |_ _ __ ___ | |
 *    | ___ \/ _` / __|/ _ \ | |    / _ \| '_ \| __| '__/ _ \| |
 *    | |_/ / (_| \__ \  __/ | \__/\ (_) | | | | |_| | | (_) | |
 *    \____/ \__,_|___/\___|  \____/\___/|_| |_|\__|_|  \___/|_|
 */
void driverBaseControl() {
  Robot::model()->xArcade(mAnalog(RIGHT_X), mAnalog(RIGHT_Y), mAnalog(LEFT_X));

  // if (mDigital(A)) autonomous();
}

/***
 *    ______           _            _____             _             _ 
 *    |  _  \         (_)          /  __ \           | |           | |
 *    | | | |_____   ___  ___ ___  | /  \/ ___  _ __ | |_ _ __ ___ | |
 *    | | | / _ \ \ / / |/ __/ _ \ | |    / _ \| '_ \| __| '__/ _ \| |
 *    | |/ /  __/\ V /| | (_|  __/ | \__/\ (_) | | | | |_| | | (_) | |
 *    |___/ \___| \_/ |_|\___\___|  \____/\___/|_| |_|\__|_|  \___/|_|
 */
static liftStates liftState = liftStates::hold;
static liftStates lastLiftState = liftStates::hold;

static clawStates leftClawState = clawStates::hold;
static clawStates lastLeftClawState = clawStates::hold;

// static clawStates rightClawState = clawStates::hold;
// static clawStates lastRightClawState = clawStates::hold;

void driverDeviceControl() {

  /***
   *     _     _  __ _   
   *    | |   (_)/ _| |  
   *    | |    _| |_| |_ 
   *    | |   | |  _| __|
   *    | |___| | | | |_ 
   *    \_____/_|_|  \__|
   */
  bool moveSlow = mDigital(Y) || mDigital(A);

  if (mDigital(X)) {
    liftState = moveSlow ? liftStates::upSlow : liftStates::up;
  } else if (mDigital(B)) {
    liftState = moveSlow ? liftStates::downSlow : liftStates::down;
  } else if (mDigital(Y)) {
    liftState = liftStates::downMedium;
  } else if (mDigital(A)) {
    liftState = liftStates::upMedium;
  } else {
    liftState = liftStates::hold;
  }

  if (liftState != lastLiftState) {
    Robot::lift()->setState(liftState);
    lastLiftState = liftState;
  }

  /***
   *     _____ _                
   *    /  __ \ |               
   *    | /  \/ | __ ___      __
   *    | |   | |/ _` \ \ /\ / /
   *    | \__/\ | (_| |\ V  V / 
   *     \____/_|\__,_| \_/\_/  
   */
  // left
  // if (mDigital(L2) && mDigital(L1)) {
  //   leftClawState = clawStates::off;
  // } else
  if (mDigital(L2)) {
    leftClawState = clawStates::clamp;
  } else if (mDigital(L1)) {
    leftClawState = clawStates::release;
  } else {
    leftClawState = clawStates::hold;
  }

  if (leftClawState != lastLeftClawState) {
    Robot::clawLeft()->setState(leftClawState);
    lastLeftClawState = leftClawState;
  }

  // // right
  // if (mDigital(R2) && mDigital(R1)) {
  //   rightClawState = clawStates::off;
  // } else if (mDigital(R2)) {
  //   rightClawState = clawStates::clamp;
  // } else if (mDigital(R1)) {
  //   rightClawState = clawStates::release;
  // } else {
  //   rightClawState = clawStates::hold;
  // }

  // if (rightClawState != lastRightClawState) {
  //   Robot::clawRight()->setState(rightClawState);
  //   lastRightClawState = rightClawState;
  // }
}