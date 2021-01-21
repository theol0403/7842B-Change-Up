#include "main.h"
#include "config.hpp"
#include "driver.hpp"

void initialize() {
  pros::delay(200);
  Robot::initialize();
}

void disabled() {}
void competition_initialize() {}

void autonomous() {}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
