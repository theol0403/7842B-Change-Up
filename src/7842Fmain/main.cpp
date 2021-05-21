#include "main.h"
#include "driver.hpp"

void disabled() {}
void competition_initialize() {}

void initialize() {
  pros::delay(100);
  Robot::initialize();
}

void opcontrol() {
  while (true) {
    driverBaseControl();
    driverDeviceControl();
    pros::delay(10);
  }
}
