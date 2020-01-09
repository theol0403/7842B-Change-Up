#include "main.h"
#include "config.hpp"
#include "driver.hpp"

void initialize() {
  pros::delay(200);
  // Logger::setDefaultLogger(
  //   std::make_shared<Logger>(std::make_unique<Timer>(), "/usd/log", Logger::LogLevel::debug));

  Robot::initialize();
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
  Robot::selector()->run();
}

void opcontrol() {

  while (true) {

    driverBaseControl();
    driverDeviceControl();

    pros::delay(10);
  }
}
