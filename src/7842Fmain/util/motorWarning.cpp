#include "motorWarning.hpp"

void MotorWarning::addMotor(const std::shared_ptr<Motor>& imotor, const std::string& iname) {
  motors.emplace_back(imotor, iname);
}

void MotorWarning::loop() {
  while (true) {
    for (auto&& [motor, name] : motors) {
      if (motor->isOverTemp()) LOG_WARN(name + " is over temp");
    }
    pros::delay(20);
  }
}