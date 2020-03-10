#include "tray.hpp"

Tray::Tray(const std::shared_ptr<AbstractMotor>& imotor,
           const std::shared_ptr<RotarySensor>& isensor) :
  motor(imotor), sensor(isensor) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Tray");
}

void Tray::setPosition(double ipos) {
  targetPos = ipos;
}

double Tray::getPosition() const {
  return getRawPosition() - startPos;
}

double Tray::getError() const {
  return std::abs(targetPos - getPosition());
}

std::shared_ptr<AbstractMotor> Tray::getMotor() const {
  return motor;
}

double Tray::getRawPosition() const {
  return sensor->get();
}

void Tray::initialize() {
  motor->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  motor->setBrakeMode(AbstractMotor::brakeMode::coast);

  startPos = getRawPosition();
}

void Tray::loop() {

  Rate rate;

  while (true) {

    switch (state) {

      case trayStates::off:
        motor->setBrakeMode(AbstractMotor::brakeMode::coast);
        motor->moveVoltage(0);
        break;

      case trayStates::brake:
        motor->setBrakeMode(AbstractMotor::brakeMode::brake);
        motor->moveVelocity(0);
        break;

      case trayStates::up: motor->moveVoltage(12000); break;

      case trayStates::down: motor->moveVoltage(-12000); break;

      case trayStates::calibrate:
        do {
          motor->moveVoltage(-5000);
          pros::delay(200);
        } while (std::abs(motor->getActualVelocity()) > 30);
        pros::delay(200);
        startPos = getRawPosition();
        state = trayStates::off;
        break;
    }

    // if (timer.repeat(100_ms)) {
    //   std::cout << "L: " << getPosition()[0] << ", R: " << getPosition()[1] << std::endl;
    // }

    rate.delayUntil(5_ms);
  }
}