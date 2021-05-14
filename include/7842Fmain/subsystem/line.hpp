#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include <memory>

class LineSensor {
public:
  LineSensor(const std::shared_ptr<pros::ADILineSensor>& itopLeft,
             const std::shared_ptr<pros::ADILineSensor>& itopRight,
             const std::shared_ptr<pros::ADILineSensor>& ibottomRight,
             const std::shared_ptr<pros::ADILineSensor>& ibottomLeft) :
    topLeft(itopLeft), topRight(itopRight), bottomRight(ibottomRight), bottomLeft(ibottomLeft) {
    calibrate();
    /* startTask("Line"); */
  }

  void calibrate() {
    topLeft->calibrate();
    topRight->calibrate();
    bottomRight->calibrate();
    bottomLeft->calibrate();

    topLeftOffset = topLeft->get_value_calibrated();
    topRightOffset = topRight->get_value_calibrated();
    bottomRightOffset = bottomRight->get_value_calibrated();
    bottomLeftOffset = bottomLeft->get_value_calibrated();
  }

  double getFront() {
    return -1 * (topRight->get_value_calibrated() - topRightOffset) -
           -1 * (topLeft->get_value_calibrated() - topLeftOffset);
  }

  double getBack() {
    return -1 * (bottomRight->get_value_calibrated() - bottomRightOffset) -
           -1 * (bottomLeft->get_value_calibrated() - bottomLeftOffset);
  }

  double getRotation() {
    return getFront() - getBack();
  }

  double getStrafe() {
    return (getFront() + getBack()) / 2.0;
  }

  /* void loop() override { */
  /*   while (true) { */
  /*     std::cout << "TopRight: " << topRight->get_value_calibrated() - topRightOffset */
  /*               << " TopLeft: " << topLeft->get_value_calibrated() - topLeftOffset */
  /*               << " BottomRight: " << bottomRight->get_value_calibrated() - bottomRightOffset */
  /*               << " BottomLeft: " << bottomLeft->get_value_calibrated() - bottomLeftOffset */
  /*               << std::endl; */
  /*     std::cout << "Front: " << getFront() << " Back: " << getBack() << std::endl; */
  /*     std::cout << "Rotation: " << getRotation() << ", Strafe: " << getStrafe() << std::endl; */
  /*     pros::delay(20); */
  /*   } */
  /* } */

  std::shared_ptr<pros::ADILineSensor> topLeft;
  std::shared_ptr<pros::ADILineSensor> topRight;
  std::shared_ptr<pros::ADILineSensor> bottomRight;
  std::shared_ptr<pros::ADILineSensor> bottomLeft;

  double topLeftOffset;
  double topRightOffset;
  double bottomRightOffset;
  double bottomLeftOffset;
};
