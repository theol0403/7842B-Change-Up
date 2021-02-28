#include "7842Fmain/subsystem/vision_drive.hpp"
#include "7842Fmain/config.hpp"

namespace lib7842 {

const QLength visionCruise = 0.7_ft;
const Number visionSpeed = 30_pct;

void XVisionGenerator::strafe(const Spline& spline, const ProfileFlags& flags,
                              const std::optional<Number>& vision) {
  auto visionD = spline.length() * vision.value_or(105_pct);
  auto runner = [&](double t, KinematicState& k) {
    // get the location on the spline
    auto pos = spline.calc(t);
    auto theta = pos.theta;

    if (k.d > visionD && k.d < visionD + visionCruise) {
      theta += Robot::vision()->getOffset() * 0.005_deg;
    }

    // limit the velocity according to path angle.
    // since this is passed by reference it will affect the generator code
    k.v = std::min(k.v, limits.v / (sin(theta).abs() + cos(theta).abs()));

    auto left = k.v * sin(theta + 45_deg);
    auto right = k.v * sin(theta - 45_deg);

    Number topLeftSpeed = Generator::toWheel(left, scales, gearset);
    Number topRightSpeed = Generator::toWheel(right, scales, gearset);

    double topLeft = topLeftSpeed.convert(number);
    double topRight = topRightSpeed.convert(number);

    model->getTopLeftMotor()->moveVoltage(topLeft * 12000);
    model->getTopRightMotor()->moveVoltage(topRight * 12000);
    model->getBottomLeftMotor()->moveVoltage(topRight * 12000);
    model->getBottomRightMotor()->moveVoltage(topLeft * 12000);
  };

  PiecewiseTrapezoidal::Markers markers;
  if (vision) markers.emplace_back(vision.value(), visionSpeed);
  Generator::generate(limits, runner, spline, dt, flags, markers);
}

void XVisionGenerator::curve(const Spline& spline, const ProfileFlags& flags,
                             const std::optional<Number>& vision) {
  auto visionD = spline.length() * vision.value_or(105_pct);
  auto runner = [&](double t, KinematicState& k) {
    // get the curvature along the path
    auto curvature = spline.curvature(t);

    // limit the velocity according to curvature.
    // since this is passed by reference it will affect the generator code
    k.v = std::min(k.v, limits.max_vel_at_curvature(curvature));

    // angular speed is curvature times limited speed
    QAngularSpeed w = curvature * k.v * radian;

    QSpeed left = k.v / std::sqrt(2) - (w / radian * scales.wheelTrack) / 2;
    QSpeed right = k.v / std::sqrt(2) + (w / radian * scales.wheelTrack) / 2;

    Number leftSpeed = Generator::toWheel(left, scales, gearset);
    Number rightSpeed = Generator::toWheel(right, scales, gearset);

    double offset = 0;
    if (k.d > visionD && k.d < visionD + visionCruise) {
      offset = Robot::vision()->getOffset() * 0.0055;
    }

    double leftMotor = leftSpeed.convert(number);
    double rightMotor = rightSpeed.convert(number);

    model->getTopLeftMotor()->moveVoltage((leftMotor + offset) * 12000);
    model->getTopRightMotor()->moveVoltage((rightMotor - offset) * 12000);
    model->getBottomLeftMotor()->moveVoltage((leftMotor - offset) * 12000);
    model->getBottomRightMotor()->moveVoltage((rightMotor + offset) * 12000);
  };

  PiecewiseTrapezoidal::Markers markers;
  if (vision) markers.emplace_back(vision.value(), visionSpeed);
  Generator::generate(limits, runner, spline, dt, flags, markers);
}

} // namespace lib7842