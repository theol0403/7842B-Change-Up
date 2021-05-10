#pragma once
#include "config.hpp"
#include "main.h"

#define asyncTask(x) pros::Task([&]() { x });

// shorthand
#define move Robot::chassis()->follow
#define roll(x) Robot::roller()->setNewState(rollerStates::x)
#define AB AnglerBuilder

// drive a certain distance
inline void drive(const QLength& m, const XFlags& flags = {}) {
  Robot::chassis()->follow(Line({0_m, 0_m}, {m, 0_m}), flags);
}

inline void turn(const QAngle& a) {
  Robot::imu()->turn(a);
}
