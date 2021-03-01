#pragma once
#include "config.hpp"
#include "main.h"

#define asyncTask(x) pros::Task([&]() { x });

// shorthand
#define move Robot::chassis()
#define roll(x) Robot::roller()->setNewState(rollerStates::x)

// drive a certain distance
inline void drive(const QLength& m, const ChassisFlags& flags = {}) {
  Robot::chassis()->strafe(Line({0_m, 0_m}, {0_m, m}), flags);
}

inline void turn(const QAngle& a) {
  Robot::imu()->turn(a);
}