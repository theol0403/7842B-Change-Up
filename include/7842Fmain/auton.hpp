#pragma once
#include "config.hpp"
#include "main.h"

#define asyncTask(x) pros::Task([&]() { x });

// shorthand
#define move Robot::chassis()
#define roll(x) Robot::roller()->setNewState(rollerStates::x)

// drive a certain distance
inline void drive(const QLength& m) {
  Robot::chassis()->strafe(Line({0_m, 0_m}, {0_m, m}));
}

inline void driveBall(const QLength& m, const Number& vision) {
  Robot::chassis()->strafeBall(Line({0_m, 0_m}, {0_m, m}), vision);
}

inline void turn(const QAngle& a) {
  Robot::imu()->turn(a);
}