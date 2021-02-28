#pragma once
#include "config.hpp"
#include "main.h"

#define asyncTask(x) pros::Task([&]() { x });
#define roll(x) Robot::roller()->setNewState(rollerStates::x)

// drive a certain distance
inline void drive(const QLength& m) {
  Robot::chassis()->strafe(Line({0_m, 0_m}, {0_m, m}));
}
