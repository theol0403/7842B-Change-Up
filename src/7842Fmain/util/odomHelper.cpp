#include "odomHelper.hpp"

using namespace lib7842::units;

const QLength clawOffset {1_ft}; // TODO: adjust claw offset

const QLength cubeWidth {5.5_in};
const QLength cubeHalf {cubeWidth / 2};
const QLength fieldWidth {6_tile};
const QLength towerBaseWidth {7.8_in};
const QLength zoneWidth {10_in};
const QLength bigZoneLength {15.5_in};
const QLength barrierWidth {2_in};

const Vector closeTower = {3_tile, 1_tile};
const Vector leftTower = {1.5_tile, 3_tile};
const Vector middleTower = {3_tile, 3_tile};
const Vector rightTower = {4.5_tile, 3_tile};
const Vector allianceTower = {fieldWidth, 1.5_tile};

const Vector innerProtectedCube = {1_tile - cubeHalf, 1_tile + cubeHalf};
const Vector outerProtectedCube = {2_tile - cubeHalf, 1_tile + cubeHalf};

const Vector fourStackCube = {2_tile - cubeHalf, 2_tile + cubeHalf};

Vector toClaw(const State& state) {
  return Vector(state)
         - Vector {std::sin(state.theta.convert(radian)) * clawOffset,
                   std::cos(state.theta.convert(radian)) * clawOffset};
}

QAngle mirror(const QAngle& angle) {
  return angle * -1;
}

QLength mirror(const QLength& y) {
  return 12_ft - y;
}

Vector mirror(const Vector& point) {
  return {point.x, mirror(point.y)};
}

State mirror(const State& state) {
  return {state.x, mirror(state.y), mirror(state.theta)};
}