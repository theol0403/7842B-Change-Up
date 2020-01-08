#include "odomHelper.hpp"

using namespace lib7842::units;

const QLength clawOffset {1_ft}; // TODO: adjust claw offset

const QLength cubeWidth {5.5_in};
const QLength fieldWidth {6_tile};
const QLength towerBaseWidth {7.8_in};
const QLength zoneWidth {10_in};
const QLength bigZoneLength {15.5_in};
const QLength barrierWidth {2_in};

const Vector closeTower = {3 * tile, tile};
const Vector leftTower = {1.5 * tile, 3 * tile};
const Vector middleTower = {3 * tile, 3 * tile};
const Vector rightTower = {4.5 * tile, 3 * tile};

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