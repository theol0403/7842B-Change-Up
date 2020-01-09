#include "odomHelper.hpp"

using namespace lib7842::units;

const QLength clawOffset {1_ft}; // TODO: adjust claw offset

const QLength cubeWidth {5.5_in};
const QLength cubeHalf {cubeWidth / 2};
const QLength fieldWidth {6_tile};
const QLength zoneWidth {10_in};
const QLength bigZoneLength {15.5_in};
const QLength barrierWidth {2_in};

const Vector closeTower {1_tile, 3_tile};
const Vector leftTower {3_tile, 4.5_tile};
const Vector middleTower {3_tile, 3_tile};
const Vector rightTower {3_tile, 1.5_tile};
const Vector allianceTower {1.5_tile, 0_tile};

const Vector innerProtectedCube {1_tile + cubeHalf, 5_tile + cubeHalf};
const Vector outerProtectedCube {1_tile + cubeHalf, 4_tile + cubeHalf};

const Vector fourStackCube {2_tile + cubeHalf, 4_tile + cubeHalf};

Vector toClaw(const State& state) {
  return Vector(state) - Vector {std::sin(state.theta.convert(radian)) * clawOffset,
                                 std::cos(state.theta.convert(radian)) * clawOffset};
}