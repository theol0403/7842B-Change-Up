#include "odomHelper.hpp"

QLength clawOffset = 0.5_ft; // TODO: adjust claw offset

Vector toClaw(const State& state) {
  return Vector(state)
         - Vector {std::sin(state.theta.convert(radian)) * clawOffset,
                   std::cos(state.theta.convert(radian)) * clawOffset};
}