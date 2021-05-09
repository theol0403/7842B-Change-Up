#pragma once
#include "main.h"
#include <variant>

namespace lib7842 {
class Injector {
public:
  constexpr Injector() = default;

  using Marker = std::variant<nullptr_t, Number, QLength, QTime>;

  Injector&& add(const Rotator& rotator, const Marker& start = {}, const Marker& end = {});
  Injector&& addBallVision(const Marker& start = {}, const Marker& end = -0.7_ft);
  Injector&& addGoalVision(const Marker& start = 80_pct, const Marker& end = -1.5_ft);
  Injector&& addImu(const QAngle& a, const Marker& start = {}, const Marker& end = {});
  Injector&& addRoller(const rollerStates& roller, const Marker& start = {},
                       const Marker& end = {});

  Rotator build();

protected:
  std::vector<Rotator> rotators {};
};
} // namespace lib7842
