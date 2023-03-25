#pragma once
#include "main.h"
#include "roller.hpp"
#include <variant>

namespace lib7842 {
class AnglerBuilder {
public:
  constexpr AnglerBuilder() = default;

  using Marker = std::variant<nullptr_t, Number, QLength, QTime>;

  AnglerBuilder&& add(const Angler& angler, const Marker& start = {}, const Marker& end = {});
  AnglerBuilder&& addBallVision(const Marker& start = {}, const Marker& end = -0.7_ft);
  AnglerBuilder&& addGoalVision(const Marker& start = 70_pct, const Marker& end = -1.2_ft);
  AnglerBuilder&& addImu(const QAngle& a, const Marker& start = {}, const Marker& end = {});
  AnglerBuilder&& addRoller(const rollerStates& roller, const Marker& start = {},
                            const Marker& end = {});
  AnglerBuilder&& addLineStrafe(const Marker& start = {}, const Marker& end = {});

  operator Angler();
  operator Strafer();

protected:
  std::vector<std::tuple<Angler, Marker, Marker>> anglers {};
};
} // namespace lib7842
