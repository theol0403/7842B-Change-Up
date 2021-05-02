#pragma once
#include "lib7842/api/trajectory/generator/generator.hpp"
#include "lib7842/api/trajectory/generator/xGenerator.hpp"

namespace lib7842 {

struct VisionFlags {
  Number ball {105_pct}; // how far along the path before vision seeking
  Number goal {105_pct}; // how far along the path before goal seeking

  const QLength ballCruise {2_ft}; // how much distance to seek the ball
  const QLength ballStop {0.7_ft}; // how far from the end of the path to seek the ball
  // how far from the end of the path to seek the ball while goal seeking while goal seeking
  const QLength goalStop {1.5_ft};
};

Rotator makeVisionRotator(const VisionFlags& flags, const Rotator& other = makeRotator(0_rpm));

inline Rotator makeVisionRotator(const VisionFlags& flags, const QAngularSpeed& speed) {
  return makeVisionRotator(flags, makeRotator(speed));
}

inline Rotator makeVisionRotator(const VisionFlags& flags, const QAngle& angle,
                                 const Limits<QAngle>& limits) {
  return makeVisionRotator(flags, makeRotator(angle, limits));
}

} // namespace lib7842
