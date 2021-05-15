#include "7842Fmain/subsystem/angler.hpp"
#include "7842Fmain/config.hpp"

namespace lib7842 {
AnglerBuilder&& AnglerBuilder::add(const Angler& angler, const Marker& start, const Marker& end) {
  anglers.emplace_back(std::make_tuple(angler, start, end));
  return std::move(*this);
}

AnglerBuilder&& AnglerBuilder::addBallVision(const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& /*state*/) {
      return Robot::vision()->getOffset() * -0.7_deg / second;
    },
    start, end);
  return std::move(*this);
}

AnglerBuilder&& AnglerBuilder::addGoalVision(const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& /*state*/) {
      return Robot::vision()->getBlueOffset() * -0.8_deg / second;
    },
    start, end);
  return std::move(*this);
}

AnglerBuilder&& AnglerBuilder::addLineStrafe(const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& /*state*/) {
      return Robot::line()->getStrafe() * -0.023_deg / second;
    },
    start, end);
  return std::move(*this);
}

AnglerBuilder&& AnglerBuilder::addImu(const QAngle& a, const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& /*state*/) {
      auto error = util::rollAngle180(a - Robot::imu()->get());
      return Robot::imu()->pid->step(-error.convert(degree)) * rpm * 20;
    },
    start, end);
  return std::move(*this);
}

AnglerBuilder&& AnglerBuilder::addRoller(const rollerStates& roller, const Marker& start,
                                         const Marker& end) {
  add(
    [=](const Profile<>::State& /*state*/) {
      Robot::roller()->setNewState(roller);
      return 0_rpm;
    },
    start, end);
  return std::move(*this);
}

AnglerBuilder::operator Angler() {
  return [anglers = std::move(anglers)](const Profile<>::State& state) {
    return std::accumulate(anglers.begin(), anglers.end(), 0_rpm,
                           [&](const QAngularSpeed& w, const auto& tuple) {
                             auto& angler = std::get<0>(tuple);
                             auto& start = std::get<1>(tuple);
                             auto& end = std::get<2>(tuple);

                             Number pct {};
                             QLength d {};
                             QTime t {};

                             bool afterStart = false;
                             switch (start.index()) {
                               case 0: afterStart = true; break;
                               case 1:
                                 pct = std::get<Number>(start);
                                 if (pct < 0_pct) { pct = 100_pct + pct; }
                                 if (state.d / state.length >= pct) afterStart = true;
                                 break;
                               case 2:
                                 d = std::get<QLength>(start);
                                 if (d < 0_m) { d = state.length + d; }
                                 if (state.d >= d) afterStart = true;
                                 break;
                               case 3:
                                 t = std::get<QTime>(start);
                                 if (t < 0_s) { t = state.time + t; }
                                 if (state.t >= t) afterStart = true;
                                 break;
                             }

                             bool beforeEnd = false;
                             switch (end.index()) {
                               case 0: beforeEnd = true; break;
                               case 1:
                                 pct = std::get<Number>(end);
                                 if (pct < 0_pct) { pct = 100_pct + pct; }
                                 if (state.d / state.length <= pct) beforeEnd = true;
                                 break;
                               case 2:
                                 d = std::get<QLength>(end);
                                 if (d < 0_m) { d = state.length + d; }
                                 if (state.d <= d) beforeEnd = true;
                                 break;
                               case 3:
                                 t = std::get<QTime>(end);
                                 if (t < 0_s) { t = state.time + t; }
                                 if (state.t <= t) beforeEnd = true;
                                 break;
                             }

                             if (afterStart && beforeEnd) { return w + angler(state); }
                             return w;
                           });
  };
}

AnglerBuilder::operator Strafer() {
  return [angler = Angler(*this)](const Profile<>::State& state) {
    return std::clamp(angler(state) * 0.8_s, -40_deg, 40_deg);
  };
}

} // namespace lib7842
