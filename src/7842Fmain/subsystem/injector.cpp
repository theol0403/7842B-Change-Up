#include "7842Fmain/subsystem/injector.hpp"
#include "7842Fmain/config.hpp"

namespace lib7842 {
Injector&& Injector::add(const Rotator& rotator, const Marker& start, const Marker& end) {
  rotators.emplace_back([=](const Profile<>::State& state) {
    bool afterStart = false;
    Number pct {};
    QLength d {};
    QTime t {};
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
    switch (start.index()) {
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

    if (afterStart && beforeEnd) { return rotator(state); }
    return 0_rpm;
  });
  return std::move(*this);
}

Injector&& Injector::addBallVision(const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& state) { return Robot::vision()->getOffset() * -0.7_deg / second; },
    start, end);
}

Injector&& Injector::addGoalVision(const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& state) {
      return Robot::vision()->getBlueOffset() * -0.7_deg / second;
    },
    start, end);
}

Injector&& Injector::addImu(const QAngle& a, const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& state) {
      auto error = util::rollAngle180(
        a - (-1 * Robot::imu()->imu->get_rotation() * degree - Robot::imu()->offset));
      return Robot::imu()->pid->step(-error.convert(degree)) * rpm * 20;
    },
    start, end);
}

Injector&& Injector::addRoller(const rollerStates& roller, const Marker& start, const Marker& end) {
  add(
    [=](const Profile<>::State& state) {
      Robot::roller()->setNewState(roller);
      return 0_rpm;
    },
    start, end);
}

Rotator Injector::build() {
  return [=](const Profile<>::State& state) {
    return std::accumulate(
      rotators.begin(), rotators.end(), 0_rpm,
      [&](const QAngularSpeed& w, const Rotator& rotator) { return w + rotator(state); });
  };
}

} // namespace lib7842
