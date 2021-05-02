#include "7842Fmain/subsystem/vision_drive.hpp"
#include "7842Fmain/config.hpp"
#include "pros/rtos.hpp"

namespace lib7842 {

Rotator makeVisionRotator(const VisionFlags& flags, const Rotator& other) {
  return [=](const Profile<>::State& state) {
    auto ballD = state.length * flags.ball; // how far for ball seek
    auto goalD = state.length * flags.goal; // how far for gol seek

    double error = 0;
    if (state.d > ballD && state.d < ballD + flags.ballCruise &&
        state.d < state.length - flags.ballStop) {
      error += Robot::vision()->getOffset();
    }
    if (state.d > goalD && state.d < state.length - flags.goalStop) {
      error += Robot::vision()->getBlueOffset();
    }

    return other(state) + error * 0.001_deg / second;
  };
}

} // namespace lib7842
