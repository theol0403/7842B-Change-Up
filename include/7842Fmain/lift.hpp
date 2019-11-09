#pragma once
#include "main.h"
#include "statemachine.hpp"

enum class liftStates { off, up, down, upSlow, downSlow, hold, holdAtPos, bottom };

class Lift : public StateMachine<liftStates> {

 public:
  Lift(
    std::unique_ptr<Motor>&& ileftLift,
    std::unique_ptr<Motor>&& irightLift,
    std::unique_ptr<IterativePosPIDController>&& ilpid,
    std::unique_ptr<IterativePosPIDController>&& irpid);

  void setPosition(const std::valarray<double>& ipos);
  std::valarray<double> getPosition() const;

 protected:
  void calibrate() override;
  void loop() override;

  std::array<std::unique_ptr<Motor>, 2> lift {nullptr, nullptr};
  std::array<std::unique_ptr<IterativePosPIDController>, 2> pid {nullptr, nullptr};

  std::valarray<double> startPos {0, 0};
  std::valarray<double> holdPos {0, 0};
};