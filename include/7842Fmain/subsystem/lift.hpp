#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class liftStates {
  off,
  up,
  down,
  upMedium,
  downMedium,
  upSlow,
  downSlow,
  hold,
  holdAtPos,
  bottom
};

class Lift : public StateMachine<liftStates, liftStates::hold> {

 public:
  Lift(
    const std::shared_ptr<Motor>& ileftLift,
    const std::shared_ptr<Motor>& irightLift,
    const std::shared_ptr<IterativePosPIDController>& ilpid,
    const std::shared_ptr<IterativePosPIDController>& irpid);

  void setPosition(const std::valarray<double>& ipos);
  std::valarray<double> getPosition() const;

  std::shared_ptr<Motor> getLeftMotor() const;
  std::shared_ptr<Motor> getRightMotor() const;

 protected:
  void calibrate() override;
  void loop() override;

  std::array<std::shared_ptr<Motor>, 2> lift {nullptr, nullptr};
  std::array<std::shared_ptr<IterativePosPIDController>, 2> pid {nullptr, nullptr};

  std::valarray<double> startPos {0, 0};
  std::valarray<double> holdPos {0, 0};
};