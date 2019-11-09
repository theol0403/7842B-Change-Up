#pragma once
#include "main.h"
#include "statemachine.hpp"

enum class liftStates { off, holdCurrentPos, holdAtPos, up, down, bottom };

class Lift : public StateMachine<liftStates> {

 public:
  Lift(
    const std::unique_ptr<Motor>& ileftLift,
    const std::unique_ptr<Motor>& irightLift,
    const std::unique_ptr<IterativePosPIDController>& ilpid,
    const std::unique_ptr<IterativePosPIDController>& irpid);

  double getLAngle() const;
  double getRAngle() const;

  double setLHoldPos(double iholdPos);
  double setRHoldPos(double iholdPos);

 protected:
  void calibrate() override;
  void loop() override;

  std::unique_ptr<Motor> leftLift {nullptr};
  std::unique_ptr<Motor> rightLift {nullptr};
  std::unique_ptr<IterativePosPIDController> lpid {nullptr};
  std::unique_ptr<IterativePosPIDController> rpid {nullptr};

  double lstartAngle = 0;
  double rstartAngle = 0;

  double lholdPos = 0;
  double rholdPos = 0;
};