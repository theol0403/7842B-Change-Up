#pragma once
#include "main.h"
#include "statemachine.hpp"

enum class liftStates { off, holdCurrentPos, holdAtPos, up, down, bottom };

class Lift : public StateMachine<liftStates> {

 public:
  Lift(
    const std::shared_ptr<Motor>& ileftLift,
    const std::shared_ptr<Motor>& irightLift,
    const std::shared_ptr<IterativePosPIDController>& ilpid,
    const std::shared_ptr<IterativePosPIDController>& irpid);

  double getLAngle() const;
  double getRAngle() const;

  double setLHoldPos(double iholdPos);
  double setRHoldPos(double iholdPos);

 protected:
  void calibrate() override;
  void loop() override;

  std::shared_ptr<Motor> leftLift {nullptr};
  std::shared_ptr<Motor> rightLift {nullptr};
  std::shared_ptr<IterativePosPIDController> lpid {nullptr};
  std::shared_ptr<IterativePosPIDController> rpid {nullptr};

  double lstartAngle = 0;
  double rstartAngle = 0;

  double lholdPos = 0;
  double rholdPos = 0;
};