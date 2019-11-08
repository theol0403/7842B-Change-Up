#pragma once
#include "main.h"

class Lift : public TaskWrapper {

 public:
  Lift(
    const std::shared_ptr<Motor>& ileftLift,
    const std::shared_ptr<Motor>& irightLift,
    const std::shared_ptr<IterativePosPIDController>& ilpid,
    const std::shared_ptr<IterativePosPIDController>& irpid);

  enum class states { off, holdCurrentPos, holdAtPos, up, down, bottom };

  void calibrate();

  void setState(const states& istate);
  const states& getState() const;

  double getLAngle() const;
  double getRAngle() const;

  double setLHoldPos(double iholdPos);
  double setRHoldPos(double iholdPos);

  void loop() override;

 protected:
  std::shared_ptr<Motor> leftLift {nullptr};
  std::shared_ptr<Motor> rightLift {nullptr};
  std::shared_ptr<IterativePosPIDController> lpid {nullptr};
  std::shared_ptr<IterativePosPIDController> rpid {nullptr};

  double lstartAngle = 0;
  double rstartAngle = 0;

  double lholdPos = 0;
  double rholdPos = 0;

  states state = states::off;
};