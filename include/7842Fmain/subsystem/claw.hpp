#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class clawStates { off, close, open, clamp, release, brake };

class Claw : public StateMachine<clawStates, clawStates::brake> {

 public:
  Claw(const std::shared_ptr<Motor>& iclaw, const std::shared_ptr<IterativePosPIDController>& ipid);

  std::shared_ptr<Motor> getMotor() const;

 protected:
  void calibrate() override;
  void loop() override;

  std::shared_ptr<Motor> claw {nullptr};
  std::shared_ptr<IterativePosPIDController> pid {nullptr};

  double startPos {0};
};