#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class clawStates { off, close, open, clamp, release, hold, holdAtPos };

class Claw : public StateMachine<clawStates> {

 public:
  Claw(std::unique_ptr<Motor>&& iclaw, std::unique_ptr<IterativePosPIDController>&& ipid);

 protected:
  void calibrate() override;
  void loop() override;

  std::unique_ptr<Motor> claw {nullptr};
  std::unique_ptr<IterativePosPIDController> pid {nullptr};

  double startPos {0};
  double holdPos {0};
};