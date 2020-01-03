#pragma once
#include "main.h"
#include "7842Fmain/util/statemachine.hpp"

enum class intakeStates {
  off, // motors 0 voltage
  brake, // motors 0 velocity
  intake, // motors 100% voltage
  open, // motors -80% voltage
  intakeSlow, // motors 80% velocity
  openFull, // motors -100% voltage
};

class Intake : public StateMachine<intakeStates, intakeStates::brake> {
 public:
  /**
   * Constructs a new intake instance.
   *
   * @param ileft  The left motor
   * @param iright The right motor
   */
  Intake(const std::shared_ptr<AbstractMotor>& ileft, const std::shared_ptr<AbstractMotor>& iright);

  /**
   * Get the left motor.
   */
  std::shared_ptr<AbstractMotor> getLeft() const;

  /**
   * Get the right motor.
   */
  std::shared_ptr<AbstractMotor> getRight() const;

 protected:
  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> left {nullptr};
  std::shared_ptr<AbstractMotor> right {nullptr};
};