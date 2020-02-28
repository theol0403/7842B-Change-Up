#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class clawStates {
  off, // motor 0 voltage
  brake, // motor 0 velocity
  close, // motor 100% voltage
  open, // motor -100% voltage
  openMedium, // motor -80% voltage
  clamp, // constant close voltage
  closeMedium, // hold close
  // openGrab, // open to grab pos
};

class Claw : public StateMachine<clawStates, clawStates::brake> {
public:
  /**
   * Constructs a new claw instance.
   *
   * @param iclaw The claw motor
   */
  Claw(const std::shared_ptr<AbstractMotor>& iclaw);

  /**
   * Gets the claw motor.
   *
   * @return The motor.
   */
  std::shared_ptr<AbstractMotor> getMotor() const;

protected:
  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> claw {nullptr};
};