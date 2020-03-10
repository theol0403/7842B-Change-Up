#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class trayStates {
  off, // all motors off
  brake, // all motors brake
  up, // full voltage up
  down, // full voltage down
  score, // use velocity mapping to score
  calibrate, // calibrate the tray position
};

class Tray : public StateMachine<trayStates, trayStates::off> {
public:
  /**
   * Constructs a new tray instance.
   *
   * @param imotor  The tray motor
   * @param isensor The tray sensor (can be the integrated encoder)
   * @param ioutPos The tray out position
   */
  Tray(const std::shared_ptr<AbstractMotor>& imotor, const std::shared_ptr<RotarySensor>& isensor,
       double ioutPos);

  /**
   * Sets the desired tray position. Can only move forward.
   *
   * @param ipos The position
   */
  void setPosition(double ipos);

  /**
   * Gets the current tray position.
   *
   * @return The position.
   */
  double getPosition() const;

  /**
   * Gets the position error.
   *
   * @return The error.
   */
  double getError() const;

  /**
   * Get the tray motor.
   *
   * @return The motor.
   */
  std::shared_ptr<AbstractMotor> getMotor() const;

  /**
   * Gets the tray velocity mapping over position.
   * Input and output is in the range [0, 1].
   *
   * @return The velocity.
   */
  double getVelocityMapping();

protected:
  double getRawPosition() const;

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> motor {nullptr};
  std::shared_ptr<RotarySensor> sensor {nullptr};
  std::shared_ptr<IterativePosPIDController> pid {nullptr};

  double outPos {0};
  double startPos {0};
  double targetPos {0};
};