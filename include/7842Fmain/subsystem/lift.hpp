#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

enum class liftStates {
  off, // all motors off
  up, // full voltage up
  down, /// full voltage down
  upSlow, // low velocity up
  downSlow, // low velocity down
  brake, // brake for a small about of time, record position, then hold
  holdAtPos, // pid to hold desired position
  aboveCube, // pid to above the cube
  calibrate, // calibrate the lift position
};

class Lift : public StateMachine<liftStates, liftStates::brake> {
public:
  /**
   * Constructs a new lift instance.
   *
   * @param ileftMotor   The left motor
   * @param irightMotor  The right motor
   * @param ileftSensor  The left sensor
   * @param irightSensor The right sensor
   * @param ilpid        The lpid
   * @param irpid        The rpid
   */
  Lift(const std::shared_ptr<AbstractMotor>& ileftMotor,
       const std::shared_ptr<AbstractMotor>& irightMotor,
       const std::shared_ptr<RotarySensor>& ileftSensor,
       const std::shared_ptr<RotarySensor>& irightSensor,
       const std::shared_ptr<IterativePosPIDController>& ilpid,
       const std::shared_ptr<IterativePosPIDController>& irpid);

  /**
   * Sets the desired lift position.
   *
   * @param ipos The position
   */
  void setPosition(const std::valarray<double>& ipos);

  /**
   * Gets the current lift position.
   *
   * @return The position.
   */
  std::valarray<double> getPosition() const;

  /**
   * Gets the position error.
   *
   * @return The error.
   */
  double getError() const;

  std::shared_ptr<AbstractMotor> getLeftMotor() const;
  std::shared_ptr<AbstractMotor> getRightMotor() const;

  void setPowerWithBrake(const std::valarray<double>& power);

protected:
  std::valarray<double> getRawPosition() const;

  void initialize() override;
  void loop() override;

  std::array<std::shared_ptr<AbstractMotor>, 2> motors {nullptr, nullptr};
  std::array<std::shared_ptr<RotarySensor>, 2> sensors {nullptr, nullptr};
  std::array<std::shared_ptr<IterativePosPIDController>, 2> pids {nullptr, nullptr};

  std::valarray<double> startPos {0, 0};
  std::valarray<double> holdPos {0, 0};
};