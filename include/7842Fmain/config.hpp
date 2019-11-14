#pragma once
#include "main.h"

#include "subsystem/lift.hpp"
#include "subsystem/claw.hpp"
#include "util/motorWarning.hpp"

class Robot {
 protected:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;

  void _initializeChassis();
  void _initializeDevices();
  void _initializeScreen();

  std::shared_ptr<ThreeEncoderXDriveModel> _model {nullptr};
  std::shared_ptr<CustomOdometry> _odom {nullptr};
  std::shared_ptr<OdomXController> _controller {nullptr};

  std::shared_ptr<Lift> _lift {nullptr};
  std::shared_ptr<Claw> _clawLeft {nullptr};
  std::shared_ptr<Claw> _clawRight {nullptr};

  std::shared_ptr<Screen> _screen {nullptr};
  std::shared_ptr<MotorWarning> _motorWarning {nullptr};

 public:
  static Robot& get();
  static Robot& initialize();
  static std::shared_ptr<ThreeEncoderXDriveModel> model();
  static std::shared_ptr<CustomOdometry> odom();
  static std::shared_ptr<OdomXController> chassis();
  static std::shared_ptr<Lift> lift();
  static std::shared_ptr<Claw> clawLeft();
  static std::shared_ptr<Claw> clawRight();

  static void deploy();
};