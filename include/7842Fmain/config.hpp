#pragma once
#include "main.h"

#include "subsystem/lift.hpp"
#include "subsystem/claw.hpp"

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
  std::shared_ptr<OdomController> _controller {nullptr};

  std::shared_ptr<Lift> _lift {nullptr};
  std::shared_ptr<Claw> _clawLeft {nullptr};
  std::shared_ptr<Claw> _clawRight {nullptr};

  std::shared_ptr<Screen> _screen {nullptr};

 public:
  static Robot& get();
  static Robot& initialize();
  static std::shared_ptr<ThreeEncoderXDriveModel> model();
  static std::shared_ptr<CustomOdometry> odom();
  static std::shared_ptr<OdomController> chassis();
  static std::shared_ptr<Lift> lift();
  static std::shared_ptr<Claw> clawLeft();
  static std::shared_ptr<Claw> clawRight();
};