#pragma once
#include "main.h"

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

  std::shared_ptr<Screen> _screen {nullptr};

 public:
  static Robot& get();
  static Robot& initialize();
  static std::shared_ptr<ThreeEncoderXDriveModel> model();
  static std::shared_ptr<CustomOdometry> odom();
  static std::shared_ptr<OdomController> chassis();
};