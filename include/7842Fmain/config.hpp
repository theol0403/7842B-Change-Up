#pragma once
#include "main.h"

using namespace lib7842;

class Robot {
 public:
  static Robot& get();
  static Robot& initialize();
  static Robot& update();

  static std::shared_ptr<ThreeEncoderXDriveModel> getModel();
  static std::shared_ptr<CustomOdometry> getOdom();
  static std::shared_ptr<OdomController> getController();

 protected:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;

  void initializeChassis();
  void initializeDevices();
  void initializeScreen();

  std::shared_ptr<ThreeEncoderXDriveModel> model {nullptr};
  std::shared_ptr<CustomOdometry> odom {nullptr};
  std::shared_ptr<OdomController> controller {nullptr};

  std::shared_ptr<Screen> screen {nullptr};
};