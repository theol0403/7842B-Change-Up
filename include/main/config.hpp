#pragma once
#include "main.h"
#include "utility/threeEncXDriveModel.hpp"
#include "odomDebug/odomDebug.hpp"

class Robot {
 public:
  static Robot& get();
  static Robot& initialize();
  static Robot& update();

  static std::shared_ptr<ThreeEncXDriveModel> getModel();
  static std::shared_ptr<ThreeEncoderOdometry> getOdom();

 protected:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;

  void initializeChassis();
  void initializeDevices();
  void initializeScreen();

  void updateOdom();
  void updateScreen();

  std::shared_ptr<ThreeEncXDriveModel> model {nullptr};
  std::shared_ptr<ThreeEncoderOdometry> odom {nullptr};
  std::shared_ptr<OdomDebug> odomScreen {nullptr};
};