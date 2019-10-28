#pragma once
#include "main.h"
#include "lib7842/odometry/threeEncXDriveModel.hpp"
#include "lib7842/odometry/customOdometry.hpp"

#include "odomDebug/odomDebug.hpp"

using namespace lib7842;

class Robot {
 public:
  static Robot& get();
  static Robot& initialize();
  static Robot& update();

  static std::shared_ptr<ThreeEncXDriveModel> getModel();
  static std::shared_ptr<CustomOdometry> getOdom();

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
  std::shared_ptr<CustomOdometry> odom {nullptr};
  std::shared_ptr<OdomDebug> odomScreen {nullptr};
};