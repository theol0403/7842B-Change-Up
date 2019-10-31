#pragma once
#include "main.h"
#include "lib7842/odometry/threeEncXDriveModel.hpp"
#include "lib7842/odometry/customOdometry.hpp"
#include "lib7842/odometry/odomController.hpp"

#include "lib7842/gui/screen.hpp"
#include "lib7842/gui/odomDebug.hpp"

using namespace lib7842;

class Robot {
 public:
  static Robot& get();
  static Robot& initialize();
  static Robot& update();

  static std::shared_ptr<ThreeEncXDriveModel> getModel();
  static std::shared_ptr<CustomOdometry> getOdom();
  static std::shared_ptr<OdomController> getController();

 protected:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;

  void initializeChassis();
  void initializeDevices();
  void initializeScreen();

  std::shared_ptr<ThreeEncXDriveModel> model {nullptr};
  std::shared_ptr<CustomOdometry> odom {nullptr};
  std::shared_ptr<OdomController> controller {nullptr};

  std::shared_ptr<Screen> screen {nullptr};
};