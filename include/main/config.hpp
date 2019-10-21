#pragma once
#include "main.h"
#include "utility/threeEncXDriveModel.hpp"
#include "odomDebug/odomDebug.hpp"

class Robot {
 public:
  static Robot& get();
  static Robot& initialize();

  std::shared_ptr<ThreeEncXDriveModel> getModel();
  std::shared_ptr<ThreeEncoderOdometry> getOdom();

  void updateScreen();

 protected:
  void initializeChassis();
  void initializeDevices();
  void initializeScreen();

  std::shared_ptr<ThreeEncXDriveModel> model {nullptr};
  std::shared_ptr<ThreeEncoderOdometry> odom {nullptr};
  std::shared_ptr<OdomDebug> odomScreen {nullptr};

 private:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;
};