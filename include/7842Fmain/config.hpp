#pragma once
#include "main.h"

#include "subsystem/roller.hpp"

class Robot {
protected:
  Robot() = default;
  Robot(const Robot& irobot) = delete;
  ~Robot() = default;

  void _initializeChassis();
  void _initializeDevices();

  // base
  std::shared_ptr<XDriveModel> _model {nullptr};

  // devices
  std::shared_ptr<Roller> _roller;

public:
  static Robot& get();
  static Robot& initialize();

  //base
  static std::shared_ptr<XDriveModel> model();

  //devices
  static std::shared_ptr<Roller> roller();
};