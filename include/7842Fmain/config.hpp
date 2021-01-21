#pragma once
#include "main.h"

#include "subsystem/tray.hpp"

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
  // std::shared_ptr<Tray> _tray;
  std::shared_ptr<AbstractMotor> _rollers;
  std::shared_ptr<AbstractMotor> _topRoller;

public:
  static Robot& get();
  static Robot& initialize();

  //base
  static std::shared_ptr<XDriveModel> model();

  //devices
  // static std::shared_ptr<Tray> tray();
  static std::shared_ptr<AbstractMotor> rollers();
  static std::shared_ptr<AbstractMotor> topRoller();
};