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

  // base
  std::shared_ptr<ThreeEncoderXDriveModel> _model {nullptr};
  std::shared_ptr<CustomOdometry> _odom {nullptr};
  std::shared_ptr<OdomXController> _controller {nullptr};
  std::shared_ptr<PathFollower> _follower {nullptr};

  // devices
  std::shared_ptr<AbstractMotor> _rollers;
  std::shared_ptr<AbstractMotor> _tipper;
  std::shared_ptr<AbstractMotor> _arm;

  // screen
  std::shared_ptr<GUI::Screen> _screen {nullptr};
  GUI::Selector* _selector {nullptr};

public:
  static Robot& get();
  static Robot& initialize();

  //base
  static std::shared_ptr<ThreeEncoderXDriveModel> model();
  static std::shared_ptr<CustomOdometry> odom();
  static std::shared_ptr<OdomXController> chassis();
  static std::shared_ptr<PathFollower> follower();

  //devices
  static std::shared_ptr<AbstractMotor> rollers();
  static std::shared_ptr<AbstractMotor> tipper();
  static std::shared_ptr<AbstractMotor> arm();

  //screen
  static GUI::Selector* selector();

  static void deploy();
};