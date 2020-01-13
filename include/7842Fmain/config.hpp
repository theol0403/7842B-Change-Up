#pragma once
#include "main.h"

#include "subsystem/claw.hpp"
#include "subsystem/lift.hpp"

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
  std::shared_ptr<OdomXController> _controller {nullptr};
  std::shared_ptr<PathFollower> _follower {nullptr};

  std::shared_ptr<Lift> _lift {nullptr};
  std::shared_ptr<Claw> _claw {nullptr};

  std::shared_ptr<GUI::Screen> _screen {nullptr};
  GUI::Selector* _selector {nullptr};

public:
  static Robot& get();
  static Robot& initialize();
  static std::shared_ptr<ThreeEncoderXDriveModel> model();
  static std::shared_ptr<CustomOdometry> odom();
  static std::shared_ptr<OdomXController> chassis();
  static std::shared_ptr<PathFollower> follower();
  static std::shared_ptr<Lift> lift();
  static std::shared_ptr<Claw> claw();

  static GUI::Selector* selector();

  static void deploy();
};