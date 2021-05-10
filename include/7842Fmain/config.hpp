#pragma once
#include "main.h"

#include "subsystem/angler.hpp"
#include "subsystem/imu.hpp"
#include "subsystem/roller.hpp"
#include "subsystem/vision.hpp"

class Robot {
public:
  static Robot& get();
  static Robot& initialize();

#define ADD(name, type) static std::shared_ptr<type> name();
#include "systems.def"
#undef ADD

protected:
  void _initializeChassis();
  void _initializeDevices();

#define ADD(name, type) std::shared_ptr<type> _##name {nullptr};
#include "systems.def"
#undef ADD
};
