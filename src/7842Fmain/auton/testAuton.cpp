#include "7842Fmain/auton.hpp"

void testAuton(const std::shared_ptr<SideController>& controller) {
  auto&& [chassis, side] = getChassis();

  spikeFourStack();
}