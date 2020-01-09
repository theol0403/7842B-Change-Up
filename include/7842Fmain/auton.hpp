#include "config.hpp"
#include "util/odomHelper.hpp"

/**
 * Run auton
 *
 * @param auton The auton
 * @param side  The side
 */
inline void runAuton(const std::function<void(const std::shared_ptr<SideController>)> auton,
                     const SideController::sides& side) {
  auton(std::make_shared<SideController>(Robot::chassis(), side));
}

void bigZone(const std::shared_ptr<SideController>& controller);