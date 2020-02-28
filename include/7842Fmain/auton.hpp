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

/**
 * Autons
 */
void bigZone6(const std::shared_ptr<SideController>& controller);
void bigZone7(const std::shared_ptr<SideController>& controller);
void bigZoneNoFourStack(const std::shared_ptr<SideController>& controller);
void testAuton(const std::shared_ptr<SideController>& controller);
void skills(const std::shared_ptr<SideController>& controller);

void slowDown();
void speedUp();

void spikeCube();
void spikeFourStack();
void spikeFourStack(const Timer& timer);
