#include "main.h"
#include "sideController.hpp"

/**
 * Constants
 */
extern const QLength clawOffset;

extern const QLength cubeWidth;
extern const QLength cubeHalf;
extern const QLength fieldWidth;
extern const QLength towerBaseWidth;
extern const QLength zoneWidth;
extern const QLength bigZoneLength;
extern const QLength barrierWidth;

extern const Vector closeTower;
extern const Vector leftTower;
extern const Vector middleTower;
extern const Vector rightTower;
extern const Vector allianceTower;

extern const Vector innerProtectedCube;
extern const Vector outerProtectedCube;

extern const Vector closeTowerCube;
extern const Vector leftTowerCube;

extern const Vector fourStackCube;

extern const PursuitLimits defaultLimits;

/**
 * Utility functions
 */
#define makeAngle(x) SideController::makeAngleCalculator(x, side)
#define makeSettle OdomController::makeSettler

#define mirror SideController::mirror

#define getChassis() std::make_pair(*controller, controller->getSide());

/**
 * Calculate position of robot for claw to be at given position and angle
 *
 * @param  state The state
 * @return The robot position
 */
Vector toClaw(const State& state);

/**
 * Create an async task from a function
 */
#define asyncTask(x) pros::Task([&]() { x });