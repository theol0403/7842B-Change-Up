#include "main.h"

/**
 * Constants
 */
extern const QLength clawOffset;

extern const QLength cubeWidth;
extern const QLength fieldWidth;
extern const QLength towerBaseWidth;
extern const QLength zoneWidth;
extern const QLength bigZoneLength;
extern const QLength barrierWidth;

extern const Vector closeTower;
extern const Vector leftTower;
extern const Vector middleTower;
extern const Vector rightTower;

/**
 * Calculate position of robot for claw to be at given position and angle
 *
 * @param  state The state
 * @return The robot position
 */
Vector toClaw(const State& state);

/**
 * Mirror the given coordinates across the field
 */
QAngle mirror(const QAngle& angle);
QLength mirror(const QLength& y);
Vector mirror(const Vector& point);
State mirror(const State& state);