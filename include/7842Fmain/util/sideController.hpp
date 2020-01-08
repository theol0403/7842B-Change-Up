#include "main.h"

enum class autonSide { red, blue };

/**
 * Mirror the given coordinates across the field
 */
QAngle mirror(const QAngle& angle, const autonSide& side) {
  return side == autonSide::red ? angle : angle * -1;
}
QLength mirror(const QLength& y, const autonSide& side) {
  return side == autonSide::red ? y : 12_ft - y;
}
Vector mirror(const Vector& point, const autonSide& side) {
  return {point.x, mirror(point.y, side)};
}
State mirror(const State& state, const autonSide& side) {
  return {state.x, mirror(state.y, side), mirror(state.theta, side)};
}

class SideController {
public:
  SideController(const std::shared_ptr<OdomXController>& icontroller, const autonSide& iside) :
    controller(icontroller), side(iside) {}

  void turnToAngle(const QAngle& angle, const Turner& turner = OdomController::pointTurn,
                   const Settler& settler = OdomController::defaultTurnSettler) {
    controller->turnToAngle(mirror(angle, side), turner, settler);
  }

  void turnAngle(const QAngle& angle, const Turner& turner = OdomController::pointTurn,
                 const Settler& settler = OdomController::defaultTurnSettler) {
    controller->turnAngle(mirror(angle, side), turner, settler);
  }

  void turnToPoint(const Vector& point, const Turner& turner = OdomController::pointTurn,
                   const Settler& settler = OdomController::defaultTurnSettler) {
    controller->turnToPoint(mirror(point, side), turner, settler);
  }

  void driveToPoint(const Vector& targetPoint, double turnScale = 1,
                    const Settler& settler = OdomController::defaultDriveSettler) {
    controller->driveToPoint(mirror(targetPoint, side), turnScale, settler);
  }

  void driveToPoint2(const Vector& targetPoint, double turnScale = 1,
                     const Settler& settler = OdomController::defaultDriveSettler) {
    controller->driveToPoint2(mirror(targetPoint, side), turnScale, settler);
  }

  void strafeRelativeDirection(const QLength& distance, const QAngle& direction,
                               const AngleCalculator& angleCalculator = makeAngleCalculator(),
                               double turnScale = 1,
                               const Settler& settler = OdomController::defaultDriveSettler) {
    controller->strafeRelativeDirection(distance, mirror(direction, side), angleCalculator,
                                        turnScale, settler);
  }

  void strafeAbsoluteDirection(const QLength& distance, const QAngle& direction,
                               const AngleCalculator& angleCalculator = makeAngleCalculator(),
                               double turnScale = 1,
                               const Settler& settler = OdomController::defaultDriveSettler) {
    controller->strafeAbsoluteDirection(distance, mirror(direction, side), angleCalculator,
                                        turnScale, settler);
  }

  void strafeToPoint(const Vector& targetPoint,
                     const AngleCalculator& angleCalculator = makeAngleCalculator(),
                     double turnScale = 1,
                     const Settler& settler = OdomController::defaultDriveSettler) {
    controller->strafeToPoint(mirror(targetPoint, side), angleCalculator, turnScale, settler);
  }

  static AngleCalculator makeAngleCalculator(const QAngle& angle, const autonSide& side) {
    return OdomController::makeAngleCalculator(mirror(angle, side));
  }

  static AngleCalculator makeAngleCalculator(const Vector& point, const autonSide& side) {
    return OdomController::makeAngleCalculator(mirror(point, side));
  }

  static AngleCalculator makeAngleCalculator(double error, const autonSide& side) {
    return OdomController::makeAngleCalculator(error);
  }

  static AngleCalculator makeAngleCalculator() {
    return OdomController::makeAngleCalculator();
  };

protected:
  std::shared_ptr<OdomXController> controller;
  autonSide side = autonSide::red;
};