#include "main.h"

class SideController {
public:
  enum class sides { red, blue };

  SideController(const std::shared_ptr<OdomXController>& icontroller, const sides& iside) :
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
                    const Settler& settler = OdomController::defaultDriveAngleSettler) {
    controller->driveToPoint(mirror(targetPoint, side), turnScale, settler);
  }

  void driveToPoint2(const Vector& targetPoint, double turnScale = 1,
                     const Settler& settler = OdomController::defaultDriveAngleSettler) {
    controller->driveToPoint2(mirror(targetPoint, side), turnScale, settler);
  }

  void strafeRelativeDirection(const QLength& distance, const QAngle& direction,
                               const AngleCalculator& angleCalculator = makeAngleCalculator(),
                               double turnScale = 1,
                               const Settler& settler = OdomController::defaultDriveAngleSettler) {
    controller->strafeRelativeDirection(distance, mirror(direction, side), angleCalculator,
                                        turnScale, settler);
  }

  void strafeAbsoluteDirection(const QLength& distance, const QAngle& direction,
                               const AngleCalculator& angleCalculator = makeAngleCalculator(),
                               double turnScale = 1,
                               const Settler& settler = OdomController::defaultDriveAngleSettler) {
    controller->strafeAbsoluteDirection(distance, mirror(direction, side), angleCalculator,
                                        turnScale, settler);
  }

  void strafeToPoint(const Vector& targetPoint,
                     const AngleCalculator& angleCalculator = makeAngleCalculator(),
                     double turnScale = 1,
                     const Settler& settler = OdomController::defaultDriveAngleSettler) {
    controller->strafeToPoint(mirror(targetPoint, side), angleCalculator, turnScale, settler);
  }

  static AngleCalculator makeAngleCalculator(const QAngle& angle, const sides& side) {
    return OdomController::makeAngleCalculator(mirror(angle, side));
  }

  static AngleCalculator makeAngleCalculator(const Vector& point, const sides& side) {
    return OdomController::makeAngleCalculator(mirror(point, side));
  }

  static AngleCalculator makeAngleCalculator(double error, const sides& side) {
    return OdomController::makeAngleCalculator(error);
  }

  static AngleCalculator makeAngleCalculator() {
    return OdomController::makeAngleCalculator();
  };

  std::shared_ptr<OdomXController>& getController() {
    return controller;
  }

  sides& getSide() {
    return side;
  };

  /**
   * Mirror the given coordinates across the field
   */
  static QAngle mirror(const QAngle& angle, const sides& side) {
    return side == sides::red ? angle : angle * -1;
  }

  static QLength mirror(const QLength& x, const sides& side) {
    return side == sides::red ? x : 12_ft - x;
  }

  static Vector mirror(const Vector& point, const sides& side) {
    return {mirror(point.x, side), point.y};
  }

  static State mirror(const State& state, const sides& side) {
    return {mirror(state.x, side), state.y, mirror(state.theta, side)};
  }

protected:
  std::shared_ptr<OdomXController> controller;
  sides side = sides::red;
};