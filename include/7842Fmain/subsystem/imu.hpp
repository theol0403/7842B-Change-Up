#pragma once
#include "main.h"

class IMUTurn {
public:
  IMUTurn(const std::shared_ptr<pros::Imu>& iimu1, const std::shared_ptr<pros::Imu>& iimu2,
          const std::shared_ptr<XDriveModel>& imodel,
          const std::shared_ptr<IterativePosPIDController>& ipid) :
    imu1(iimu1), imu2(iimu2), model(imodel), pid(ipid) {
    imu1->set_data_rate(5);
    imu2->set_data_rate(5);
    calibrate();
  }

  const QAngle tolerance = 2_deg;

  void turn(const QAngle& a) {
    QAngle error = 0_deg;
    pid->controllerSet(0);
    Timer settleTime;
    Rate rate;
    do {
      error = util::rollAngle180(a - get());
      double out = pid->step(-error.convert(degree));
      model->tank(-out, out);
      if (abs(error) < tolerance) settleTime.placeHardMark();
      rate.delayUntil(10_ms);
    } while ((abs(error) >= tolerance && settleTime.getDtFromStart() < 3_s) ||
             settleTime.getDtFromHardMark() < 50_ms);
    model->tank(0, 0);
  }

  void reset(const QAngle& start = 0_deg) {
    offset1 = -1 * imu1->get_rotation() * degree - start;
    offset2 = -1 * imu2->get_rotation() * degree - start;
  }

  QAngle get() {
    auto one(-1 * imu1->get_rotation() * degree - offset1);
    auto two(-1 * imu2->get_rotation() * degree - offset2);
    return (one + two) / 2.0;
  }

  void calibrate() {
    imu1->reset();
    imu2->reset();
    pros::delay(1000);
    while (imu1->is_calibrating() || imu2->is_calibrating()) {
      pros::delay(10);
    }
    reset();
  }

  std::shared_ptr<pros::Imu> imu1;
  std::shared_ptr<pros::Imu> imu2;
  std::shared_ptr<XDriveModel> model;
  std::shared_ptr<IterativePosPIDController> pid;

  QAngle offset1 {0_deg};
  QAngle offset2 {0_deg};
};
