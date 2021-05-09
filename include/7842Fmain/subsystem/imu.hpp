#pragma once
#include "main.h"

class IMUTurn {
public:
  IMUTurn(const std::shared_ptr<pros::Imu>& iimu, const std::shared_ptr<XDriveModel>& imodel,
          const std::shared_ptr<IterativePosPIDController>& ipid) :
    imu(iimu), model(imodel), pid(ipid) {
    imu->set_data_rate(5);
    calibrate();
  }

  const QAngle tolerance = 2_deg;

  void turn(const QAngle& a) {
    QAngle error = 0_deg;
    pid->controllerSet(0);
    Timer settleTime;
    Rate rate;
    do {
      error = util::rollAngle180(a - (-1 * imu->get_rotation() * degree - offset));
      double out = pid->step(-error.convert(degree));
      model->tank(-out, out);
      if (abs(error) < tolerance) settleTime.placeHardMark();
      rate.delayUntil(10_ms);
    } while ((abs(error) >= tolerance && settleTime.getDtFromStart() < 3_s) ||
             settleTime.getDtFromHardMark() < 50_ms);
    model->tank(0, 0);
  }

  void reset(const QAngle& start = 0_deg) {
    offset = -1 * imu->get_rotation() * degree - start;
  }

  void calibrate() {
    imu->reset();
    pros::delay(2000);
    while (imu->is_calibrating()) {
      pros::delay(10);
    }
    reset();
  }

  std::shared_ptr<pros::Imu> imu;
  std::shared_ptr<XDriveModel> model;
  std::shared_ptr<IterativePosPIDController> pid;

  QAngle offset {0_deg};
};
