#pragma once
#include "lib7842/api/trajectory/generator/generator.hpp"

namespace lib7842 {

struct ChassisFlags {
  Number start_v = 0_pct;
  Number end_v = 0_pct;
  Number top_v = 100_pct;
  Number ball_seek {105_pct};
  Number goal_seek {105_pct};
};

class XVisionGenerator {
public:
  virtual ~XVisionGenerator() = default;

  XVisionGenerator(std::shared_ptr<XDriveModel> imodel, const QAngularSpeed& igearset,
                   const ChassisScales& iscales, const Limits& ilimits, const QTime& idt) :
    model(std::move(imodel)), gearset(igearset), scales(iscales), limits(ilimits), dt(idt) {
    limits.v *= std::sqrt(2);
    limits.a *= std::sqrt(2);
  };

  void strafe(const Spline& spline, const ChassisFlags& flags = {},
              const PiecewiseTrapezoidal::Markers& markers = {});

  void curve(const Spline& spline, const ChassisFlags& flags = {},
             const PiecewiseTrapezoidal::Markers& markers = {});

protected:
  std::shared_ptr<XDriveModel> model;
  QAngularSpeed gearset;
  ChassisScales scales;
  Limits limits;
  QTime dt;
};

} // namespace lib7842