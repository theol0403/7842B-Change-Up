#pragma once
#include "lib7842/api/trajectory/generator/generator.hpp"

namespace lib7842 {

class XVisionGenerator {
public:
  virtual ~XVisionGenerator() = default;

  XVisionGenerator(std::shared_ptr<XDriveModel> imodel, const QAngularSpeed& igearset,
                   const ChassisScales& iscales, const Limits& ilimits, const QTime& idt) :
    model(std::move(imodel)), gearset(igearset), scales(iscales), limits(ilimits), dt(idt) {
    limits.v *= std::sqrt(2);
    limits.a *= std::sqrt(2);
  };

  void strafeBall(const Spline& spline, const Number& vision, const ProfileFlags& flags = {},
                  const PiecewiseTrapezoidal::Markers& markers = {});

  void strafe(const Spline& spline, const ProfileFlags& flags = {},
              const PiecewiseTrapezoidal::Markers& markers = {}) {
    strafeBall(spline, 105_pct, flags, markers);
  }

  void curveBall(const Spline& spline, const Number& vision, const ProfileFlags& flags = {},
                 const PiecewiseTrapezoidal::Markers& markers = {});

  void curve(const Spline& spline, const ProfileFlags& flags = {},
             const PiecewiseTrapezoidal::Markers& markers = {}) {
    curveBall(spline, 105_pct, flags, markers);
  }

protected:
  std::shared_ptr<XDriveModel> model;
  QAngularSpeed gearset;
  ChassisScales scales;
  Limits limits;
  QTime dt;
};

} // namespace lib7842