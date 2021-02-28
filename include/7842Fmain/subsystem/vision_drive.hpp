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
  };

  void strafe(const Spline& spline, const ProfileFlags& flags = {},
              const std::vector<std::pair<Number, Number>>& markers = {});

  void curve(const Spline& spline, const ProfileFlags& flags = {},
             const std::vector<std::pair<Number, Number>>& markers = {});

protected:
  std::shared_ptr<XDriveModel> model;
  QAngularSpeed gearset;
  ChassisScales scales;
  Limits limits;
  QTime dt;
};

} // namespace lib7842