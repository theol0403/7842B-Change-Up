#pragma once
#include "lib7842/api/async/taskWrapper.hpp"
#include "main.h"
#include <memory>
using namespace lib7842;

class VisionTask : public TaskWrapper {
public:
  virtual ~VisionTask() = default;

  VisionTask(const std::shared_ptr<Vision::Vision>& vision,
             const std::shared_ptr<GUI::VisionPage>& idrawer);

  double getOffset() const;

protected:
  const std::shared_ptr<Vision::Vision> vision;
  const std::shared_ptr<GUI::VisionPage> drawer;

  double offset {0};

protected:
  /**
   * Override this method to implement setup procedures.
   */
  virtual void initialize();

  /**
   * Override this method to implement the statemachine task
   */
  void loop() override;
};