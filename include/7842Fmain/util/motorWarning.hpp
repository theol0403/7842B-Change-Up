#pragma once
#include "main.h"

class MotorWarning : public TaskWrapper {
 public:
  using TaskWrapper::TaskWrapper;
  void addMotor(const std::shared_ptr<Motor>& imotor, const std::string& iname);

 protected:
  void loop() override;

  std::vector<std::pair<std::shared_ptr<Motor>, std::string>> motors {};
};