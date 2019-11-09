#pragma once
#include "main.h"

template <typename States> class StateMachine : public TaskWrapper {

 public:
  StateMachine() = default;
  virtual ~StateMachine() = default;

  virtual void setState(const States& istate) {
    state = istate;
  }

  virtual const States& getState() const {
    return state;
  }

 protected:
  virtual void calibrate() = 0;
  virtual void loop() = 0;

  States state {States::off};
};