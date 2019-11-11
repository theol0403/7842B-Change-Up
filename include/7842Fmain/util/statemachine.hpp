#pragma once
#include "main.h"

template <typename States, States assumedState = States::off>
class StateMachine : public TaskWrapper {

 public:
  StateMachine() = default;
  virtual ~StateMachine() = default;

  virtual void setState(const States& istate) {
    state = istate;
  }

  virtual void setNewState(const States& istate) {
    if (istate != lastState) {
      state = istate;
      lastState = istate;
    }
  }

  virtual const States& getState() const {
    return state;
  }

 protected:
  virtual void calibrate() = 0;
  virtual void loop() = 0;

  States state {States::off};
  States lastState {assumedState};
};