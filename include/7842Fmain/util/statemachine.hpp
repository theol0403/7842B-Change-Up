#pragma once
#include "main.h"
#include <atomic>

/**
 * State machine helper class.
 *
 * @tparam States       An enum class representing the states of a subsystem. Required to have an
 *                      off state.
 * @tparam assumedState Optional - The assumed last state when using setNewState. Initially calling
 *                      setNewState with this state will not trigger a state transition.
 */
template <typename States, States assumedState = States::off>
class StateMachine : public TaskWrapper {

public:
  StateMachine() = default;
  virtual ~StateMachine() = default;

  /**
   * Sets the state.
   *
   * @param istate The state
   */
  virtual void setState(const States& istate) {
    stateLock.take(TIMEOUT_MAX);
    state.store(istate, std::memory_order_release);
    stateLock.give();
  }

  /**
   * Sets the state and waits until the statemachine reports done.
   *
   * @param istate The istate
   */
  virtual void setStateBlocking(const States& istate) {
    stateLock.take(TIMEOUT_MAX);
    _isDone.store(false, std::memory_order_release);
    state.store(istate, std::memory_order_release);
    stateLock.give();
    while (!isDone()) {
      pros::delay(10);
    };
  }

  /**
   * Sets the state only if the state is different from the last time this function was called.
   *
   * @param istate The state
   */
  virtual void setNewState(const States& istate) {
    stateLock.take(TIMEOUT_MAX);
    if (state.load(std::memory_order_acquire) != lastState.load(std::memory_order_acquire)) {
      state.store(istate, std::memory_order_release);
      lastState.store(istate, std::memory_order_release);
    }
    stateLock.give();
  }

  /**
   * Gets the state.
   *
   * @return The state.
   */
  virtual const States getState() {
    stateLock.take(TIMEOUT_MAX);
    auto istate = state.load(std::memory_order_acquire);
    stateLock.give();
    return istate;
  }

  /**
   * Gets the state.
   *
   * @return The state.
   */
  virtual bool isDone() {
    stateLock.take(TIMEOUT_MAX);
    auto iisDone = _isDone.load(std::memory_order_acquire);
    stateLock.give();
    return iisDone;
  }

protected:
  /**
   * Override this method to implement setup procedures.
   */
  virtual void initialize() = 0;

  /**
   * Override this method to implement the statemachine task
   */
  void loop() override = 0;

  virtual void setDone() {
    _isDone.store(true, std::memory_order_release);
  }

  std::atomic<States> state {States::off};
  std::atomic<States> lastState {assumedState};

  pros::Mutex stateLock {};

  std::atomic<bool> _isDone = false;
};