#pragma once
#include "7842Fmain/util/statemachine.hpp"
#include "main.h"

#define ENUM_FLAG_OPERATORS(T)                                                                     \
  constexpr T operator~(T a) {                                                                     \
    return static_cast<T>(~static_cast<std::underlying_type_t<T>>(a));                             \
  }                                                                                                \
  constexpr T operator|(T a, T b) {                                                                \
    return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) |                              \
                          static_cast<std::underlying_type_t<T>>(b));                              \
  }                                                                                                \
  constexpr T operator&(T a, T b) {                                                                \
    return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) &                              \
                          static_cast<std::underlying_type_t<T>>(b));                              \
  }                                                                                                \
  constexpr T operator^(T a, T b) {                                                                \
    return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) ^                              \
                          static_cast<std::underlying_type_t<T>>(b));                              \
  }                                                                                                \
  constexpr T operator|=(T& a, T b) {                                                              \
    a = static_cast<T>(static_cast<std::underlying_type_t<T>>(a) |                                 \
                       static_cast<std::underlying_type_t<T>>(b));                                 \
    return a;                                                                                      \
  }                                                                                                \
  constexpr T operator&=(T& a, T b) {                                                              \
    a = static_cast<T>(static_cast<std::underlying_type_t<T>>(a) &                                 \
                       static_cast<std::underlying_type_t<T>>(b));                                 \
    return a;                                                                                      \
  }

// The first 5 bits enable various flags that control the behavior of the rollers. These can then be combined to encode various states.
// The bits after enumerate other states/actions.
constexpr uint8_t fbits = 5; // the number of feature bits
enum class rollerStates {
  off = 0, // all off, no poop

  intake = 1 << 0, // move intakes
  bottom = 1 << 1, // move bottom roller
  top = 1 << 2, // move top roller

  poop = 1 << 3, // enable auto poop
  out = 1 << 4, // outtake all motors except for the ones enabled

  // helper combinations
  on = intake | bottom | top, // all on
  shoot = bottom | top, // all on but don't move intakes
  loading = intake | bottom, // load balls into robot. Disable rollers one by one
  compress = out | intake, // reverse top and botton while intaking
  purge = out | top, // shoot top while outtaking rest
  shootOut = out | shoot, // shoot top and bottom while outtaking

  onPoop = on | poop,
  outPoop = out | poop,
  shootPoop = shoot | poop,
  loadingPoop = loading | poop,

  // other actions
  deploy = 1 << fbits,
  timedPoop = 2 << fbits,
  timedShootPoop = 3 << fbits,
  spacedShoot = 4 << fbits, // all on but space the ball
  topPoop = 5 << fbits, // bring down then poop
};

ENUM_FLAG_OPERATORS(rollerStates)

class Roller : public StateMachine<rollerStates, rollerStates::off> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottomRoller,
         const std::shared_ptr<AbstractMotor>& itopRoller,
         const std::shared_ptr<OpticalSensor>& itoplight,
         const std::shared_ptr<OpticalSensor>& ibottomLight,
         const std::shared_ptr<GUI::Graph>& igraph);

public:
  enum class colors { none = 0, red, blue };

  colors getTopLight() const;
  colors getBottomLight() const;

  bool shouldPoop();
  bool shouldShootPoop();
  bool shouldSpacedShoot();

  int getIntake();

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottomRoller {nullptr};
  std::shared_ptr<AbstractMotor> topRoller {nullptr};
  std::shared_ptr<OpticalSensor> topLight {nullptr};
  std::shared_ptr<OpticalSensor> bottomLight {nullptr};
  std::shared_ptr<GUI::Graph> graph {nullptr};

  Timer macroTime;
  rollerStates macroReturnState = rollerStates::off;
  int macroIntakeVel = 12000;
};