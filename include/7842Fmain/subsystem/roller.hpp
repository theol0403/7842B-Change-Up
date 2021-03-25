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

// The first few bits enable various flags that control the behavior of the rollers. These can then be combined to encode various states.
// The bits after enumerate other states/actions.
constexpr uint8_t fbits = 5; // the number of flag bits
enum class rollerStates {
  off = 0, // all off, no poop

  intake = 1 << 0, // move intakes
  bottom = 1 << 1, // move bottom roller
  top = 1 << 2, // move top roller

  out = 1 << 3, // reverse anything that is not enabled
  poop = 1 << 4, // enable auto poop

  // helper combinations
  loading = intake | bottom, // load balls into robot. Disable rollers one by one
  shoot = bottom | top, // all on but don't move intakes
  on = intake | bottom | top, // all on

  compress = out | intake, // reverse top and botton rollers while intaking
  fill = out | shoot, // shoot top and bottom while outtaking
  purge = out | top, // shoot top while reversing rest

  // onPoop = on | poop,
  // outPoop = out | poop,
  // shootPoop = shoot | poop,
  // loadingPoop = loading | poop,

  // other actions
  deploy = 1 << fbits,
  timedPoop = 2 << fbits,
  timedShootPoop = 3 << fbits,
  spacedShoot = 4 << fbits, // all on but space the ball
  topPoop = 5 << fbits, // bring down then poop
  shootRev = 6 << fbits, // bring rollers back

  flags = 0b11111, // bitmask for flags
  actions = 0b11100000, // bitmask for actions
};

ENUM_FLAG_OPERATORS(rollerStates)

class Roller : public StateMachine<rollerStates, rollerStates::poop> {
public:
  Roller(const std::shared_ptr<AbstractMotor>& iintakes,
         const std::shared_ptr<AbstractMotor>& ibottom, const std::shared_ptr<AbstractMotor>& itop,
         const std::shared_ptr<OpticalSensor>& itoplight,
         const std::shared_ptr<OpticalSensor>& ibottomLight);

public:
  enum class colors { none = 0, red, blue };

  colors getTopLight() const;
  colors getBottomLight() const;

  bool shouldPoop();
  bool shouldShootPoop();
  bool shouldSpacedShoot();

  void initialize() override;
  void loop() override;

  std::shared_ptr<AbstractMotor> intakes {nullptr};
  std::shared_ptr<AbstractMotor> bottom {nullptr};
  std::shared_ptr<AbstractMotor> top {nullptr};
  std::shared_ptr<OpticalSensor> topLight {nullptr};
  std::shared_ptr<OpticalSensor> bottomLight {nullptr};

  Timer macroTime;
};