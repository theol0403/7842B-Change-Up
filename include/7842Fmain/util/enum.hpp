#pragma once

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
  constexpr T operator|=(T& a, T b) {                                                              \
    a = static_cast<T>(static_cast<std::underlying_type_t<T>>(a) |                                 \
                       static_cast<std::underlying_type_t<T>>(b));                                 \
    return a;                                                                                      \
  }                                                                                                \
  constexpr T operator&=(T& a, T b) {                                                              \
    a = static_cast<T>(static_cast<std::underlying_type_t<T>>(a) &                                 \
                       static_cast<std::underlying_type_t<T>>(b));                                 \
    return a;                                                                                      \
  }                                                                                                \
  constexpr bool operator!(T a) {                                                                  \
    return !static_cast<bool>(a);                                                                  \
  }
