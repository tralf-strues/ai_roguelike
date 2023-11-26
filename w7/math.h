#pragma once

#include <cmath>
#include <tuple>

struct IVec2
{
  int x, y;

  constexpr IVec2() = default;

  constexpr explicit IVec2(int v) : x(v), y(v) {}
  constexpr explicit IVec2(int x, int y) : x(x), y(y) {}
};

inline bool operator==(const IVec2 &lhs, const IVec2 &rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool operator!=(const IVec2 &lhs, const IVec2 &rhs) { return !(lhs == rhs); }

inline IVec2 operator-(const IVec2 &lhs, const IVec2 &rhs)
{
  return IVec2{lhs.x - rhs.x, lhs.y - rhs.y};
}

inline IVec2 operator+(const IVec2 &lhs, const IVec2 &rhs)
{
  return IVec2{lhs.x + rhs.x, lhs.y + rhs.y};
}

inline IVec2 operator*(const IVec2 &lhs, int rhs)
{
  return IVec2{lhs.x * rhs, lhs.y * rhs};
}

inline IVec2 operator/(const IVec2 &up, int down)
{
  return IVec2{up.x / down, up.y / down};
}

inline bool operator<(const IVec2 &lhs, const IVec2 &rhs) {
  return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
}

template<typename T>
inline T sqr(T a){ return a*a; }

template<typename T, typename U>
inline float dist_sq(const T &lhs, const U &rhs) { return float(sqr(lhs.x - rhs.x) + sqr(lhs.y - rhs.y)); }

template<typename T, typename U>
inline float dist(const T &lhs, const U &rhs) { return sqrtf(dist_sq(lhs, rhs)); }

