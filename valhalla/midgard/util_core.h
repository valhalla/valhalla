#pragma once

#include <stdexcept>

namespace valhalla {
namespace midgard {
/**
 * equals with an epsilon for approximation
 * @param a first operand
 * @param b second operand
 * @param epsilon to help with approximate equality
 */
template <class T> bool equal(const T a, const T b, const T epsilon = static_cast<T>(.00001)) {
  if (epsilon < static_cast<T>(0)) {
    throw std::logic_error("Using a negative epsilon is not supported");
  }
  T diff = a - b;
  // if its non-negative it better be less than epsilon, if its negative then it better be bigger
  // than epsilon
  bool negative = diff < static_cast<T>(0);
  return (!negative && diff <= epsilon) || (negative && diff >= -epsilon);
}

template <class T> bool similar(const T a, const T b, const double similarity = .99) {
  if (a == 0 || b == 0) {
    return a == b;
  }
  if ((a < 0) != (b < 0)) {
    return false;
  }
  return (double)std::min(a, b) / (double)std::max(a, b) >= similarity;
}
} // namespace midgard
} // namespace valhalla
