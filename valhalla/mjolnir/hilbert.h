#pragma once

#include "midgard/pointll.h"
#include "midgard/tiles.h"

#include <cstdint>
#include <utility>

namespace valhalla {
namespace mjolnir {

// Bits each axis is quantized to. 16 is the most a uint32_t index can hold, and puts a step far
// below the spacing of neighbouring nodes on every level.
constexpr uint32_t kHilbertBits = 16;
constexpr uint32_t kHilbertSteps = 1u << kHilbertBits;

// Position of (x, y) along a Hilbert curve filling a 2^bits square; `bits` is parameterized
// only for tests. See https://en.wikipedia.org/wiki/Hilbert_curve
inline uint32_t HilbertIndex(uint32_t x, uint32_t y, const uint32_t bits = kHilbertBits) {
  const uint32_t side = 1u << bits;
  uint32_t d = 0;
  for (uint32_t s = side >> 1; s > 0; s >>= 1) {
    const uint32_t rx = (x & s) ? 1 : 0;
    const uint32_t ry = (y & s) ? 1 : 0;
    // Positions covered by the preceding quadrants.
    d += s * s * ((3 * rx) ^ ry);
    // Reflect the bottom quadrants so each sub-curve ends where the next one begins.
    if (ry == 0) {
      if (rx == 1) {
        x = side - 1 - x;
        y = side - 1 - y;
      }
      std::swap(x, y);
    }
  }
  return d;
}

// Position of a point along a Hilbert curve filling the tile it falls in.
inline uint32_t TileHilbertIndex(const midgard::PointLL& ll,
                                 const midgard::Tiles<midgard::PointLL>& tiling,
                                 const int32_t tileid) {
  const auto base = tiling.Base(tileid);
  const double step = tiling.TileSize() / static_cast<double>(kHilbertSteps);
  // Clamped: a negative cast is undefined, and the far edge must not round past the last
  // position on the curve.
  const auto quantize = [step](const double offset) -> uint32_t {
    const double steps = offset / step;
    if (steps <= 0.0) {
      return 0;
    }
    return steps >= static_cast<double>(kHilbertSteps - 1) ? kHilbertSteps - 1
                                                           : static_cast<uint32_t>(steps);
  };
  return HilbertIndex(quantize(ll.lng() - base.lng()), quantize(ll.lat() - base.lat()));
}

} // namespace mjolnir
} // namespace valhalla
