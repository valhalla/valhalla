#include "mjolnir/hilbert.h"
#include "baldr/tilehierarchy.h"
#include "midgard/pointll.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>

using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

// Orders small enough to enumerate: 2x2 up to 256x256.
constexpr uint32_t kOrders[] = {1, 2, 4, 8};

// The 2x2 base case is a U, where a Morton code would give a Z.
TEST(Hilbert, BaseCase) {
  EXPECT_EQ(HilbertIndex(0, 0, 1), 0);
  EXPECT_EQ(HilbertIndex(0, 1, 1), 1);
  EXPECT_EQ(HilbertIndex(1, 1, 1), 2);
  EXPECT_EQ(HilbertIndex(1, 0, 1), 3);
}

TEST(Hilbert, VisitsEveryPositionOnce) {
  for (const uint32_t bits : kOrders) {
    const uint32_t side = 1u << bits;
    std::vector<uint32_t> positions;
    positions.reserve(static_cast<size_t>(side) * side);
    for (uint32_t x = 0; x < side; ++x) {
      for (uint32_t y = 0; y < side; ++y) {
        positions.push_back(HilbertIndex(x, y, bits));
      }
    }
    std::sort(positions.begin(), positions.end());
    // Nothing to dedup and the ends are 0 and side*side-1, so it is exactly [0, side*side).
    EXPECT_EQ(std::unique(positions.begin(), positions.end()), positions.end())
        << "duplicate position at 2^" << bits;
    EXPECT_EQ(positions.front(), 0);
    EXPECT_EQ(positions.back(), side * side - 1);
  }
}

// The property the sort key relies on: consecutive positions are always grid neighbours.
TEST(Hilbert, IsContinuous) {
  for (const uint32_t bits : kOrders) {
    const uint32_t side = 1u << bits;
    std::vector<std::pair<uint32_t, uint32_t>> positions(static_cast<size_t>(side) * side);
    for (uint32_t x = 0; x < side; ++x) {
      for (uint32_t y = 0; y < side; ++y) {
        positions[HilbertIndex(x, y, bits)] = {x, y};
      }
    }
    for (size_t i = 1; i < positions.size(); ++i) {
      const uint32_t step = std::abs(static_cast<int64_t>(positions[i].first) -
                                     static_cast<int64_t>(positions[i - 1].first)) +
                            std::abs(static_cast<int64_t>(positions[i].second) -
                                     static_cast<int64_t>(positions[i - 1].second));
      EXPECT_EQ(step, 1) << "curve of 2^" << bits << " jumps at position " << i;
    }
  }
}

// `Node::sort_key` is a uint32_t, so the default order must fill it without overflowing.
TEST(Hilbert, FillsAUint32) {
  EXPECT_EQ(kHilbertSteps, 65536);
  EXPECT_EQ(HilbertIndex(0, 0), 0);
  EXPECT_EQ(HilbertIndex(kHilbertSteps - 1, 0), std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(HilbertIndex(0, kHilbertSteps - 1), 1431655765);
  EXPECT_EQ(HilbertIndex(kHilbertSteps - 1, kHilbertSteps - 1), 2863311530);
}

TEST(Hilbert, TileCornersAndClamping) {
  const auto& tiling = valhalla::baldr::TileHierarchy::levels()[2].tiles;
  const int32_t tileid = tiling.TileId(PointLL(13.1, 55.6));
  const auto base = tiling.Base(tileid);
  const double size = tiling.TileSize();

  EXPECT_EQ(TileHilbertIndex(base, tiling, tileid), 0);
  // Both borders clamp rather than wrapping or overflowing.
  EXPECT_EQ(TileHilbertIndex(PointLL(base.lng() - size, base.lat() - size), tiling, tileid), 0);
  EXPECT_EQ(TileHilbertIndex(PointLL(base.lng() + size, base.lat()), tiling, tileid),
            HilbertIndex(kHilbertSteps - 1, 0));

  // One quantization step in is one position along each axis.
  const double step = size / kHilbertSteps;
  EXPECT_EQ(TileHilbertIndex(PointLL(base.lng() + step, base.lat() + step), tiling, tileid),
            HilbertIndex(1, 1));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
