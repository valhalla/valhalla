#include "baldr/custom_attributes_tile.h"

#include <gtest/gtest.h>

#include <span>

namespace {
class UnmanagedGraphMemory : public valhalla::baldr::GraphMemory {
public:
  UnmanagedGraphMemory(char* const buf, const size_t len) {
    data = buf;
    size = len;
  }
};
} // namespace

TEST(CustomAttributesTile, TileConstruction) {
  using namespace valhalla::baldr;

#pragma pack(push, 1)
  struct TestTile {
    CustomAttributesTileHeader header;
    // 3 edges × 2 attributes = 6 floats, row-major
    float values[6];
  };
#pragma pack(pop)

  TestTile testdata{};
  testdata.header.directed_edge_count = 3;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TAR_VERSION;
  testdata.header.num_attributes = 2;
  // edge0: attr0=0.1, attr1=0.2
  testdata.values[0] = 0.1f;
  testdata.values[1] = 0.2f;
  // edge1: attr0=0.5, attr1=0.6
  testdata.values[2] = 0.5f;
  testdata.values[3] = 0.6f;
  // edge2: attr0=1.0, attr1=0.9
  testdata.values[4] = 1.0f;
  testdata.values[5] = 0.9f;

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  CustomAttributesTile tile(std::move(memory), 3);

  EXPECT_EQ(tile.edge_count(), 3u);
  EXPECT_EQ(tile.num_attributes(), 2u);
  EXPECT_FLOAT_EQ(tile.value(0, 0), 0.1f);
  EXPECT_FLOAT_EQ(tile.value(0, 1), 0.2f);
  EXPECT_FLOAT_EQ(tile.value(1, 0), 0.5f);
  EXPECT_FLOAT_EQ(tile.value(1, 1), 0.6f);
  EXPECT_FLOAT_EQ(tile.value(2, 0), 1.0f);
  EXPECT_FLOAT_EQ(tile.value(2, 1), 0.9f);
}

TEST(CustomAttributesTile, WeightedSum) {
  using namespace valhalla::baldr;

#pragma pack(push, 1)
  struct TestTile {
    CustomAttributesTileHeader header;
    float values[4]; // 2 edges × 2 attributes
  };
#pragma pack(pop)

  TestTile testdata{};
  testdata.header.directed_edge_count = 2;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TAR_VERSION;
  testdata.header.num_attributes = 2;
  testdata.values[0] = 0.5f; // edge0 attr0
  testdata.values[1] = 1.0f; // edge0 attr1
  testdata.values[2] = 0.2f; // edge1 attr0
  testdata.values[3] = 0.8f; // edge1 attr1

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  CustomAttributesTile tile(std::move(memory), 2);

  // edge0: 0.5 * 0.8 + 1.0 * 0.3 = 0.4 + 0.3 = 0.7
  const std::array<float, 2> weights = {0.8f, 0.3f};
  EXPECT_FLOAT_EQ(tile.weighted_sum(0, weights), 0.7f);
  // edge1: 0.2 * 0.8 + 0.8 * 0.3 = 0.16 + 0.24 = 0.4
  EXPECT_FLOAT_EQ(tile.weighted_sum(1, weights), 0.4f);
  // out-of-bounds edge
  EXPECT_FLOAT_EQ(tile.weighted_sum(99, weights), 0.0f);
}

TEST(CustomAttributesTile, SingleAttributeBackwardsCompatible) {
  // num_attributes=1 should behave like the old single-value format
  using namespace valhalla::baldr;

#pragma pack(push, 1)
  struct TestTile {
    CustomAttributesTileHeader header;
    float value0;
  };
#pragma pack(pop)

  TestTile testdata{};
  testdata.header.directed_edge_count = 1;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TAR_VERSION;
  testdata.header.num_attributes = 1;
  testdata.value0 = 0.7f;

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  CustomAttributesTile tile(std::move(memory), 1);

  EXPECT_FLOAT_EQ(tile.value(0, 0), 0.7f);
  EXPECT_FLOAT_EQ(tile.value(1, 0), 0.0f); // out-of-bounds edge
  EXPECT_FLOAT_EQ(tile.value(0, 1), 0.0f); // out-of-bounds attribute
}

TEST(CustomAttributesTile, OutOfBoundsReturnsZero) {
  using namespace valhalla::baldr;

#pragma pack(push, 1)
  struct TestTile {
    CustomAttributesTileHeader header;
    float value0;
  };
#pragma pack(pop)

  TestTile testdata{};
  testdata.header.directed_edge_count = 1;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TAR_VERSION;
  testdata.header.num_attributes = 1;
  testdata.value0 = 0.7f;

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  CustomAttributesTile tile(std::move(memory), 1);

  EXPECT_FLOAT_EQ(tile.value(0), 0.7f); // default attr_index=0
  EXPECT_FLOAT_EQ(tile.value(1), 0.0f);
  EXPECT_FLOAT_EQ(tile.value(999), 0.0f);
}

TEST(CustomAttributesTile, NullTileConstruction) {
  using namespace valhalla::baldr;
  CustomAttributesTile tile(nullptr, 0);

  EXPECT_FLOAT_EQ(tile.value(0), 0.0f);
  EXPECT_FLOAT_EQ(tile.value(99), 0.0f);
}

TEST(CustomAttributesTile, EdgeCountMismatchThrows) {
  using namespace valhalla::baldr;

#pragma pack(push, 1)
  struct TestTile {
    CustomAttributesTileHeader header;
    float value0;
    float value1;
  };
#pragma pack(pop)

  TestTile testdata{};
  testdata.header.directed_edge_count = 2;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TAR_VERSION;
  testdata.header.num_attributes = 1;
  testdata.value0 = 0.3f;
  testdata.value1 = 0.6f;

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  // expected_edge_count doesn't match header.directed_edge_count → should throw
  EXPECT_THROW(CustomAttributesTile(std::move(memory), 5), std::runtime_error);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
