#include "baldr/custom_attributes_tile.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <span>

using namespace valhalla::baldr;

namespace {
class UnmanagedGraphMemory : public GraphMemory {
public:
  UnmanagedGraphMemory(char* const buf, const size_t len) {
    data = buf;
    size = len;
  }
};

#pragma pack(push, 1)
// 3 edges × 2 uint8 attributes, row-major
struct TestTile {
  CustomAttributesTileHeader header;
  uint8_t values[6];
};
#pragma pack(pop)

CustomAttributesTile make_tile(TestTile& testdata, uint32_t expected_edge_count) {
  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  return CustomAttributesTile(std::move(memory), expected_edge_count);
}

TestTile make_test_tile() {
  TestTile testdata{};
  testdata.header.directed_edge_count = 3;
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TILE_VERSION;
  testdata.header.num_attributes = 2;
  testdata.header.row_size = 2;
  const uint8_t values[6] = {10, 20, 50, 60, 255, 0};
  std::copy(std::begin(values), std::end(values), testdata.values);
  return testdata;
}

const std::vector<CustomAttributeDef> kDefs =
    ParseCustomAttributeDefs(R"([{"name": "scenic", "type": "uint8", "scale": 0.01},
                                 {"name": "walkability", "type": "uint8"}])");
} // namespace

TEST(CustomAttributesTile, ParseDefs) {
  ASSERT_EQ(kDefs.size(), 2u);
  EXPECT_EQ(kDefs[0].name, "scenic");
  EXPECT_EQ(kDefs[0].type, CustomAttributeType::kUint8);
  EXPECT_FLOAT_EQ(kDefs[0].scale, 0.01f);
  EXPECT_EQ(kDefs[0].offset, 0u);
  EXPECT_EQ(kDefs[1].name, "walkability");
  EXPECT_FLOAT_EQ(kDefs[1].scale, 1.0f);
  EXPECT_EQ(kDefs[1].offset, 8u);
  EXPECT_EQ(custom_attributes_row_size(kDefs), 2u);
}

TEST(CustomAttributesTile, ParseDefsRejectsInvalid) {
  EXPECT_THROW(ParseCustomAttributeDefs(R"(["scenic"])"), std::runtime_error);
  EXPECT_THROW(ParseCustomAttributeDefs(R"({"name": "a", "type": "uint8"})"), std::runtime_error);
  EXPECT_THROW(ParseCustomAttributeDefs(R"([{"name": "a"}])"), std::runtime_error);
  EXPECT_THROW(ParseCustomAttributeDefs(R"([{"name": "a", "type": "float"}])"), std::runtime_error);
  EXPECT_THROW(ParseCustomAttributeDefs(R"([{"name": "a", "type": "uint8", "scale": 0}])"),
               std::runtime_error);
  EXPECT_THROW(ParseCustomAttributeDefs(
                   R"([{"name": "a", "type": "uint8"}, {"name": "a", "type": "uint8"}])"),
               std::runtime_error);
}

TEST(CustomAttributesTile, TileConstruction) {
  auto testdata = make_test_tile();
  auto tile = make_tile(testdata, 3);

  EXPECT_EQ(tile.edge_count(), 3u);
  EXPECT_EQ(tile.num_attributes(), 2u);
  EXPECT_EQ(tile.row_size(), 2u);
  EXPECT_FLOAT_EQ(tile.value(0, kDefs[0]), 0.1f);
  EXPECT_FLOAT_EQ(tile.value(0, kDefs[1]), 20.0f);
  EXPECT_FLOAT_EQ(tile.value(1, kDefs[0]), 0.5f);
  EXPECT_FLOAT_EQ(tile.value(1, kDefs[1]), 60.0f);
  EXPECT_FLOAT_EQ(tile.value(2, kDefs[0]), 2.55f);
  EXPECT_FLOAT_EQ(tile.value(2, kDefs[1]), 0.0f);
}

TEST(CustomAttributesTile, WeightedSum) {
  auto testdata = make_test_tile();
  auto tile = make_tile(testdata, 3);

  // weights are pre-multiplied by the scale
  const std::array<WeightedCustomAttribute, 2> weights = {{
      {kDefs[0].offset, kDefs[0].type, 0.8f * kDefs[0].scale},
      {kDefs[1].offset, kDefs[1].type, 0.01f * kDefs[1].scale},
  }};
  // edge0: 0.1 * 0.8 + 20 * 0.01 = 0.28
  EXPECT_FLOAT_EQ(tile.weighted_sum(0, weights), 0.28f);
  // edge1: 0.5 * 0.8 + 60 * 0.01 = 1.0
  EXPECT_FLOAT_EQ(tile.weighted_sum(1, weights), 1.0f);
  // only the second column
  EXPECT_FLOAT_EQ(tile.weighted_sum(1, std::span(weights).subspan(1)), 0.6f);
  // out-of-bounds edge
  EXPECT_FLOAT_EQ(tile.weighted_sum(99, weights), 0.0f);
}

TEST(CustomAttributesTile, OutOfBoundsReturnsZero) {
  auto testdata = make_test_tile();
  auto tile = make_tile(testdata, 3);

  EXPECT_FLOAT_EQ(tile.value(3, kDefs[0]), 0.0f);
  EXPECT_FLOAT_EQ(tile.value(999, kDefs[0]), 0.0f);
  // a column past the tile's row_size
  const CustomAttributeDef past_row{"past", CustomAttributeType::kUint8, 1.0f, 16};
  EXPECT_FLOAT_EQ(tile.value(0, past_row), 0.0f);
  const std::array<WeightedCustomAttribute, 1> weights = {{{16, CustomAttributeType::kUint8, 1.0f}}};
  EXPECT_FLOAT_EQ(tile.weighted_sum(0, weights), 0.0f);
}

TEST(CustomAttributesTile, VersionMismatchReturnsZero) {
  auto testdata = make_test_tile();
  testdata.header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TILE_VERSION - 1;
  auto tile = make_tile(testdata, 3);

  EXPECT_FLOAT_EQ(tile.value(0, kDefs[1]), 0.0f);
}

TEST(CustomAttributesTile, NullTileConstruction) {
  CustomAttributesTile tile(nullptr, 0);

  EXPECT_FALSE(tile);
  EXPECT_FLOAT_EQ(tile.value(0, kDefs[0]), 0.0f);
  EXPECT_FLOAT_EQ(tile.value(99, kDefs[0]), 0.0f);
}

TEST(CustomAttributesTile, EdgeCountMismatchThrows) {
  auto testdata = make_test_tile();
  EXPECT_THROW(make_tile(testdata, 5), std::runtime_error);
}

TEST(CustomAttributesTile, TooSmallForRowSizeThrows) {
  auto testdata = make_test_tile();
  testdata.header.row_size = 3;
  EXPECT_THROW(make_tile(testdata, 3), std::runtime_error);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
