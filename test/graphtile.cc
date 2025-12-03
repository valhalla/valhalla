#include "baldr/graphtile.h"
#include "baldr/complexrestriction.h"
#include "config.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"
#include "mjolnir/complexrestrictionbuilder.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

struct testable_graphtile : public valhalla::baldr::GraphTile {
  testable_graphtile(const uint32_t (&offsets)[kBinCount], std::vector<GraphId>& bins) {
    header_ = new GraphTileHeader();
    header_->set_edge_bin_offsets(offsets);
    edge_bins_ = bins.data();
  }
  ~testable_graphtile() {
    delete header_;
  }
};

TEST(Graphtile, FileSuffix) {
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(2, 2, 0)), "2/000/000/002.gph");
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(4, 2, 0)), "2/000/000/004.gph");
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(64799, 1, 0)), "1/064/799.gph");
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(49, 0, 0)), "0/000/049.gph");
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(1000000, 3, 1)), "3/001/000/000.gph");
  EXPECT_THROW(GraphTile::FileSuffix(GraphId(64800, 1, 0)), std::runtime_error);
  EXPECT_THROW(GraphTile::FileSuffix(GraphId(1337, 6, 0)), std::runtime_error);
  EXPECT_THROW(GraphTile::FileSuffix(GraphId(1036800, 2, 0)), std::runtime_error);
  EXPECT_THROW(GraphTile::FileSuffix(GraphId(4050, 0, 0)), std::runtime_error);
  EXPECT_THROW(GraphTile::FileSuffix(GraphId(1036800, 3, 0)), std::runtime_error);

  TileLevel level{7, valhalla::baldr::RoadClass::kSecondary, "half_degree_is_a_multiple_of_3",
                  Tiles<PointLL>{{{-180, -90}, {180, 90}}, .5, 1}};

  EXPECT_EQ(GraphTile::FileSuffix(GraphId(1234, 7, 0), ".qux", false, &level), "7/001/234.qux");
  EXPECT_EQ(GraphTile::FileSuffix(GraphId(123456, 7, 0), ".qux", false, &level), "7/123/456.qux");
}

TEST(Graphtile, IdFromString) {
  EXPECT_EQ(GraphTile::GetTileId("foo/bar/baz/qux/corge/1/000/002.gph"), GraphId(2, 1, 0));
  EXPECT_EQ(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/1/000/002.gph"),
            GraphId(2, 1, 0));
  EXPECT_EQ(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/2/001/000/002.gph"),
            GraphId(1000002, 2, 0));
  EXPECT_EQ(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/3/001/000/002.gph"),
            GraphId(1000002, 3, 0));
  EXPECT_EQ(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/3/001/000/002"),
            GraphId(1000002, 3, 0));
  EXPECT_EQ(GraphTile::GetTileId("2/000/791/317.gph.gz"), GraphId(791317, 2, 0));

  EXPECT_THROW(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/1/000/002/.gph"),
               std::runtime_error);
  EXPECT_THROW(GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/0/004/050.gph"),
               std::runtime_error);
  EXPECT_THROW(GraphTile::GetTileId("foo/bar/0/004/0-1.gph"), std::runtime_error);
  EXPECT_THROW(GraphTile::GetTileId("foo/bar/0/004//001.gph"), std::runtime_error);
  EXPECT_THROW(GraphTile::GetTileId("foo/bar/1/000/004/001.gph"), std::runtime_error);
  EXPECT_THROW(GraphTile::GetTileId("00/002.gph"), std::runtime_error);
}

TEST(Graphtile, Bin) {
  uint32_t offsets[kBinCount] = {1, 2, 3, 0, 1, 2, 3, 1, 1, 2, 3, 2, 1,
                                 2, 3, 3, 1, 2, 3, 4, 1, 2, 3, 5, 1};
  std::vector<uint32_t> offs = {0};
  std::vector<GraphId> bins;
  uint32_t offset = 0;
  for (size_t i = 0, j = 0; i < kBinCount; ++i) {
    offset += offsets[i];
    offs.push_back(offset);
    offsets[i] = offset;
    for (size_t k = 0; k < offsets[i]; ++k)
      bins.emplace_back(j++);
  }
  testable_graphtile t(offsets, bins);
  for (size_t i = 0; i < kBinCount; ++i) {
    std::span<GraphId> itr(bins.data() + offs[i], bins.data() + offs[i + 1]);
    auto idx_itr = t.GetBin(i);
    auto rc_itr = t.GetBin(i % kBinsDim, i / kBinsDim);

    EXPECT_EQ(itr.size(), idx_itr.size()) << "Wrong bin!";
    EXPECT_EQ(itr.size(), rc_itr.size()) << "Wrong bin!";

    for (auto j = itr.begin(), k = idx_itr.begin(), l = rc_itr.begin(); j != itr.end();
         ++j, ++k, ++l) {
      EXPECT_EQ(*j, *k) << "Wrong edge found in bin";
      EXPECT_EQ(*j, *l) << "Wrong edge found in bin";
    }
  }
}

class TestGraphMemory final : public GraphMemory {
public:
  TestGraphMemory(size_t bytes) : memory_(bytes) {
    data = const_cast<char*>(memory_.data());
    size = memory_.size();
  }

private:
  const std::vector<char> memory_;
};

TEST(GraphTileIntegrity, SizeZero) {
  EXPECT_THROW(GraphTile::Create(GraphId(), std::make_unique<const TestGraphMemory>(0)),
               std::runtime_error);
}

TEST(GraphTileIntegrity, SizeLessThanHeader) {
  size_t tileSize = sizeof(GraphTileHeader) - 1;
  EXPECT_THROW(GraphTile::Create(GraphId(), std::make_unique<const TestGraphMemory>(tileSize)),
               std::runtime_error);
}

TEST(GraphTileIntegrity, SizeLessThanPayload) {
  size_t tile_size = 10000;

  GraphTileHeader header;
  // set offset not equal to data size
  header.set_end_offset(tile_size - 1);

  std::vector<char> tile_data(tile_size);
  memcpy(tile_data.data(), &header, sizeof(header));

  EXPECT_THROW(GraphTile::Create(GraphId(), std::make_unique<const TestGraphMemory>(tile_size)),
               std::runtime_error);
}

TEST(GraphTileVersion, VersionChecksum) {
  std::string tile_dir = VALHALLA_BUILD_DIR "test/data/utrecht_tiles";

  auto tile = GraphTile::Create(tile_dir, {3196, 0, 0});

  std::string expected_version = VALHALLA_VERSION;
  EXPECT_TRUE(tile->header()->version().compare(0, expected_version.size(), expected_version) == 0);

  auto checksum = tile->header()->checksum();
  EXPECT_GT(checksum, 0);
}

struct RestrictionBuilder {
  std::vector<char> data;

  void add_restriction(GraphId from_id,
                       GraphId to_id,
                       uint16_t modes,
                       const std::vector<GraphId>& vias = {}) {
    valhalla::mjolnir::ComplexRestrictionBuilder builder;
    builder.set_from_id(from_id);
    builder.set_to_id(to_id);
    builder.set_modes(modes);
    if (!vias.empty()) {
      builder.set_via_list(vias);
    }

    std::ostringstream oss;
    oss << builder;
    std::string serialized = oss.str();

    data.insert(data.end(), serialized.begin(), serialized.end());
  }
};

TEST(ComplexRestrictionView, EmptyView) {
  std::vector<char> empty_data;
  ComplexRestrictionView view(empty_data.data(), 0, GraphId(100, 0, 0), 0xFF, true);

  EXPECT_TRUE(view.empty());
  EXPECT_EQ(view.begin(), view.end());

  int count = 0;
  for (const auto& cr : view) {
    (void)cr;
    count++;
  }
  EXPECT_EQ(count, 0);
}

TEST(ComplexRestrictionView, NoMatchingRestrictions) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(1, 0, 0), GraphId(2, 0, 0), 0x1);
  builder.add_restriction(GraphId(3, 0, 0), GraphId(4, 0, 0), 0x2);
  builder.add_restriction(GraphId(5, 0, 0), GraphId(6, 0, 0), 0x4);

  // Looking for to_graphid = 100 (doesn't exist)
  ComplexRestrictionView view(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0xFF,
                              true);

  EXPECT_TRUE(view.empty());
  EXPECT_EQ(view.begin(), view.end());
}

TEST(ComplexRestrictionView, ForwardRestrictions) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(1, 0, 0), GraphId(100, 0, 0), 0x1); // matches
  builder.add_restriction(GraphId(2, 0, 0), GraphId(200, 0, 0), 0x2); // doesn't match id
  builder.add_restriction(GraphId(3, 0, 0), GraphId(100, 0, 0), 0x4); // matches
  builder.add_restriction(GraphId(4, 0, 0), GraphId(100, 0, 0), 0x8); // doesn't match modes

  // Looking for to_graphid = 100, modes = 0x5 (matches modes 0x1 and 0x4)
  ComplexRestrictionView view(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0x5,
                              true);

  EXPECT_FALSE(view.empty());

  std::vector<const ComplexRestriction*> results;
  for (const auto& cr : view) {
    results.push_back(&cr);
  }

  ASSERT_EQ(results.size(), 2);
  EXPECT_EQ(results[0]->to_graphid(), GraphId(100, 0, 0));
  EXPECT_EQ(results[0]->from_graphid(), GraphId(1, 0, 0));
  EXPECT_EQ(results[1]->to_graphid(), GraphId(100, 0, 0));
  EXPECT_EQ(results[1]->from_graphid(), GraphId(3, 0, 0));
}

TEST(ComplexRestrictionView, ReverseRestrictions) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(100, 0, 0), GraphId(1, 0, 0), 0x1); // matches
  builder.add_restriction(GraphId(200, 0, 0), GraphId(2, 0, 0), 0x2); // doesn't match id
  builder.add_restriction(GraphId(100, 0, 0), GraphId(3, 0, 0), 0x4); // matches

  // Looking for from_graphid = 100 (reverse direction), modes = 0xFF
  ComplexRestrictionView view(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0xFF,
                              false);

  EXPECT_FALSE(view.empty());

  std::vector<const ComplexRestriction*> results;
  for (const auto& cr : view) {
    results.push_back(&cr);
  }

  ASSERT_EQ(results.size(), 2);
  EXPECT_EQ(results[0]->from_graphid(), GraphId(100, 0, 0));
  EXPECT_EQ(results[1]->from_graphid(), GraphId(100, 0, 0));
}

TEST(ComplexRestrictionView, ViewInterfaceMethods) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(1, 0, 0), GraphId(100, 0, 0), 0x1);
  builder.add_restriction(GraphId(2, 0, 0), GraphId(100, 0, 0), 0x2);

  ComplexRestrictionView view(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0xFF,
                              true);

  EXPECT_FALSE(view.empty());
  const auto& first = view.front();
  EXPECT_EQ(first.from_graphid(), GraphId(1, 0, 0));
  EXPECT_TRUE(static_cast<bool>(view));
}

TEST(ComplexRestrictionView, IteratorIncrement) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(1, 0, 0), GraphId(100, 0, 0), 0x1);
  builder.add_restriction(GraphId(2, 0, 0), GraphId(100, 0, 0), 0x2);
  builder.add_restriction(GraphId(3, 0, 0), GraphId(100, 0, 0), 0x4);

  ComplexRestrictionView view(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0xFF,
                              true);

  auto it = view.begin();
  ASSERT_NE(it, view.end());
  EXPECT_EQ((*it).from_graphid(), GraphId(1, 0, 0));

  ++it;
  ASSERT_NE(it, view.end());
  EXPECT_EQ((*it).from_graphid(), GraphId(2, 0, 0));

  ++it;
  ASSERT_NE(it, view.end());
  EXPECT_EQ((*it).from_graphid(), GraphId(3, 0, 0));

  ++it;
  EXPECT_EQ(it, view.end());
}

TEST(ComplexRestrictionView, ModeFiltering) {
  RestrictionBuilder builder;
  builder.add_restriction(GraphId(1, 0, 0), GraphId(100, 0, 0), 0x1);  // mode 1
  builder.add_restriction(GraphId(2, 0, 0), GraphId(100, 0, 0), 0x2);  // mode 2
  builder.add_restriction(GraphId(3, 0, 0), GraphId(100, 0, 0), 0x4);  // mode 4
  builder.add_restriction(GraphId(4, 0, 0), GraphId(100, 0, 0), 0x8);  // mode 8
  builder.add_restriction(GraphId(5, 0, 0), GraphId(100, 0, 0), 0x1F); // modes 1,2,4,8,16

  // Query for mode 0x1 (should match restrictions with modes 0x1 and 0x1F)
  ComplexRestrictionView view1(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0x1,
                               true);
  int count1 = 0;
  for (const auto& cr : view1) {
    (void)cr;
    count1++;
  }
  EXPECT_EQ(count1, 2);

  // Query for mode 0x2 (should match restrictions with modes 0x2 and 0x1F)
  ComplexRestrictionView view2(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0x2,
                               true);
  int count2 = 0;
  for (const auto& cr : view2) {
    (void)cr;
    count2++;
  }
  EXPECT_EQ(count2, 2);

  // Query for mode 0x3 (should match restrictions with modes 0x1, 0x2, and 0x1F)
  ComplexRestrictionView view3(builder.data.data(), builder.data.size(), GraphId(100, 0, 0), 0x3,
                               true);
  int count3 = 0;
  for (const auto& cr : view3) {
    (void)cr;
    count3++;
  }
  EXPECT_EQ(count3, 3);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
