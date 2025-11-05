#include "baldr/graphtile.h"
#include "config.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"
#include "mjolnir/complexrestrictionbuilder.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

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
    valhalla::midgard::iterable_t<GraphId> itr(bins.data() + offs[i], bins.data() + offs[i + 1]);
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

// Helper function to serialize a ComplexRestriction to a buffer
size_t SerializeRestriction(const ComplexRestrictionBuilder& builder,
                            std::vector<char>& buffer,
                            size_t offset,
                            const std::vector<GraphId>& vias) {
  // Write the ComplexRestriction structure
  const ComplexRestriction* restriction = reinterpret_cast<const ComplexRestriction*>(&builder);
  size_t restriction_size = restriction->SizeOf();

  if (offset + restriction_size > buffer.size()) {
    buffer.resize(offset + restriction_size);
  }

  memcpy(buffer.data() + offset, restriction, sizeof(ComplexRestriction));

  // Write the via list after the structure
  size_t via_offset = offset + sizeof(ComplexRestriction);
  for (const auto& via : vias) {
    memcpy(buffer.data() + via_offset, &via, sizeof(GraphId));
    via_offset += sizeof(GraphId);
  }

  return restriction_size;
}

TEST(RestrictionView, EmptyView) {
  std::vector<char> buffer;
  GraphId target_id(100, 1, 0);
  uint64_t modes = 0xFF; // all modes

  RestrictionView view(buffer.data(), buffer.size(), target_id, modes, true);

  EXPECT_TRUE(view.empty()) << "Empty buffer should create empty view";
  EXPECT_EQ(view.begin(), view.end()) << "Begin should equal end for empty view";
}

TEST(RestrictionView, ForwardRestrictionMatching) {
  std::vector<char> buffer;
  GraphId from_id(100, 1, 0);
  GraphId to_id(200, 1, 0);
  uint64_t mode = 0x01; // mode bit 0

  // Create a restriction from->to with mode 0x01
  ComplexRestrictionBuilder builder;
  builder.set_from_id(from_id);
  builder.set_to_id(to_id);
  builder.set_modes(mode);
  builder.set_type(RestrictionType::kNoLeftTurn);

  SerializeRestriction(builder, buffer, 0, {});

  // Forward search: looking for restrictions TO a specific edge
  RestrictionView view(buffer.data(), buffer.size(), to_id, mode, true);

  EXPECT_FALSE(view.empty()) << "Should find matching restriction";

  int count = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    ComplexRestriction* r = *it;
    EXPECT_EQ(r->to_graphid(), to_id) << "Should match target to_id";
    EXPECT_EQ(r->from_graphid(), from_id) << "Should have correct from_id";
    EXPECT_TRUE(r->modes() & mode) << "Should have matching mode";
    count++;
  }
  EXPECT_EQ(count, 1) << "Should iterate over exactly one restriction";
}

TEST(RestrictionView, BackwardRestrictionMatching) {
  std::vector<char> buffer;
  GraphId from_id(100, 1, 0);
  GraphId to_id(200, 1, 0);
  uint64_t mode = 0x02; // mode bit 1

  // Create a restriction from->to with mode 0x02
  ComplexRestrictionBuilder builder;
  builder.set_from_id(from_id);
  builder.set_to_id(to_id);
  builder.set_modes(mode);
  builder.set_type(RestrictionType::kNoRightTurn);

  SerializeRestriction(builder, buffer, 0, {});

  // Backward search: looking for restrictions FROM a specific edge
  RestrictionView view(buffer.data(), buffer.size(), from_id, mode, false);

  EXPECT_FALSE(view.empty()) << "Should find matching restriction";

  int count = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    ComplexRestriction* r = *it;
    EXPECT_EQ(r->from_graphid(), from_id) << "Should match target from_id";
    EXPECT_EQ(r->to_graphid(), to_id) << "Should have correct to_id";
    EXPECT_TRUE(r->modes() & mode) << "Should have matching mode";
    count++;
  }
  EXPECT_EQ(count, 1) << "Should iterate over exactly one restriction";
}

TEST(RestrictionView, ModeFiltering) {
  std::vector<char> buffer;
  GraphId from_id(100, 1, 0);
  GraphId to_id(200, 1, 0);

  // Create a restriction with mode 0x01
  ComplexRestrictionBuilder builder;
  builder.set_from_id(from_id);
  builder.set_to_id(to_id);
  builder.set_modes(0x01);
  builder.set_type(RestrictionType::kOnlyLeftTurn);

  SerializeRestriction(builder, buffer, 0, {});

  // Search with mode 0x02 (different mode, should not match)
  RestrictionView view(buffer.data(), buffer.size(), to_id, 0x02, true);

  EXPECT_TRUE(view.empty()) << "Should not find restriction with different mode";

  // Search with mode 0x01 (matching mode, should match)
  RestrictionView view2(buffer.data(), buffer.size(), to_id, 0x01, true);

  EXPECT_FALSE(view2.empty()) << "Should find restriction with matching mode";
}

TEST(RestrictionView, MultipleRestrictions) {
  std::vector<char> buffer;
  GraphId target_id(200, 1, 0);
  uint64_t mode = 0xFF; // all modes

  // Create three restrictions, two pointing TO target_id, one pointing elsewhere
  ComplexRestrictionBuilder builder1;
  builder1.set_from_id(GraphId(100, 1, 0));
  builder1.set_to_id(target_id);
  builder1.set_modes(mode);
  builder1.set_type(RestrictionType::kNoLeftTurn);

  ComplexRestrictionBuilder builder2;
  builder2.set_from_id(GraphId(150, 1, 0));
  builder2.set_to_id(GraphId(300, 1, 0)); // different target
  builder2.set_modes(mode);
  builder2.set_type(RestrictionType::kNoRightTurn);

  ComplexRestrictionBuilder builder3;
  builder3.set_from_id(GraphId(175, 1, 0));
  builder3.set_to_id(target_id);
  builder3.set_modes(mode);
  builder3.set_type(RestrictionType::kOnlyRightTurn);

  size_t offset = 0;
  offset += SerializeRestriction(builder1, buffer, offset, {});
  offset += SerializeRestriction(builder2, buffer, offset, {});
  offset += SerializeRestriction(builder3, buffer, offset, {});

  // Forward search for target_id
  RestrictionView view(buffer.data(), buffer.size(), target_id, mode, true);

  EXPECT_FALSE(view.empty()) << "Should find matching restrictions";

  int count = 0;
  std::vector<GraphId> found_from_ids;
  for (auto it = view.begin(); it != view.end(); ++it) {
    ComplexRestriction* r = *it;
    EXPECT_EQ(r->to_graphid(), target_id) << "All restrictions should point to target_id";
    found_from_ids.push_back(r->from_graphid());
    count++;
  }

  EXPECT_EQ(count, 2) << "Should find exactly two restrictions";
  EXPECT_EQ(found_from_ids.size(), 2) << "Should have collected two from_ids";

  // Verify the correct restrictions were found
  bool found_100 = false, found_175 = false;
  for (const auto& id : found_from_ids) {
    if (id == GraphId(100, 1, 0))
      found_100 = true;
    if (id == GraphId(175, 1, 0))
      found_175 = true;
  }
  EXPECT_TRUE(found_100) << "Should find restriction from edge 100";
  EXPECT_TRUE(found_175) << "Should find restriction from edge 175";
}

TEST(RestrictionView, WithViaEdges) {
  std::vector<char> buffer;
  GraphId from_id(100, 1, 0);
  GraphId to_id(200, 1, 0);
  uint64_t mode = 0xFF;

  // Create a restriction with via edges
  ComplexRestrictionBuilder builder;
  builder.set_from_id(from_id);
  builder.set_to_id(to_id);
  builder.set_modes(mode);
  builder.set_type(RestrictionType::kNoUTurn);

  std::vector<GraphId> vias = {GraphId(150, 1, 0), GraphId(175, 1, 0), GraphId(180, 1, 0)};
  builder.set_via_list(vias);

  SerializeRestriction(builder, buffer, 0, vias);

  RestrictionView view(buffer.data(), buffer.size(), to_id, mode, true);

  EXPECT_FALSE(view.empty()) << "Should find restriction with vias";

  for (auto it = view.begin(); it != view.end(); ++it) {
    ComplexRestriction* r = *it;
    EXPECT_EQ(r->via_count(), 3) << "Should have 3 via edges";

    // Verify vias
    std::vector<GraphId> walked_vias;
    r->WalkVias([&walked_vias](const GraphId* via) {
      walked_vias.push_back(*via);
      return WalkingVia::KeepWalking;
    });

    EXPECT_EQ(walked_vias.size(), 3) << "Should walk 3 vias";
    for (size_t i = 0; i < vias.size(); i++) {
      EXPECT_EQ(walked_vias[i], vias[i]) << "Via " << i << " should match";
    }
  }
}

TEST(RestrictionView, IteratorOperations) {
  std::vector<char> buffer;
  GraphId target_id(200, 1, 0);
  uint64_t mode = 0xFF;

  // Create two restrictions
  ComplexRestrictionBuilder builder1;
  builder1.set_from_id(GraphId(100, 1, 0));
  builder1.set_to_id(target_id);
  builder1.set_modes(mode);
  builder1.set_type(RestrictionType::kNoLeftTurn);

  ComplexRestrictionBuilder builder2;
  builder2.set_from_id(GraphId(150, 1, 0));
  builder2.set_to_id(target_id);
  builder2.set_modes(mode);
  builder2.set_type(RestrictionType::kNoRightTurn);

  size_t offset = 0;
  offset += SerializeRestriction(builder1, buffer, offset, {});
  offset += SerializeRestriction(builder2, buffer, offset, {});

  RestrictionView view(buffer.data(), buffer.size(), target_id, mode, true);

  auto it1 = view.begin();
  auto it2 = view.begin();

  // Test equality
  EXPECT_EQ(it1, it2) << "Two begin iterators should be equal";
  EXPECT_NE(it1, view.end()) << "Begin should not equal end";

  // Test dereference
  ComplexRestriction* r1 = *it1;
  EXPECT_NE(r1, nullptr) << "Dereferenced iterator should not be null";

  // Test arrow operator
  EXPECT_EQ(it1->to_graphid(), target_id) << "Arrow operator should work";

  // Test pre-increment
  ++it1;
  EXPECT_NE(it1, it2) << "Incremented iterator should differ from original";

  // Test post-increment
  auto it3 = it2++;
  EXPECT_NE(it3, it2) << "Post-increment should return old value";
  EXPECT_EQ(it1, it2) << "Both iterators should now point to second element";

  // Increment to end
  ++it1;
  EXPECT_EQ(it1, view.end()) << "Should reach end";
}

TEST(RestrictionView, NoMatchingId) {
  std::vector<char> buffer;
  GraphId from_id(100, 1, 0);
  GraphId to_id(200, 1, 0);
  GraphId search_id(999, 1, 0); // different id
  uint64_t mode = 0xFF;

  ComplexRestrictionBuilder builder;
  builder.set_from_id(from_id);
  builder.set_to_id(to_id);
  builder.set_modes(mode);
  builder.set_type(RestrictionType::kNoStraightOn);

  SerializeRestriction(builder, buffer, 0, {});

  // Search for an ID that doesn't match
  RestrictionView view(buffer.data(), buffer.size(), search_id, mode, true);

  EXPECT_TRUE(view.empty()) << "Should not find restriction with non-matching ID";

  int count = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    count++;
  }
  EXPECT_EQ(count, 0) << "Should not iterate over any restrictions";
}

TEST(RestrictionView, MixedModesAndDirections) {
  std::vector<char> buffer;
  GraphId edge_id(100, 1, 0);
  uint64_t mode_auto = 0x01;
  uint64_t mode_truck = 0x02;
  uint64_t mode_all = 0xFF;

  // Create restrictions with different modes
  ComplexRestrictionBuilder builder1;
  builder1.set_from_id(edge_id);
  builder1.set_to_id(GraphId(200, 1, 0));
  builder1.set_modes(mode_auto);
  builder1.set_type(RestrictionType::kNoLeftTurn);

  ComplexRestrictionBuilder builder2;
  builder2.set_from_id(edge_id);
  builder2.set_to_id(GraphId(300, 1, 0));
  builder2.set_modes(mode_truck);
  builder2.set_type(RestrictionType::kNoRightTurn);

  ComplexRestrictionBuilder builder3;
  builder3.set_from_id(GraphId(400, 1, 0));
  builder3.set_to_id(edge_id);
  builder3.set_modes(mode_all);
  builder3.set_type(RestrictionType::kOnlyLeftTurn);

  size_t offset = 0;
  offset += SerializeRestriction(builder1, buffer, offset, {});
  offset += SerializeRestriction(builder2, buffer, offset, {});
  offset += SerializeRestriction(builder3, buffer, offset, {});

  // Backward search with auto mode (should find 1)
  RestrictionView view1(buffer.data(), buffer.size(), edge_id, mode_auto, false);
  int count1 = 0;
  for (auto it = view1.begin(); it != view1.end(); ++it) {
    count1++;
  }
  EXPECT_EQ(count1, 1) << "Should find 1 backward restriction with auto mode";

  // Backward search with truck mode (should find 1)
  RestrictionView view2(buffer.data(), buffer.size(), edge_id, mode_truck, false);
  int count2 = 0;
  for (auto it = view2.begin(); it != view2.end(); ++it) {
    count2++;
  }
  EXPECT_EQ(count2, 1) << "Should find 1 backward restriction with truck mode";

  // Backward search with all modes (should find 2)
  RestrictionView view3(buffer.data(), buffer.size(), edge_id, mode_all, false);
  int count3 = 0;
  for (auto it = view3.begin(); it != view3.end(); ++it) {
    count3++;
  }
  EXPECT_EQ(count3, 2) << "Should find 2 backward restrictions with all modes";

  // Forward search with all modes (should find 1)
  RestrictionView view4(buffer.data(), buffer.size(), edge_id, mode_all, true);
  int count4 = 0;
  for (auto it = view4.begin(); it != view4.end(); ++it) {
    count4++;
  }
  EXPECT_EQ(count4, 1) << "Should find 1 forward restriction with all modes";
}

TEST(GraphTileVersion, VersionChecksum) {
  std::string tile_dir = VALHALLA_BUILD_DIR "test/data/utrecht_tiles";

  auto tile = GraphTile::Create(tile_dir, {3196, 0, 0});

  std::string expected_version = VALHALLA_VERSION;
  EXPECT_TRUE(tile->header()->version().compare(0, expected_version.size(), expected_version) == 0);

  auto checksum = tile->header()->checksum();
  EXPECT_GT(checksum, 0);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
