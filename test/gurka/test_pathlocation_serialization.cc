//
// These tests sanity check the PathLocation serialization+deserialization functions.
// Each test varies a different constructor argument, and then verifies that
// serializing the object to PBF, and then deserializing it back yields an object
// with the same properties.
//

#include "baldr/pathlocation.h"
#include "gurka.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
struct PathlocationSerialization : public ::testing::Test {

  static valhalla::gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A---1B----C
    |    |
    D----E----F
    |
    G----H---2I)";

    const valhalla::gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},  {"BC", {{"highway", "primary"}}},
        {"DEF", {{"highway", "primary"}}}, {"GHI", {{"highway", "primary"}}},
        {"ADG", {{"highway", "primary"}}}, {"BE", {{"highway", "primary"}}},
    };

    const auto layout = valhalla::gurka::detail::map_to_coordinates(ascii_map, 10, PointLL{.1, .1});
    map = valhalla::gurka::buildtiles(layout, ways, {}, {}, "test/data/gurak_pathloc_serial",
                                      {{"mjolnir.hierarchy", "false"}, {"mjolnir.concurrency", "1"}});
  }
};

valhalla::gurka::map PathlocationSerialization::map = {};

void TryLocationSerializeAndDeserialize(const PathLocation& originalLoc) {
  GraphReader reader(PathlocationSerialization::map.config.get_child("mjolnir"));
  valhalla::Location odinLoc;

  // Serialize the passed in location
  PathLocation::toPBF(originalLoc, &odinLoc, reader);

  // Deserialize it back so we can sanity check the serialization
  PathLocation loc = PathLocation::fromPBF(odinLoc);

  // Make sure the object we got back is equivalent to the original
  EXPECT_EQ(originalLoc, loc) << "Deserialized location is not equal to the original";
}

TEST_F(PathlocationSerialization, TestDefault) {
  PointLL point;
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

TEST_F(PathlocationSerialization, TestPoint) {
  PointLL point(42, 42);
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

TEST_F(PathlocationSerialization, TestStopType) {
  PointLL point{};

  PathLocation locBreak(point, Location::StopType::BREAK);
  TryLocationSerializeAndDeserialize(locBreak);

  PathLocation locThrough(point, Location::StopType::THROUGH);
  TryLocationSerializeAndDeserialize(locThrough);
}

TEST_F(PathlocationSerialization, TestMinimumReachability) {
  PointLL point{};
  PathLocation loc(point, Location::StopType::BREAK, 42, 42);
  TryLocationSerializeAndDeserialize(loc);
}

TEST_F(PathlocationSerialization, TestRadius) {
  PointLL point;
  PathLocation loc(point, Location::StopType::BREAK, 42, 42, 3);
  TryLocationSerializeAndDeserialize(loc);
}

TEST_F(PathlocationSerialization, TestEdges) {
  GraphReader reader(PathlocationSerialization::map.config.get_child("mjolnir"));
  auto id = *reader.GetTileSet().begin();
  PathLocation loc(PointLL{2, 13}, Location::StopType::BREAK);
  loc.edges.emplace_back(PathLocation::PathEdge{id, 0, PointLL{50, 50}, 10});

  valhalla::Location pbfloc;
  PathLocation::toPBF(loc, &pbfloc, reader);
  EXPECT_EQ(pbfloc.correlation().edges_size(), 1);
  EXPECT_EQ(pbfloc.correlation().filtered_edges_size(), 0);

  loc.edges.clear();
  loc.filtered_edges.emplace_back(PathLocation::PathEdge{++id, 1, PointLL{50, 50}, 10});
  pbfloc.Clear();
  PathLocation::toPBF(loc, &pbfloc, reader);
  EXPECT_EQ(pbfloc.correlation().edges_size(), 0);
  EXPECT_EQ(pbfloc.correlation().filtered_edges_size(), 1);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
