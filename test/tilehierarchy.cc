#include "baldr/tilehierarchy.h"
#include "baldr/graphid.h"
#include "midgard/pointll.h"

#include "test.h"

#include <boost/property_tree/ptree.hpp>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

TEST(TileHierarchy, Parse) {
  EXPECT_EQ(TileHierarchy::levels().size(), 3) << "Incorrect number of hierarchy levels";
  EXPECT_EQ((++TileHierarchy::levels().begin())->second.name, "arterial")
      << "Middle hierarchy should be named arterial";
  EXPECT_EQ(TileHierarchy::levels().begin()->second.level, 0) << "Top hierarchy should have level 0";
  EXPECT_EQ(TileHierarchy::levels().rbegin()->second.tiles.TileSize(), .25f)
      << "Bottom hierarchy should have tile size of .25f";
  EXPECT_EQ(TileHierarchy::levels().find(5), TileHierarchy::levels().end())
      << "There should only be levels 0, 1, 2";
  EXPECT_NE(TileHierarchy::levels().find(2), TileHierarchy::levels().end())
      << "There should be a level 2";
  GraphId id = TileHierarchy::GetGraphId(PointLL(0, 0), 34);

  EXPECT_FALSE(id.Is_Valid()) << "GraphId should be invalid as the level doesn't exist";

  // there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
  id = TileHierarchy::GetGraphId(PointLL(-76.5, 40.5), 2);
  EXPECT_EQ(id.level(), 2);
  EXPECT_EQ(id.tileid(), (522 * 1440) + 414);
  EXPECT_EQ(id.id(), 0);

  EXPECT_EQ(TileHierarchy::levels().begin()->second.importance, RoadClass::kPrimary)
      << "Importance should be set to primary";
  EXPECT_EQ((++TileHierarchy::levels().begin())->second.importance, RoadClass::kTertiary)
      << "Importance should be set to tertiary";
  EXPECT_EQ(TileHierarchy::levels().rbegin()->second.importance, RoadClass::kServiceOther)
      << "Importance should be set to service/other";
}

TEST(TileHierarchy, Tiles) {

  // there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
  AABB2<PointLL> bbox{{-76.49, 40.51}, {-76.48, 40.52}};
  auto ids = TileHierarchy::GetGraphIds(bbox, 2);
  EXPECT_EQ(ids.size(), 1) << "Should have only found one result.";

  auto id = ids[0];
  EXPECT_EQ(id.level(), 2);
  EXPECT_EQ(id.tileid(), (522 * 1440) + 414);
  EXPECT_EQ(id.id(), 0);

  bbox = AABB2<PointLL>{{-76.51, 40.49}, {-76.49, 40.51}};
  ids = TileHierarchy::GetGraphIds(bbox, 2);
  EXPECT_EQ(ids.size(), 4) << "Should have found 4 results.";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
