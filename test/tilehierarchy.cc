#include "baldr/tilehierarchy.h"
#include "baldr/graphid.h"
#include "midgard/pointll.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

TEST(TileHierarchy, Parse) {
  EXPECT_EQ(TileHierarchy::levels().size(), 3) << "Incorrect number of hierarchy levels";
  EXPECT_EQ((++TileHierarchy::levels().begin())->name, "arterial")
      << "Middle hierarchy should be named arterial";
  EXPECT_EQ(TileHierarchy::levels().begin()->level, 0) << "Top hierarchy should have level 0";
  EXPECT_EQ(TileHierarchy::levels().rbegin()->tiles.TileSize(), .25f)
      << "Bottom hierarchy should have tile size of .25f";
  EXPECT_EQ(TileHierarchy::levels()[0].level, 0);
  EXPECT_EQ(TileHierarchy::levels()[1].level, 1);
  EXPECT_EQ(TileHierarchy::levels()[2].level, 2);
  GraphId id = TileHierarchy::GetGraphId(PointLL(0, 0), 34);

  EXPECT_FALSE(id.Is_Valid()) << "GraphId should be invalid as the level doesn't exist";

  // there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
  id = TileHierarchy::GetGraphId(PointLL(-76.5, 40.5), 2);
  EXPECT_EQ(id.level(), 2);
  EXPECT_EQ(id.tileid(), (522 * 1440) + 414);
  EXPECT_EQ(id.id(), 0);

  EXPECT_EQ(TileHierarchy::levels().begin()->importance, RoadClass::kPrimary)
      << "Importance should be set to primary";
  EXPECT_EQ((++TileHierarchy::levels().begin())->importance, RoadClass::kTertiary)
      << "Importance should be set to tertiary";
  EXPECT_EQ(TileHierarchy::levels().rbegin()->importance, RoadClass::kServiceOther)
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

TEST(TileHierarchy, parent) {
  GraphId id(1440 * 16 + 16, 3, 0);
  auto level2 = TileHierarchy::parent(id);
  EXPECT_EQ(level2, GraphId(id.tileid(), 2, 0));
  auto level1 = TileHierarchy::parent(level2);
  EXPECT_EQ(level1, GraphId(360 * 4 + 4, 1, 0));
  auto level0 = TileHierarchy::parent(level1);
  EXPECT_EQ(level0, GraphId(90 * 1 + 1, 0, 0));
  auto invalid = TileHierarchy::parent(level0);
  EXPECT_EQ(invalid, GraphId(kInvalidGraphId));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
