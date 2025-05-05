
#include "baldr/graphid.h"
#include "midgard/point2.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

TEST(GraphId, TestValues) {
  GraphId target(123, 2, 8);
  EXPECT_EQ(target.tileid(), 123);
  EXPECT_EQ(target.level(), 2);
  EXPECT_EQ(target.id(), 8);

  GraphId target2(5689, 1, 1234567);
  EXPECT_EQ(target2.tileid(), 5689);
  EXPECT_EQ(target2.level(), 1);
  EXPECT_EQ(target2.id(), 1234567);

  // Test the tile_value
  EXPECT_EQ(target2.Tile_Base().value, target2.tile_value());

  target.set_id(5678);
  EXPECT_EQ(target.id(), 5678);
  EXPECT_EQ(target.tileid(), 123);
  EXPECT_EQ(target.level(), 2);
}

TEST(GraphId, TestInvalidValues) {
  EXPECT_THROW(GraphId badlevel(111, kMaxGraphHierarchy + 1, 222), logic_error);
  EXPECT_THROW(GraphId badtile(kMaxGraphTileId + 1, 0, 222), logic_error);
  EXPECT_THROW(GraphId badid(111, 1, kMaxGraphId + 1), logic_error);
}

TEST(GraphId, TestCtorDefault) {
  GraphId target;
  EXPECT_FALSE(target.Is_Valid());
}

void TryCtorUintUintUint(const unsigned int tileid,
                         const unsigned int level,
                         const unsigned int id,
                         const GraphId& expected) {
  GraphId result(tileid, level, id);
  EXPECT_EQ(expected, result);
}

TEST(GraphId, TestCtorUintUintUint) {
  TryCtorUintUintUint(10, 2, 1, GraphId(10, 2, 1));
  TryCtorUintUintUint(5, 1, 50, GraphId(5, 1, 50));
}

void TryCtorCopy(const GraphId& gid, const GraphId& expected) {
  GraphId result(gid);
  EXPECT_EQ(expected, result);
}

TEST(GraphId, TestCtorCopy) {
  TryCtorCopy(GraphId(10, 2, 1), GraphId(10, 2, 1));
  TryCtorCopy(GraphId(5, 1, 50), GraphId(5, 1, 50));
}

void TryGet_tileid(const GraphId& gid, const unsigned int expected) {
  EXPECT_EQ(gid.tileid(), expected);
}

TEST(GraphIdGet, Tileid) {
  TryGet_tileid(GraphId(10, 2, 1), 10);
  TryGet_tileid(GraphId(5, 1, 50), 5);
}

void TryGet_level(const GraphId& gid, const unsigned int expected) {
  EXPECT_EQ(expected, gid.level());
}

TEST(GraphIdGet, Level) {
  TryGet_level(GraphId(10, 2, 1), 2);
  TryGet_level(GraphId(5, 1, 50), 1);
}

void TryGet_id(const GraphId& gid, const unsigned int expected) {
  EXPECT_EQ(expected, gid.id());
}

TEST(GraphIdGet, Id) {
  TryGet_id(GraphId(10, 2, 1), 1);
  TryGet_id(GraphId(5, 1, 50), 50);
}

TEST(GraphId, TestIsValid) {
  GraphId id(1, 2, 3);
  EXPECT_TRUE(id.Is_Valid());

  id = GraphId();
  EXPECT_FALSE(id.Is_Valid()) << "Default constructor should never return valid graphid";
}

void TryOpPostIncrement(GraphId& gid, const unsigned int expected) {
  auto old = gid++;
  EXPECT_EQ(expected, gid.id());
  EXPECT_EQ(expected - 1, old.id());
}

TEST(GraphId, TestOpPostIncrement) {
  GraphId graphid1{10, 5, 0};
  TryOpPostIncrement(graphid1, 1);
  GraphId graphid2{10, 5, 1};
  TryOpPostIncrement(graphid2, 2);
  GraphId graphid3{5, 1, 50};
  TryOpPostIncrement(graphid3, 51);
}

TEST(GraphId, TestOpLessThan) {
  EXPECT_LT(GraphId(0, 0, 0), GraphId(0, 0, 1));
  EXPECT_LT(GraphId(10, 5, 1), GraphId(10, 6, 1));
  EXPECT_LT(GraphId(5, 1, 50), GraphId(6, 1, 50));
  EXPECT_LT(GraphId(111, 6, 333), GraphId(112, 7, 334));
}

void TryOpEqualTo(const GraphId& gid, const GraphId& expected) {
  EXPECT_EQ(expected, gid);
  EXPECT_EQ(gid, expected);
}

TEST(GraphId, TestOpEqualTo) {
  TryOpEqualTo(GraphId(0, 0, 0), GraphId(0, 0, 0));
  TryOpEqualTo(GraphId(10, 5, 1), GraphId(10, 5, 1));
  TryOpEqualTo(GraphId(5, 1, 50), GraphId(5, 1, 50));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
