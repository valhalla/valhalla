//
// These tests sanity check the PathLocation serialization+deserialization functions.
// Each test varies a different constructor argument, and then verifies that
// serializing the object to PBF, and then deserializing it back yields an object
// with the same properties.
//

#include <boost/property_tree/ptree.hpp>

#include "baldr/pathlocation.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

boost::property_tree::ptree generate_dummy_config() {
  boost::property_tree::ptree p;
  p.put("tile_dir", "/file/does/not/exist");
  return p;
}

const boost::property_tree::ptree fake_config = generate_dummy_config();

void TryLocationSerializeAndDeserialize(const PathLocation& originalLoc) {
  GraphReader reader(fake_config);
  valhalla::Location odinLoc;

  // Serialize the passed in location
  PathLocation::toPBF(originalLoc, &odinLoc, reader);

  // Deserialize it back so we can sanity check the serialization
  PathLocation loc = PathLocation::fromPBF(odinLoc);

  // Make sure the object we got back is equivalent to the original
  EXPECT_EQ(originalLoc, loc) << "Deserialized location is not equal to the original";
}

TEST(PathlocationSerialization, TestDefault) {
  PointLL point;
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

TEST(PathlocationSerialization, TestPoint) {
  PointLL point(42, 42);
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

TEST(PathlocationSerialization, TestStopType) {
  PointLL point{};

  PathLocation locBreak(point, Location::StopType::BREAK);
  TryLocationSerializeAndDeserialize(locBreak);

  PathLocation locThrough(point, Location::StopType::THROUGH);
  TryLocationSerializeAndDeserialize(locThrough);
}

TEST(PathlocationSerialization, DISABLED_TestMinimumReachability) {
  PointLL point{};
  PathLocation loc(point, Location::StopType::BREAK, 42);
  TryLocationSerializeAndDeserialize(loc);
}

TEST(PathlocationSerialization, DISABLED_TestRadius) {
  PointLL point;
  PathLocation loc(point, Location::StopType::BREAK, 0, 42);
  TryLocationSerializeAndDeserialize(loc);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
