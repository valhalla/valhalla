//
// These tests sanity check the PathLocation serialization+deserialization functions.
// Each test varies a different constructor argument, and then verifies that
// serializing the object to PBF, and then deserializing it back yields an object
// with the same properties.
//

#include <boost/property_tree/ptree.hpp>

#include "test.h"

#include "baldr/pathlocation.h"

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

void TryLocationSerializeAndDeserialize(const PathLocation& loc) {
  GraphReader reader(fake_config);
  valhalla::Location odinLoc;

  // Serialize the passed in location
  PathLocation::toPBF(loc, &odinLoc, reader);

  // Deserialize it back so we can sanity check the serialization
  PathLocation loc2 = PathLocation::fromPBF(odinLoc);

  // Make sure the object we got back is equivalent to the original
  if (!(loc == loc2)) {
    throw runtime_error("Deserialized location is not equal to the original");
  }
}

void TestDefault() {
  PointLL point;
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

void TestPoint() {
  PointLL point(42, 42);
  PathLocation loc(point);
  TryLocationSerializeAndDeserialize(loc);
}

void TestStopType() {
  PointLL point;

  PathLocation locBreak(point, Location::StopType::BREAK);
  TryLocationSerializeAndDeserialize(locBreak);

  PathLocation locThrough(point, Location::StopType::THROUGH);
  TryLocationSerializeAndDeserialize(locThrough);
}

void TestMinimumReachability() {
  PointLL point;
  PathLocation loc(point, Location::StopType::BREAK, 42);
  TryLocationSerializeAndDeserialize(loc);
}

void TestRadius() {
  PointLL point;
  PathLocation loc(point, Location::StopType::BREAK, 0, 42);
  TryLocationSerializeAndDeserialize(loc);
}

} // namespace

int main() {
  test::suite suite("pathlocation_serialization");

  // Test using the default constructor.
  suite.test(TEST_CASE(TestDefault));

  // Test using a non-default point.
  suite.test(TEST_CASE(TestPoint));

  // Test using both StopTypes.
  suite.test(TEST_CASE(TestStopType));

  // Test using a non-default minimum reachability.
  suite.test(TEST_CASE(TestMinimumReachability));

  // Test using a non-default radius.
  suite.test(TEST_CASE(TestRadius));
}
