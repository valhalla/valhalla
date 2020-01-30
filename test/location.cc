#include "baldr/location.h"
#include "midgard/util.h"

#include <unordered_map>

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

TEST(Location, Hashing) {
  Location a({123.456789, 9.87654321}, Location::StopType::BREAK);
  a.name_ = "name";
  a.street_ = "street";
  a.city_ = "city";
  a.state_ = "state";
  a.zip_ = "zip";
  a.country_ = "country";
  a.date_time_ = "date";
  a.heading_ = 1;
  a.way_id_ = 2;
  Location b = a;
  b.name_ = "nameb";
  Location c = a;
  c.way_id_.reset();
  Location d = a;
  d.latlng_.first = 123.4567;

  std::unordered_map<Location, int> m{{a, 1}};
  EXPECT_NE(m.find(a), m.cend()) << "Should have found a";
  EXPECT_TRUE(m.insert({b, 2}).second) << "Should not have found b";
  EXPECT_TRUE(m.insert({c, 3}).second) << "Should not have found c";
  EXPECT_TRUE(m.insert({d, 4}).second) << "Should not have found d";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
