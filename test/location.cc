#include "test.h"
#include <cstdint>

#include "baldr/location.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include <unordered_map>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

void test_hashing() {

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
  if (m.find(a) == m.cend())
    throw std::logic_error("Should have found a");
  if (!m.insert({b, 2}).second)
    throw std::logic_error("Should not have found b");
  if (!m.insert({c, 3}).second)
    throw std::logic_error("Should not have found c");
  if (!m.insert({d, 4}).second)
    throw std::logic_error("Should not have found d");
}

} // namespace

int main(void) {
  test::suite suite("location");

  suite.test(TEST_CASE(test_hashing));

  return suite.tear_down();
}
