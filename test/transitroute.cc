#include "test.h"

#include "baldr/transitroute.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 40 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kTransitRouteExpectedSize = 40;

namespace {

void test_sizeof() {
  if (sizeof(TransitRoute) != kTransitRouteExpectedSize)
    throw std::runtime_error("TransitRoute size should be " +
                             std::to_string(kTransitRouteExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(TransitRoute)));
}

void TestWriteRead() {
  // Test building a transit route and reading back values
  TransitType route_type = TransitType::kMetro;
  TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, 888, 999);
  if (route.route_type() != route_type) {
    throw runtime_error("TranstRoute route type failed");
  }
  if (route.one_stop_offset() != 111) {
    throw runtime_error("TransitRoute one_stop_offset failed");
  }
  if (route.op_by_onestop_id_offset() != 222) {
    throw runtime_error("TransitRoute op_by_onestop_id_offset failed");
  }
  if (route.op_by_name_offset() != 333) {
    throw runtime_error("TransitRoute op_by_name_offset failed");
  }
  if (route.op_by_website_offset() != 444) {
    throw runtime_error("TransitRoute op_by_website_offset failed");
  }
  if (route.route_color() != 555) {
    throw runtime_error("TransitRoute route_color failed");
  }
  if (route.route_text_color() != 666) {
    throw runtime_error("TransitRoute route_text_color failed");
  }
  if (route.short_name_offset() != 777) {
    throw runtime_error("TransitRoute short_name_offset failed");
  }
  if (route.long_name_offset() != 888) {
    throw runtime_error("TransitRoute long_name_offset failed");
  }
  if (route.desc_offset() != 999) {
    throw runtime_error("TransitRoute desc_offset failed");
  }

  // Test bounds for each text offset
  try {
    TransitRoute route(route_type, kMaxNameOffset + 1, 222, 333, 444, 555, 666, 777, 888, 999);
    throw runtime_error("TransitRoute one_stop_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, kMaxNameOffset + 1, 333, 444, 555, 666, 777, 888, 999);
    throw runtime_error("TransitRoute op_by_onestop_id_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, 222, kMaxNameOffset + 1, 444, 555, 666, 777, 888, 999);
    throw runtime_error("TransitRoute op_by_name_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, 222, 333, kMaxNameOffset + 1, 555, 666, 777, 888, 999);
    throw runtime_error("TransitRoute op_by_website_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, kMaxNameOffset + 1, 888, 999);
    throw runtime_error("TransitRoute short_name_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, kMaxNameOffset + 1, 999);
    throw runtime_error("TransitRoute long_name_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
  try {
    TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, 888, kMaxNameOffset + 1);
    throw runtime_error("TransitRoute desc_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
}
} // namespace

int main(void) {
  test::suite suite("transitroute");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into TransitRoute
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
