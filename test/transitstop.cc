#include "test.h"

#include "baldr/transitstop.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 8 bytes
constexpr size_t kTransitStopExpectedSize = 8;

namespace {

void test_sizeof() {
  if (sizeof(TransitStop) != kTransitStopExpectedSize)
    throw std::runtime_error("TransitStop size should be " +
                             std::to_string(kTransitStopExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(TransitStop)));
}

void TestWriteRead() {
  // Test building a transit stop and reading back values
  uint32_t trav = static_cast<uint32_t>(Traversability::kForward);
  TransitStop transit_stop(555, 777, false, trav);
  if (transit_stop.one_stop_offset() != 555) {
    throw runtime_error("TranstStop one_stop_offset failed");
  }
  if (transit_stop.name_offset() != 777) {
    throw runtime_error("TransitStop name_offset failed");
  }
  if (transit_stop.generated() != false) {
    throw runtime_error("TransitStop generated failed");
  }
  if (transit_stop.traversability() != Traversability::kForward) {
    throw runtime_error("TransitStop traversability failed");
  }

  // Test bounds of one stop offset
  try {
    TransitStop transit_stop(kMaxNameOffset + 1, 777, false, trav);
    throw runtime_error("TransitStop one_stop_offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }

  // Test bounds of name offset
  try {
    TransitStop transit_stop(123, kMaxNameOffset + 1, false, trav);
    throw runtime_error("TransitStop name offset limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
}
} // namespace

int main(void) {
  test::suite suite("transitstop");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into TransitStop
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
