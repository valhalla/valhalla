#include "test.h"

#include "baldr/transitdeparture.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 24 bytes
constexpr size_t kTransitDepartureExpectedSize = 24;

namespace {

void test_sizeof() {
  if (sizeof(TransitDeparture) != kTransitDepartureExpectedSize)
    throw std::runtime_error("TransitDeparture size should be " +
                             std::to_string(kTransitDepartureExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(TransitDeparture)));
}

// Test building a transit departure (fixed schedule) and reading back values
void TestWriteReadFixed() {
  TransitDeparture dep(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  if (dep.type() != kFixedSchedule) {
    throw runtime_error("TransitDeparture type (fixed) failed");
  }
  if (dep.lineid() != 111) {
    throw runtime_error("TransitDeparture lineid failed");
  }
  if (dep.tripid() != 222) {
    throw runtime_error("TransitDeparture tripid failed");
  }
  if (dep.routeid() != 333) {
    throw runtime_error("TransitDeparture routeid failed");
  }
  if (dep.blockid() != 444) {
    throw runtime_error("TransitDeparture blockid failed");
  }
  if (dep.headsign_offset() != 555) {
    throw runtime_error("TransitDeparture headsign_offset failed");
  }
  if (dep.departure_time() != 9000) {
    throw runtime_error("TransitDeparture departure_time failed");
  }
  if (dep.elapsed_time() != 90) {
    throw runtime_error("TransitDeparture elapsed_time failed");
  }
  if (dep.schedule_index() != 23) {
    throw runtime_error("TransitDeparture schedule_index failed");
  }
  if (dep.wheelchair_accessible() != false) {
    throw runtime_error("TransitDeparture wheelchair_accessible failed");
  }
  if (dep.bicycle_accessible() != true) {
    throw runtime_error("TransitDeparture bicycle_accessible failed");
  }

  // Test bounds for fixed schedule departures. When above limits an exception should be thrown
  // Catch these exceptions - which means test passes. If an exception is not thrown the test
  // fails.
  try {
    TransitDeparture dep2(kMaxTransitLineId + 100, 222, 333, 444, 555, 9000, 90, 23, false, true);
    throw runtime_error("TransitDeparture line Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, kMaxTripId + 10, 333, 444, 555, 9000, 90, 23, false, true);
    throw runtime_error("TransitDeparture trip Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, kMaxTransitRoutes + 10, 444, 555, 9000, 90, 23, false, true);
    throw runtime_error("TransitDeparture route Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, kMaxTransitBlockId + 10, 555, 9000, 90, 23, false, true);
    throw runtime_error("TransitDeparture block Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, kMaxNameOffset + 10, 9000, 90, 23, false, true);
    throw runtime_error("TransitDeparture headsign offset limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, 555, kMaxTransitDepartureTime + 10, 90, 23, false, true);
    throw runtime_error("TransitDeparture departure time limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, 555, 9000, 90, kMaxTransitSchedules + 10, false, true);
    throw runtime_error("TransitDeparture transit schedule index limit check failed");
  } catch (...) {}

  // Test that elapsed time is clamped
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, kMaxTransitElapsedTime + 10, 23, false, true);
  if (dep1.elapsed_time() != kMaxTransitElapsedTime) {
    throw runtime_error("TransitDeparture elapsed_time clamping failed");
  }
}

// Test building a transit departure (frequency based schedule) and reading back values
void TestWriteReadFreq() {
  TransitDeparture dep(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  if (dep.type() != kFrequencySchedule) {
    throw runtime_error("TransitDeparture type (freq) failed");
  }
  if (dep.lineid() != 111) {
    throw runtime_error("TransitDeparture lineid failed");
  }
  if (dep.tripid() != 222) {
    throw runtime_error("TransitDeparture tripid failed");
  }
  if (dep.routeid() != 333) {
    throw runtime_error("TransitDeparture routeid failed");
  }
  if (dep.blockid() != 444) {
    throw runtime_error("TransitDeparture blockid failed");
  }
  if (dep.headsign_offset() != 555) {
    throw runtime_error("TransitDeparture headsign_offset failed");
  }
  if (dep.departure_time() != 9000) {
    throw runtime_error("TransitDeparture departure_time failed");
  }
  if (dep.end_time() != 18000) {
    throw runtime_error("TransitDeparture end_time failed");
  }
  if (dep.frequency() != 600) {
    throw runtime_error("TransitDeparture frequency failed");
  }
  if (dep.elapsed_time() != 90) {
    throw runtime_error("TransitDeparture elapsed_time failed");
  }
  if (dep.schedule_index() != 23) {
    throw runtime_error("TransitDeparture schedule_index failed");
  }
  if (dep.wheelchair_accessible() != false) {
    throw runtime_error("TransitDeparture wheelchair_accessible failed");
  }
  if (dep.bicycle_accessible() != true) {
    throw runtime_error("TransitDeparture bicycle_accessible failed");
  }

  // Test bounds for frequency based departures. When above limits an exception should be thrown
  // Catch these exceptions - which means test passes. If an exception is not thrown the test
  // fails.
  try {
    TransitDeparture dep(kMaxTransitLineId + 10, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false,
                         true);
    throw runtime_error("TransitDeparture line Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, kMaxTripId + 10, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
    throw runtime_error("TransitDeparture trip Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, kMaxTransitRoutes + 10, 444, 555, 9000, 18000, 600, 90, 23, false,
                         true);
    throw runtime_error("TransitDeparture route Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, kMaxTransitBlockId + 10, 555, 9000, 18000, 600, 90, 23, false,
                         true);
    throw runtime_error("TransitDeparture block Id limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, kMaxNameOffset + 10, 9000, 18000, 600, 90, 23, false,
                         true);
    throw runtime_error("TransitDeparture headsign offset limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, 555, kMaxTransitDepartureTime + 10, 18000, 600, 90, 23,
                         false, true);
    throw runtime_error("TransitDeparture departure time limit check failed");
  } catch (...) {}
  try {
    TransitDeparture dep(111, 222, 333, 444, 555, 9000, kMaxEndTime + 10, 600, 90, 23, false, true);
    throw runtime_error("TransitDeparture end time limit check failed");
  } catch (...) {}
  try {
    printf("Check for frequency limit\n");
    TransitDeparture dep(111, 222, 333, 444, 555, 9000, 18000, kMaxFrequency + 10, 90, 23, false,
                         true);
    throw runtime_error("TransitDeparture frequency limit check failed");
  } catch (...) {}

  // Test that elapsed time is clamped
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, kMaxTransitElapsedTime + 10, 23, false, true);
  if (dep1.elapsed_time() != kMaxTransitElapsedTime) {
    throw runtime_error("TransitDeparture elapsed_time clamping failed");
  }
}

void TestSort() {
  // Equal line Id, type, and trip Id. Sort by departure time
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep2(111, 222, 333, 444, 555, 10000, 90, 23, false, true);
  if (!(dep1 < dep2)) {
    throw runtime_error("TransitDeparture sort (1) failed");
  }

  // Equal line Id, and type (both of these are fixed) - sort bu trip Id
  TransitDeparture dep3(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep4(111, 2222, 333, 444, 555, 9000, 90, 23, false, true);
  if (!(dep3 < dep4)) {
    throw runtime_error("TransitDeparture sort (2) failed");
  }

  // Equal line Id, sort by type (fixed before frequency)
  TransitDeparture dep5(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep6(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  if (!(dep5 < dep6)) {
    throw runtime_error("TransitDeparture sort (3) failed");
  }

  // Should sort by line Id
  TransitDeparture dep7(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  TransitDeparture dep8(1111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  if (!(dep7 < dep8)) {
    throw runtime_error("TransitDeparture sort (4) failed");
  }
}
} // namespace

int main(void) {
  test::suite suite("transitdeparture");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into fixed transit departure
  suite.test(TEST_CASE(TestWriteReadFixed));

  // Write to file and read into frequency based transit departure
  suite.test(TEST_CASE(TestWriteReadFreq));

  // Test sorting
  suite.test(TEST_CASE(TestSort));

  return suite.tear_down();
}
