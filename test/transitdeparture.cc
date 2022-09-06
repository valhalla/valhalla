#include "baldr/transitdeparture.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 24 bytes
constexpr size_t kTransitDepartureExpectedSize = 24;

namespace {

TEST(TransitDeparture, test_sizeof) {
  EXPECT_EQ(sizeof(TransitDeparture), kTransitDepartureExpectedSize);
}

// Test building a transit departure (fixed schedule) and reading back values
TEST(TransitDeparture, TestWriteReadFixed) {
  TransitDeparture dep(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  EXPECT_EQ(dep.type(), kFixedSchedule);
  EXPECT_EQ(dep.lineid(), 111);
  EXPECT_EQ(dep.tripid(), 222);
  EXPECT_EQ(dep.routeindex(), 333);
  EXPECT_EQ(dep.blockid(), 444);
  EXPECT_EQ(dep.headsign_offset(), 555);
  EXPECT_EQ(dep.departure_time(), 9000);
  EXPECT_EQ(dep.elapsed_time(), 90);
  EXPECT_EQ(dep.schedule_index(), 23);
  EXPECT_EQ(dep.wheelchair_accessible(), false);
  EXPECT_EQ(dep.bicycle_accessible(), true);

  // Test bounds for fixed schedule departures. When above limits an exception should be thrown
  // Catch these exceptions - which means test passes. If an exception is not thrown the test
  // fails.
  EXPECT_THROW(TransitDeparture dep2(kMaxTransitLineId + 100, 222, 333, 444, 555, 9000, 90, 23, false,
                                     true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, kMaxTripId + 10, 333, 444, 555, 9000, 90, 23, false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, kMaxTransitRoutes + 10, 444, 555, 9000, 90, 23, false,
                                    true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, kMaxTransitBlockId + 10, 555, 9000, 90, 23, false,
                                    true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, kMaxNameOffset + 10, 9000, 90, 23, false,
                                    true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, 555, kMaxTransitDepartureTime + 10, 90, 23,
                                    false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, 555, 9000, 90, kMaxTransitSchedules + 10,
                                    false, true),
               std::runtime_error);

  // Test that elapsed time is clamped
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, kMaxTransitElapsedTime + 10, 23, false, true);
  EXPECT_EQ(dep1.elapsed_time(), kMaxTransitElapsedTime)
      << "TransitDeparture elapsed_time clamping failed";
}

// Test building a transit departure (frequency based schedule) and reading back values
TEST(TransitDeparture, TestWriteReadFreq) {
  TransitDeparture dep(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  EXPECT_EQ(dep.type(), kFrequencySchedule);
  EXPECT_EQ(dep.lineid(), 111);
  EXPECT_EQ(dep.tripid(), 222);
  EXPECT_EQ(dep.routeindex(), 333);
  EXPECT_EQ(dep.blockid(), 444);
  EXPECT_EQ(dep.headsign_offset(), 555);
  EXPECT_EQ(dep.departure_time(), 9000);
  EXPECT_EQ(dep.end_time(), 18000);
  EXPECT_EQ(dep.frequency(), 600);
  EXPECT_EQ(dep.elapsed_time(), 90);
  EXPECT_EQ(dep.schedule_index(), 23);
  EXPECT_EQ(dep.wheelchair_accessible(), false);
  EXPECT_EQ(dep.bicycle_accessible(), true);

  // Test bounds for frequency based departures. When above limits an exception should be thrown
  // Catch these exceptions - which means test passes. If an exception is not thrown the test
  // fails.
  EXPECT_THROW(TransitDeparture dep(kMaxTransitLineId + 10, 222, 333, 444, 555, 9000, 18000, 600, 90,
                                    23, false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, kMaxTripId + 10, 333, 444, 555, 9000, 18000, 600, 90, 23,
                                    false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, kMaxTransitRoutes + 10, 444, 555, 9000, 18000, 600, 90,
                                    23, false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, kMaxTransitBlockId + 10, 555, 9000, 18000, 600, 90,
                                    23, false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, kMaxNameOffset + 10, 9000, 18000, 600, 90, 23,
                                    false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, 555, kMaxTransitDepartureTime + 10, 18000,
                                    600, 90, 23, false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, 555, 9000, kMaxEndTime + 10, 600, 90, 23,
                                    false, true),
               std::runtime_error);
  EXPECT_THROW(TransitDeparture dep(111, 222, 333, 444, 555, 9000, 18000, kMaxFrequency + 10, 90, 23,
                                    false, true),
               std::runtime_error)
      << "Frequency limit check failed";

  // Test that elapsed time is clamped
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, kMaxTransitElapsedTime + 10, 23, false, true);
  EXPECT_EQ(dep1.elapsed_time(), kMaxTransitElapsedTime)
      << "TransitDeparture elapsed_time clamping failed";
}

TEST(TransitDeparture, TestSort) {
  // Equal line Id, type, and trip Id. Sort by departure time
  TransitDeparture dep1(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep2(111, 222, 333, 444, 555, 10000, 90, 23, false, true);
  EXPECT_LT(dep1, dep2);

  // Equal line Id, and type (both of these are fixed) - sort bu trip Id
  TransitDeparture dep3(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep4(111, 2222, 333, 444, 555, 9000, 90, 23, false, true);
  EXPECT_LT(dep3, dep4);

  // Equal line Id, sort by type (fixed before frequency)
  TransitDeparture dep5(111, 222, 333, 444, 555, 9000, 90, 23, false, true);
  TransitDeparture dep6(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  EXPECT_LT(dep5, dep6);

  // Should sort by line Id
  TransitDeparture dep7(111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  TransitDeparture dep8(1111, 222, 333, 444, 555, 9000, 18000, 600, 90, 23, false, true);
  EXPECT_LT(dep7, dep8);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
