#include "baldr/transitschedule.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes
constexpr size_t kTransitScheduleExpectedSize = 16;

namespace {

TEST(TransitSchedule, Sizeof) {
  EXPECT_EQ(sizeof(TransitSchedule), kTransitScheduleExpectedSize);
}

TEST(TransitSchedule, TestWriteRead) {
  // Test building a transit stop and reading back values
  TransitSchedule sched(15, 0x15, 30);
  EXPECT_EQ(sched.days(), 15);
  EXPECT_EQ(sched.days_of_week(), 0x15);
  EXPECT_EQ(sched.end_day(), 30);

  // Test a valid day within the schedule
  EXPECT_FALSE(sched.IsValid(5, 4, false));

  // Test an invalid day within the schedule
  EXPECT_TRUE(sched.IsValid(35, 16, false));

  // Test IsValid with a valid dow day but after the end day
  EXPECT_TRUE(sched.IsValid(35, 4, false));

  // Check for exceeding max end day
  TransitSchedule sched1(11111001, 0x15, kMaxEndDay + 1);
  EXPECT_EQ(sched1.end_day(), kMaxEndDay) << "TransitSchedule end_day not clamped - test failed";

  // Test bounds for day of the week
  EXPECT_THROW(TransitSchedule sched2(11111001, kAllDaysOfWeek + 1, 31);, std::runtime_error);
}

TEST(TransitSchedule, TestSort) {
  // Sort with same days and same days of week
  TransitSchedule sched1(15, 0x15, 10);
  TransitSchedule sched2(15, 0x15, 30);
  EXPECT_LT(sched1, sched2);

  // Sort with same days, different dow
  TransitSchedule sched3(15, 15, 10);
  TransitSchedule sched4(15, 20, 30);
  EXPECT_LT(sched3, sched4);

  // Sort with different days
  TransitSchedule sched5(15, 15, 10);
  TransitSchedule sched6(25, 20, 30);
  EXPECT_LT(sched5, sched6);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
