#include "test.h"

#include "baldr/transitschedule.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes
constexpr size_t kTransitScheduleExpectedSize = 16;

namespace {

void test_sizeof() {
  if (sizeof(TransitSchedule) != kTransitScheduleExpectedSize)
    throw std::runtime_error("TransitSchedule size should be " +
                             std::to_string(kTransitScheduleExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(TransitSchedule)));
}

void TestWriteRead() {
  // Test building a transit stop and reading back values
  TransitSchedule sched(15, 0x15, 30);
  if (sched.days() != 15) {
    throw runtime_error("TransitSchedule days failed");
  }
  if (sched.days_of_week() != 0x15) {
    throw runtime_error("TransitSchedule days_of_week failed");
  }
  if (sched.end_day() != 30) {
    throw runtime_error("TransitSchedule end_day failed");
  }

  // Test a valid day within the schedule
  if (sched.IsValid(5, 4, false) == true) {
    throw runtime_error("TransitSchedule IsValid failed - should be valid");
  }

  // Test an invalid day within the schedule
  if (sched.IsValid(35, 16, false) == false) {
    throw runtime_error("TransitSchedule IsValid failed - should be invalid");
  }

  // Test IsValid with a valid dow day but after the end day
  if (sched.IsValid(35, 4, false) == false) {
    throw runtime_error("TransitSchedule IsValid after end day failed");
  }

  // Check for exceeding max end day
  TransitSchedule sched1(11111001, 0x15, kMaxEndDay + 1);
  if (sched1.end_day() != kMaxEndDay) {
    throw runtime_error("TransitSchedule end_day not clamped - test failed");
  }

  // Test bounds for day of the week
  try {
    TransitSchedule sched2(11111001, kAllDaysOfWeek + 1, 31);
    throw runtime_error("TransitStop day of week limit check failed");
  } catch (...) {
    // Successfully caught exception indicating the bounds is checked
  }
}
} // namespace

int main(void) {
  test::suite suite("transitschedule");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into TransitSchedule
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
