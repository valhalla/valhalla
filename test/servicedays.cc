#include "test.h"

#include <cstdint>
#include <string>

#include "baldr/graphconstants.h"
#include "mjolnir/servicedays.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TryGetServiceDays(const std::string& begin_date,
                       const std::string& end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  uint64_t days = get_service_days(b, e, days_from_pivot_date(b), dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryGetServiceDays(const std::string& tile_date,
                       const std::string& begin_date,
                       const std::string& end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto t = get_formatted_date(tile_date);
  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  uint64_t days = get_service_days(b, e, days_from_pivot_date(t), dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryIsServiceDaysUsingShift(const std::string& begin_date,
                                const std::string& date,
                                const std::string& end_date,
                                uint64_t days,
                                bool value) {

  uint32_t b = days_from_pivot_date(get_formatted_date(begin_date));
  uint32_t d = days_from_pivot_date(get_formatted_date(date));
  uint32_t e = days_from_pivot_date(get_formatted_date(end_date));
  uint64_t day = d - b;

  bool answer = false;

  if (day <= (e - b)) {
    // Check days bit

    if ((days & (1ULL << day)))
      answer = true;
  }

  if (value != answer)
    throw std::runtime_error("Invalid bits set for service days using shift.  " + begin_date + " " +
                             end_date + " " + std::to_string(days));
}

void TryGetServiceDays(bool check_b_date,
                       const std::string& begin_date,
                       const std::string& date,
                       const std::string& end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  std::string edate = end_date;
  std::string bdate = begin_date;
  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  auto tz = get_tz_db().from_index(get_tz_db().to_index("America/New_York"));
  auto tile_date = days_from_pivot_date(get_formatted_date(iso_date_time(tz)));

  uint64_t days = get_service_days(b, e, tile_date, dow_mask);

  if (check_b_date) {
    if (value != days && begin_date != date && end_date != edate)
      throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                               " " + std::to_string(days));
  } else {
    if (value != days && begin_date != bdate && end_date != date)
      throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                               " " + std::to_string(days));
  }
}

void TryRejectFeed(const std::string& begin_date,
                   const std::string& end_date,
                   uint32_t dow_mask,
                   uint64_t value) {

  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  auto tz = get_tz_db().from_index(get_tz_db().to_index("America/New_York"));

  auto tile_date = days_from_pivot_date(get_formatted_date(iso_date_time(tz)));

  uint64_t days = get_service_days(b, e, tile_date, dow_mask);

  if (value != days)
    throw std::runtime_error("Feed should of been rejected. " + begin_date + " " + end_date + " " +
                             std::to_string(days));
}

void TryAddServiceDays(uint64_t days,
                       const std::string& begin_date,
                       const std::string& end_date,
                       const std::string& added_date,
                       uint64_t value) {

  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  auto a = get_formatted_date(added_date);
  uint64_t result = add_service_day(days, e, days_from_pivot_date(b), a);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + added_date);
}

void TryRemoveServiceDays(uint64_t days,
                          const std::string& begin_date,
                          const std::string& end_date,
                          const std::string& removed_date,
                          uint64_t value) {

  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  auto r = get_formatted_date(removed_date);
  uint64_t result = remove_service_day(days, e, days_from_pivot_date(b), r);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + removed_date);
}

void TryTestServiceEndDate(const std::string& begin_date,
                           const std::string& end_date,
                           const std::string& new_end_date,
                           uint32_t dow_mask) {

  auto b = get_formatted_date(begin_date);
  auto e = get_formatted_date(end_date);
  auto n = get_formatted_date(new_end_date);

  auto tile_date = days_from_pivot_date(b);

  get_service_days(b, e, tile_date, dow_mask);

  if (e != n)
    throw std::runtime_error("End date not cut off at 60 days.");
}

} // namespace

void TestServiceDays() {

  uint32_t dow_mask = kDOWNone;

  // Test just the weekend for 4 days.
  // bits 2 and 3
  dow_mask |= kSaturday;
  dow_mask |= kSunday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 6);

  // Test just the weekend and Friday for 4 days.
  // bits 2 and 3
  dow_mask |= kFriday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 7);

  // Test just the weekend and Friday and Monday for 4 days.
  // bits 2 and 3
  dow_mask |= kMonday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 15);

  // Test just the weekend and Friday and Monday for 4 days.
  // Add Tuesday; however, result should be still 15 as we are only testing 4 days.
  // bits 2 and 3
  dow_mask |= kTuesday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 15);

  // Test everyday for 60 days.
  dow_mask |= kWednesday;
  dow_mask |= kThursday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 1152921504606846975);

  // Test using a date far in the past.  Feed will be rejected.
  TryRejectFeed("2014-09-25", "2014-09-28", dow_mask, 0);
  auto tz = get_tz_db().from_index(get_tz_db().to_index("America/New_York"));

  boost::gregorian::date today = get_formatted_date(iso_date_time(tz));

  boost::gregorian::date startdate = today - boost::gregorian::days(30);
  boost::gregorian::date enddate = today + boost::gregorian::days(59);
  // Test getting the service days from today - 30 days.  Start date should change to today's date.
  TryGetServiceDays(true, to_iso_extended_string(startdate), to_iso_extended_string(today),
                    to_iso_extended_string(enddate), dow_mask, 1152921504606846975);

  startdate = today;
  enddate = today + boost::gregorian::days(100);
  // Test getting the service days from today.  end date should change to today's date + 59.
  TryGetServiceDays(false, to_iso_extended_string(startdate),
                    to_iso_extended_string(today + boost::gregorian::days(59)),
                    to_iso_extended_string(enddate), dow_mask, 1152921504606846975);

  // Test weekends for 60 days.
  dow_mask = kDOWNone;
  dow_mask |= kSaturday;
  dow_mask |= kSunday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 435749860008887046);

  // Test weekends for 60 days plus Columbus Day
  TryAddServiceDays(435749860008887046, "2015-09-25", "2017-09-28", "2015-10-12", 435749860009018118);

  // Test adding 1 day where 21 and 24 already active.
  TryAddServiceDays(9, "2017-02-21", "2017-02-24", "2017-02-22", 11);

  // Test adding 1 day before start day where 21 and 24 already active.
  TryAddServiceDays(9, "2017-02-21", "2017-02-24", "2017-02-20", 9);

  // Test adding 1 day after end day where 21 and 24 already active.
  TryAddServiceDays(9, "2017-02-21", "2017-02-24", "2017-02-25", 9);

  // Test adding 1 day where 21 and 24 already active...should be no change as 21 already active.
  TryAddServiceDays(9, "2017-02-21", "2017-02-24", "2017-02-21", 9);

  // Test removing 1 day where 21, 22, and 24 is active.
  TryRemoveServiceDays(11, "2017-02-21", "2017-02-24", "2017-02-22", 9);

  // Test removing 1 day before start day where 21, 22, and 24 is active.
  TryRemoveServiceDays(11, "2017-02-21", "2017-02-24", "2017-02-20", 11);

  // Test removing 1 day after end where 21, 22, and 24 is active.
  TryRemoveServiceDays(11, "2017-02-21", "2017-02-24", "2017-02-25", 11);

  // Try to add a date out of the date range
  TryAddServiceDays(435749860008887046, "2015-09-25", "2017-09-28", "2018-10-12", 435749860008887046);

  // Test weekends for 60 days remove Columbus Day
  TryRemoveServiceDays(435749860009018118, "2015-09-25", "2017-09-28", "2015-10-12",
                       435749860008887046);

  // Try to remove a date out of the date range
  TryRemoveServiceDays(435749860009018118, "2015-09-25", "2017-09-28", "2018-10-12",
                       435749860009018118);

  // Test weekdays for 60 days.
  dow_mask = kDOWNone;
  dow_mask |= kMonday;
  dow_mask |= kTuesday;
  dow_mask |= kWednesday;
  dow_mask |= kThursday;
  dow_mask |= kFriday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 717171644597959929);

  // Test to confirm that enddate is cut off at 60 days
  TryTestServiceEndDate("2015-09-25", "2017-09-28", "2015-11-23", dow_mask);

  // Start date is after the tile date but before end date.
  TryGetServiceDays("2016-08-03", "2016-09-01", "2016-10-28", dow_mask, 562843568692002816);

  // Start date before tile date.
  TryGetServiceDays("2016-08-03", "2016-07-05", "2016-08-31", dow_mask, 486142951);

  // Start date is in the future.
  TryGetServiceDays("2016-08-03", "2016-10-28", "2016-12-28", dow_mask, 0);
}

void TestIsServiceAvailable() {
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-09", "2016-01-09", 580999813345182728, true);
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-10", "2016-01-09", 580999813345182728, false);
}

int main(void) {
  test::suite suite("servicedays");

  suite.test(TEST_CASE(TestServiceDays));
  suite.test(TEST_CASE(TestIsServiceAvailable));

  return suite.tear_down();
}
