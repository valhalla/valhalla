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

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  date::sys_days b_d = date::sys_days(date::year_month_day(d + date::days(b)));

  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));

  uint64_t days = get_service_days(b_d, e_d, b, dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryGetServiceDays(const std::string& tile_date,
                       const std::string& begin_date,
                       const std::string& end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  date::sys_days b_d = date::sys_days(date::year_month_day(d + date::days(b)));

  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));

  auto t = DateTime::days_from_pivot_date((DateTime::get_formatted_date(tile_date)));

  uint64_t days = get_service_days(b_d, e_d, t, dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryToIsoExtendedString(const std::string& date) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(date)));
  date::sys_days the_d = date::sys_days(date::year_month_day(d + date::days(b)));

  if (date != to_iso_extended_string(the_d))
    throw std::runtime_error("invalid iso string" + to_iso_extended_string(the_d));
}

void TryIsServiceDaysUsingShift(const std::string& begin_date,
                                const std::string& date,
                                const std::string& end_date,
                                uint64_t days,
                                bool value) {

  uint32_t b = DateTime::days_from_pivot_date(DateTime::get_formatted_date(begin_date));
  uint32_t d = DateTime::days_from_pivot_date(DateTime::get_formatted_date(date));
  uint32_t e = DateTime::days_from_pivot_date(DateTime::get_formatted_date(end_date));
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
                       const date::sys_days& begin_date,
                       const date::sys_days& date,
                       const date::sys_days& end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto edate = end_date;
  auto bdate = begin_date;

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto tile_date =
      DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = get_service_days(bdate, edate, tile_date, dow_mask);

  if (check_b_date) {
    if (value != days && begin_date != date && end_date != edate)
      throw std::runtime_error("Invalid bits set for service days. Begin date != date. " +
                               std::to_string(days));
  } else {
    if (value != days && begin_date != bdate && end_date != date)
      throw std::runtime_error("Invalid bits set for service days. End date != date. " +
                               std::to_string(days));
  }
}

void TryRejectFeed(const std::string& begin_date,
                   const std::string& end_date,
                   uint32_t dow_mask,
                   uint64_t value) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  date::sys_days b_d = date::sys_days(date::year_month_day(d + date::days(b)));

  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto tile_date =
      DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = get_service_days(b_d, e_d, tile_date, dow_mask);

  if (value != days)
    throw std::runtime_error("Feed should of been rejected. " + begin_date + " " + end_date + " " +
                             std::to_string(days));
}

void TryAddServiceDays(uint64_t days,
                       const std::string& begin_date,
                       const std::string& end_date,
                       const std::string& added_date,
                       uint64_t value) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));

  auto a = DateTime::days_from_pivot_date((DateTime::get_formatted_date(added_date)));
  date::sys_days a_d = date::sys_days(date::year_month_day(d + date::days(a)));

  uint64_t result = add_service_day(days, e_d, b, a_d);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + added_date);
}

void TryRemoveServiceDays(uint64_t days,
                          const std::string& begin_date,
                          const std::string& end_date,
                          const std::string& removed_date,
                          uint64_t value) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));

  auto r = DateTime::days_from_pivot_date((DateTime::get_formatted_date(removed_date)));
  date::sys_days r_d = date::sys_days(date::year_month_day(d + date::days(r)));

  uint64_t result = remove_service_day(days, e_d, b, r_d);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + removed_date);
}

void TryTestServiceEndDate(const std::string& begin_date,
                           const std::string& end_date,
                           const std::string& new_end_date,
                           uint32_t dow_mask) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  auto b = DateTime::days_from_pivot_date((DateTime::get_formatted_date(begin_date)));
  auto e = DateTime::days_from_pivot_date((DateTime::get_formatted_date(end_date)));
  auto n = DateTime::days_from_pivot_date((DateTime::get_formatted_date(new_end_date)));

  date::sys_days b_d = date::sys_days(date::year_month_day(d + date::days(b)));
  date::sys_days e_d = date::sys_days(date::year_month_day(d + date::days(e)));
  date::sys_days n_d = date::sys_days(date::year_month_day(d + date::days(n)));

  get_service_days(b_d, e_d, b, dow_mask);

  if (e_d != n_d)
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
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto today = DateTime::get_formatted_date(DateTime::iso_date_time(tz));

  auto d = date::floor<date::days>(today);
  date::sys_days startdate = date::sys_days(date::year_month_day(d + date::days(30)));
  date::sys_days enddate = date::sys_days(date::year_month_day(d + date::days(59)));

  // Test getting the service days from today - 30 days.  Start date should change to today's date.
  TryGetServiceDays(true, startdate, date::sys_days(date::year_month_day(d)), enddate, dow_mask,
                    1152921504606846975);

  startdate = date::sys_days(date::year_month_day(d));
  enddate = date::sys_days(date::year_month_day(d + date::days(100)));
  // Test getting the service days from today.  end date should change to today's date + 59.
  TryGetServiceDays(false, startdate, date::sys_days(date::year_month_day(d + date::days(59))),
                    enddate, dow_mask, 1152921504606846975);

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

void TestIsoString() {
  TryToIsoExtendedString("2015-11-11");
  TryToIsoExtendedString("2018-09-13");
}

void TestIsServiceAvailable() {
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-09", "2016-01-09", 580999813345182728, true);
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-10", "2016-01-09", 580999813345182728, false);
}

int main(void) {
  test::suite suite("servicedays");

  suite.test(TEST_CASE(TestServiceDays));
  suite.test(TEST_CASE(TestIsServiceAvailable));
  suite.test(TEST_CASE(TestIsoString));

  return suite.tear_down();
}
