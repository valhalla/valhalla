#include "test.h"
#include <cstdint>

#include <bitset>
#include <string>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include <algorithm>
#include <boost/algorithm/string/split.hpp>

using namespace std;
using namespace valhalla::baldr;

namespace {

std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value, std::bind1st(std::equal_to<char>(), delim),
                          boost::algorithm::token_compress_on);
  return tokens;
}

void TryGetDaysFromPivotDate(std::string date_time, uint32_t expected_days) {
  if (DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time)) != expected_days) {
    throw std::runtime_error(std::string("Incorrect number of days from ") + date_time);
  }
}

void TryGetDOW(std::string date_time, uint32_t expected_dow) {

  if (DateTime::day_of_week_mask(date_time) != expected_dow) {
    throw std::runtime_error(std::string("Incorrect dow ") + date_time);
  }
}

void TryGetDuration(std::string date_time, uint32_t seconds, std::string expected_date_time) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  if (DateTime::get_duration(date_time, seconds, tz) != expected_date_time) {
    throw std::runtime_error(std::string("Incorrect duration ") +
                             DateTime::get_duration(date_time, seconds, tz) + std::string(" ") +
                             expected_date_time);
  }
}

void TryGetSecondsFromMidnight(std::string date_time, uint32_t expected_seconds) {
  if (DateTime::seconds_from_midnight(date_time) != expected_seconds) {
    throw std::runtime_error(std::string("Incorrect number of seconds from ") + date_time);
  }
}

void TryIsoDateTime() {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string current_date_time = DateTime::iso_date_time(tz);
  std::string time;
  std::size_t found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/Chicago"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("Africa/Porto-Novo"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }
}

void TryGetServiceDays(std::string begin_date,
                       std::string end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  uint64_t days = DateTime::get_service_days(b, e, DateTime::days_from_pivot_date(b), dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryGetServiceDays(std::string tile_date,
                       std::string begin_date,
                       std::string end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  auto t = DateTime::get_formatted_date(tile_date);
  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  uint64_t days = DateTime::get_service_days(b, e, DateTime::days_from_pivot_date(t), dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryIsServiceAvailable(std::string begin_date,
                           std::string date,
                           std::string end_date,
                           uint64_t days,
                           bool value) {

  auto b = DateTime::days_from_pivot_date(DateTime::get_formatted_date(begin_date));
  auto e = DateTime::days_from_pivot_date(DateTime::get_formatted_date(end_date));
  auto d = DateTime::days_from_pivot_date(DateTime::get_formatted_date(date));

  if (value != DateTime::is_service_available(days, b, d, e))
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date +
                             " " + std::to_string(days));
}

void TryIsServiceDaysUsingShift(std::string begin_date,
                                std::string date,
                                std::string end_date,
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
                       std::string begin_date,
                       std::string date,
                       std::string end_date,
                       uint32_t dow_mask,
                       uint64_t value) {

  std::string edate = end_date;
  std::string bdate = begin_date;
  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto tile_date =
      DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = DateTime::get_service_days(b, e, tile_date, dow_mask);

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

void TryRejectFeed(std::string begin_date, std::string end_date, uint32_t dow_mask, uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  auto tile_date =
      DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = DateTime::get_service_days(b, e, tile_date, dow_mask);

  if (value != days)
    throw std::runtime_error("Feed should of been rejected. " + begin_date + " " + end_date + " " +
                             std::to_string(days));
}

void TryAddServiceDays(uint64_t days,
                       std::string begin_date,
                       std::string end_date,
                       std::string added_date,
                       uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto a = DateTime::get_formatted_date(added_date);
  uint64_t result = DateTime::add_service_day(days, e, DateTime::days_from_pivot_date(b), a);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + added_date);
}

void TryRemoveServiceDays(uint64_t days,
                          std::string begin_date,
                          std::string end_date,
                          std::string removed_date,
                          uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto r = DateTime::get_formatted_date(removed_date);
  uint64_t result = DateTime::remove_service_day(days, e, DateTime::days_from_pivot_date(b), r);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + removed_date);
}

void TryTestServiceEndDate(std::string begin_date,
                           std::string end_date,
                           std::string new_end_date,
                           uint32_t dow_mask) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto n = DateTime::get_formatted_date(new_end_date);

  auto tile_date = DateTime::days_from_pivot_date(b);

  DateTime::get_service_days(b, e, tile_date, dow_mask);

  if (e != n)
    throw std::runtime_error("End date not cut off at 60 days.");
}

void TryTestIsValid(std::string date, bool return_value) {

  auto ret = DateTime::is_iso_local(date);
  if (ret != return_value)
    throw std::runtime_error("Test is_iso_local failed: " + date);
}

void TryTestDST(const bool is_depart_at,
                const uint64_t origin_seconds,
                const uint64_t dest_seconds,
                std::string o_value,
                std::string d_value) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string iso_origin, iso_dest;
  DateTime::seconds_to_date(is_depart_at, origin_seconds, dest_seconds, tz, tz, iso_origin, iso_dest);

  if (iso_origin != o_value)
    throw std::runtime_error("Test origin DST failed.  Expected: " + o_value + " but received " +
                             iso_origin);

  if (iso_dest != d_value)
    throw std::runtime_error("Test destination DST failed.  Expected: " + d_value + " but received " +
                             iso_dest);
}

void TryIsRestricted(const TimeDomain td, const std::string date, const bool expected_value) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  if (DateTime::is_restricted(td.type(), td.begin_hrs(), td.begin_mins(), td.end_hrs(), td.end_mins(),
                              td.dow(), td.begin_week(), td.begin_month(), td.begin_day_dow(),
                              td.end_week(), td.end_month(), td.end_day_dow(),
                              DateTime::seconds_since_epoch(date, tz), tz) != expected_value) {

    throw std::runtime_error("Is Restricted " + date +
                             " test failed.  Expected: " + std::to_string(expected_value));
  }
}

void TryTestTimezoneDiff(const bool is_depart,
                         const uint64_t date_time,
                         const std::string expected1,
                         const std::string expected2,
                         const std::string time_zone1,
                         const std::string time_zone2) {

  uint64_t dt = date_time;

  auto tz1 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone1));
  auto tz2 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone2));

  std::cout << DateTime::seconds_to_date(dt, tz1) << std::endl;

  DateTime::timezone_diff(is_depart, dt, tz1, tz2);
  if (DateTime::seconds_to_date(dt, tz1) != expected1)
    throw std::runtime_error("Timezone Diff test #1: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected1 + " but got " +
                             DateTime::seconds_to_date(dt, tz1));

  if (DateTime::seconds_to_date(dt, tz2) != expected2)
    throw std::runtime_error("Timezone Diff test #2: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected2 + " but got " +
                             DateTime::seconds_to_date(dt, tz2));
}

void TryConditionalRestrictions(const std::string condition,
                                const std::vector<uint64_t> expected_values) {

  std::vector<uint64_t> results = DateTime::get_time_range(condition);

  for (uint32_t x = 0; x < results.size(); x++) {
    TimeDomain res = TimeDomain(results.at(x));

    /* used for creating new tests.
    std::cout << condition << " type " << res.type() << " dow " << res.dow() << " begin month " <<
    res.begin_month() << " begin day " << res.begin_day() << " begin week " <<
    res.begin_week() << " begin hrs " << res.begin_hrs() << " begin mins " <<
    res.begin_mins() << " end month " << res.end_month() << " end day " <<
    res.end_day() << " end week " << res.end_week() << " end hrs " <<
    res.end_hrs() << " end mins " << res.end_mins() << std::endl;*/

    if (res.td_value() != expected_values.at(x))
      throw std::runtime_error("Time domain " + condition +
                               " test failed.  Expected: " + std::to_string(expected_values.at(x)) +
                               " but received " + std::to_string(res.td_value()));
  }
}

void TryConditionalRestrictions(const std::string condition,
                                const uint32_t index,
                                const uint32_t type,
                                const uint32_t dow,
                                const uint32_t begin_month,
                                const uint32_t begin_day,
                                const uint32_t begin_week,
                                const uint32_t begin_hrs,
                                const uint32_t begin_mins,
                                const uint32_t end_month,
                                const uint32_t end_day,
                                const uint32_t end_week,
                                const uint32_t end_hrs,
                                const uint32_t end_mins) {

  std::vector<uint64_t> results = DateTime::get_time_range(condition);

  TimeDomain res = TimeDomain(results.at(index));

  if (res.type() != type || res.dow() != dow || res.begin_month() != begin_month ||
      res.begin_day_dow() != begin_day || res.begin_week() != begin_week ||
      res.begin_hrs() != begin_hrs || res.begin_mins() != begin_mins ||
      res.end_month() != end_month || res.end_day_dow() != end_day || res.end_week() != end_week ||
      res.end_hrs() != end_hrs || res.end_mins() != end_mins) {
    throw std::runtime_error(
        "Time domain " + condition + " test failed.  Output: " + std::to_string(res.type()) + " " +
        std::to_string(res.dow()) + " " + std::to_string(res.begin_month()) + " " +
        std::to_string(res.begin_day_dow()) + " " + std::to_string(res.begin_week()) + " " +
        std::to_string(res.begin_hrs()) + " " + std::to_string(res.begin_mins()) + " " +
        std::to_string(res.end_month()) + " " + std::to_string(res.end_day_dow()) + " " +
        std::to_string(res.end_week()) + " " + std::to_string(res.end_hrs()) + " " +
        std::to_string(res.end_mins()));
  }
}

} // namespace

void TestGetDaysFromPivotDate() {
  TryGetDaysFromPivotDate("20140101", 0);
  TryGetDaysFromPivotDate("20140102", 1);
  TryGetDaysFromPivotDate("19990101", 0);
  TryGetDaysFromPivotDate("20150506", 490);
  TryGetDaysFromPivotDate("2015-05-06", 490);

  TryGetDaysFromPivotDate("20140101T07:01", 0);
  TryGetDaysFromPivotDate("20140102T15:00", 1);
  TryGetDaysFromPivotDate("19990101T:00:00", 0);
  TryGetDaysFromPivotDate("2015-05-06T08:00", 490);
}

void TestDOW() {

  TryGetDOW("20140101", kWednesday);
  TryGetDOW("20140102", kThursday);
  TryGetDOW("19990101", kDOWNone);
  TryGetDOW("20150508", kFriday);
  TryGetDOW("2015-05-08", kFriday);

  TryGetDOW("20140101T07:01", kWednesday);
  TryGetDOW("20140102T15:00", kThursday);
  TryGetDOW("19990101T:00:00", kDOWNone);
  TryGetDOW("2015-05-09T08:00", kSaturday);
}

void TestDuration() {

  TryGetDuration("20140101", 30, "2014-01-01T00:00-05:00 EST");
  TryGetDuration("20140102", 60, "2014-01-02T00:01-05:00 EST");
  TryGetDuration("2014-01-02", 60, "2014-01-02T00:01-05:00 EST");
  TryGetDuration("19990101", 89, "");
  TryGetDuration("20140101T07:01", 61, "2014-01-01T07:02-05:00 EST");
  TryGetDuration("20140102T15:00", 61, "2014-01-02T15:01-05:00 EST");
  TryGetDuration("20140102T15:00", 86400, "2014-01-03T15:00-05:00 EST");
  TryGetDuration("20160714", 60, "2016-07-14T00:01-04:00 EDT");
}

void TestIsoDateTime() {
  TryIsoDateTime();
}

void TestGetSecondsFromMidnight() {
  TryGetSecondsFromMidnight("00:00:00", 0);
  TryGetSecondsFromMidnight("01:00:00", 3600);
  TryGetSecondsFromMidnight("05:34:34", 20074);
  TryGetSecondsFromMidnight("26:16:01", 94561);
  TryGetSecondsFromMidnight("36:16:01", 130561);
  TryGetSecondsFromMidnight("24:01:01", 86461);

  TryGetSecondsFromMidnight("2015-05-06T00:00:00", 0);
  TryGetSecondsFromMidnight("2015-05-06T01:00", 3600);
  TryGetSecondsFromMidnight("2015-05-06T05:34:34", 20074);
  TryGetSecondsFromMidnight("2015-05-06T26:16", 94560);
  TryGetSecondsFromMidnight("2015-05-06T36:16", 130560);
  TryGetSecondsFromMidnight("2015-05-06T24:01:01", 86461);
}

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

  boost::gregorian::date today = DateTime::get_formatted_date(DateTime::iso_date_time(tz));

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
  TryIsServiceAvailable("2015-11-11", "2016-01-09", "2016-01-09", 580999813345182728, true);
  TryIsServiceAvailable("2015-11-11", "2016-01-10", "2016-01-09", 580999813345182728, false);

  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-09", "2016-01-09", 580999813345182728, true);
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-10", "2016-01-09", 580999813345182728, false);
}

void TestIsValid() {
  TryTestIsValid("2015-05-06T01:00", true);
  TryTestIsValid("2015/05-06T01:00", false);
  TryTestIsValid("2015-05/06T01:00", false);
  TryTestIsValid("2015-05-06X01:00", false);
  TryTestIsValid("2015-05-06T01-00", false);
  TryTestIsValid("AAAa-05-06T01:00", false);
  TryTestIsValid("2015-05-06T24:00", false);

  TryTestIsValid("1983-02-30T24:01", false);
  TryTestIsValid("2015-13-06T24:01", false);
  TryTestIsValid("2015-05-06T24:60", false);
  TryTestIsValid("2015-05-06T26:02", false);
  TryTestIsValid("2015-05-06T23:59", true);
  TryTestIsValid("2015-05-06T-3:-9", false);

  TryTestIsValid("2015-05-06T01:0A", false);
  TryTestIsValid("2015-05-06T01", false);
  TryTestIsValid("01:00", false);
  TryTestIsValid("aefopijafepij", false);
}

void TestDST() {

  // bunch of tests for the start and end of dst using startat and arriveby

  TryTestDST(true, 1457845200, 1457858104, "2016-03-13T00:00-05:00", "2016-03-13T04:35-04:00");
  TryTestDST(true, 1457848800, 1457860508, "2016-03-13T01:00-05:00", "2016-03-13T05:15-04:00");
  TryTestDST(true, 1457852100, 1457853188, "2016-03-13T01:55-05:00", "2016-03-13T03:13-04:00");
  TryTestDST(true, 1457852400, 1457853488, "2016-03-13T03:00-04:00", "2016-03-13T03:18-04:00");
  TryTestDST(true, 1457854200, 1457859493, "2016-03-13T03:30-04:00", "2016-03-13T04:58-04:00");
  TryTestDST(true, 1457855700, 1457856788, "2016-03-13T03:55-04:00", "2016-03-13T04:13-04:00");
  TryTestDST(true, 1457852400, 1457853488, "2016-03-13T03:00-04:00", "2016-03-13T03:18-04:00");
  TryTestDST(true, 1457853300, 1457854388, "2016-03-13T03:15-04:00", "2016-03-13T03:33-04:00");

  TryTestDST(true, 1478406600, 1478407484, "2016-11-06T00:30-04:00", "2016-11-06T00:44-04:00");
  TryTestDST(true, 1478408340, 1478409215, "2016-11-06T00:59-04:00", "2016-11-06T01:13-04:00");
  TryTestDST(true, 1478408340, 1478413633, "2016-11-06T00:59-04:00", "2016-11-06T01:27-05:00");
  TryTestDST(true, 1478413800, 1478414684, "2016-11-06T01:30-05:00", "2016-11-06T01:44-05:00");
  TryTestDST(true, 1478413800, 1478418104, "2016-11-06T01:30-05:00", "2016-11-06T02:41-05:00");
  TryTestDST(true, 1478415600, 1478420893, "2016-11-06T02:00-05:00", "2016-11-06T03:28-05:00");
  TryTestDST(true, 1478412000, 1478417293, "2016-11-06T01:00-05:00", "2016-11-06T02:28-05:00");
  TryTestDST(true, 1478419200, 1478424493, "2016-11-06T03:00-05:00", "2016-11-06T04:28-05:00");
  TryTestDST(true, 1478408340, 1478420602, "2016-11-06T00:59-04:00", "2016-11-06T03:23-05:00");
  TryTestDST(true, 1479016740, 1479029002, "2016-11-13T00:59-05:00", "2016-11-13T04:23-05:00");

  TryTestDST(false, 1457847695, 1457848800, "2016-03-13T00:41-05:00", "2016-03-13T01:00-05:00");
  TryTestDST(false, 1457851295, 1457852400, "2016-03-13T01:41-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(false, 1457853095, 1457854200, "2016-03-13T03:11-04:00", "2016-03-13T03:30-04:00");
  TryTestDST(false, 1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false, 1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false, 1457846553, 1457852400, "2016-03-13T00:22-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(false, 1457847453, 1457852700, "2016-03-13T00:37-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false, 1457933853, 1457939100, "2016-03-14T01:37-04:00", "2016-03-14T03:05-04:00");
  TryTestDST(false, 1457848559, 1457856300, "2016-03-13T00:55-05:00", "2016-03-13T04:05-04:00");
  TryTestDST(false, 1457847978, 1457856000, "2016-03-13T00:46-05:00", "2016-03-13T04:00-04:00");
  TryTestDST(false, 1457934378, 1457942400, "2016-03-14T01:46-04:00", "2016-03-14T04:00-04:00");

  TryTestDST(false, 1478406871, 1478415600, "2016-11-06T00:34-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(false, 1478493271, 1478502000, "2016-11-06T23:34-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(false, 1478410471, 1478419200, "2016-11-06T01:34-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(false, 1478496871, 1478505600, "2016-11-07T00:34-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(false, 1478414071, 1478422800, "2016-11-06T01:34-05:00", "2016-11-06T04:00-05:00");
  TryTestDST(false, 1478500471, 1478509200, "2016-11-07T01:34-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(false, 1478410406, 1478422800, "2016-11-06T01:33-04:00", "2016-11-06T04:00-05:00");
  TryTestDST(false, 1478496806, 1478509200, "2016-11-07T00:33-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(false, 1478406806, 1478419200, "2016-11-06T00:33-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(false, 1478493206, 1478505600, "2016-11-06T23:33-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(false, 1478403206, 1478415600, "2016-11-05T23:33-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(false, 1478489606, 1478502000, "2016-11-06T22:33-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(false, 1478399606, 1478412000, "2016-11-05T21:33-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(false, 1478486006, 1478498400, "2016-11-06T21:33-05:00", "2016-11-07T01:00-05:00");
  TryTestDST(false, 1478409968, 1478412000, "2016-11-06T00:26-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(false, 1478410268, 1478412300, "2016-11-06T00:31-04:00", "2016-11-06T01:05-05:00");
  TryTestDST(false, 1478413868, 1478415900, "2016-11-06T01:31-05:00", "2016-11-06T02:05-05:00");
  TryTestDST(false, 1478417468, 1478419500, "2016-11-06T02:31-05:00", "2016-11-06T03:05-05:00");
}

void TestIsRestricted() {

  TimeDomain td = TimeDomain(23622321788); // Mo-Fr 06:00-11:00
  TryIsRestricted(td, "2018-04-17T05:00", false);
  TryIsRestricted(td, "2018-04-17T06:00", true);
  TryIsRestricted(td, "2018-04-17T10:00", true);
  TryIsRestricted(td, "2018-04-17T11:00", true);
  TryIsRestricted(td, "2018-04-17T11:11", false);

  td = TimeDomain(40802435968); // Sa 03:30-19:00
  TryIsRestricted(td, "2018-04-17T11:11", false);
  TryIsRestricted(td, "2018-04-21T03:00", false);
  TryIsRestricted(td, "2018-04-21T11:11", true);

  td = TimeDomain(36507225218); // Sa-Su 12:00-17:00
  TryIsRestricted(td, "2018-04-27T13:00", false);
  TryIsRestricted(td, "2018-04-21T13:00", true);

  td = TimeDomain(39610337986940); // Apr-Sep Mo-Fr 09:00-13:00
  TryIsRestricted(td, "2018-04-27T13:00", true);
  TryIsRestricted(td, "2018-02-27T12:00", false);
  TryIsRestricted(td, "2018-09-30T13:00", false);
  TryIsRestricted(td, "2018-09-26T13:00", true);

  td = TimeDomain(1106007905274112); // Oct 16-Nov 15: 09:00-17:30
  TryIsRestricted(td, "2018-10-10T11:00", false);
  TryIsRestricted(td, "2018-10-16T11:00", true);

  td = TimeDomain(23622321664); // 06:00-11:00 - Everyday
  TryIsRestricted(td, "2018-10-10T11:00", true);
  TryIsRestricted(td, "2018-10-16T11:00", true);
  TryIsRestricted(td, "2018-10-16T05:59", false);
  TryIsRestricted(td, "2018-10-16T06:04", true);
  TryIsRestricted(td, "2018-10-16T11:01", false);

  // Oct Su[-1]-Mar th[4] Su 09:00-16:00
  // Every Sunday 9 to 16 starting from last Sunday in Oct to 4th thurs in Oct
  td = TimeDomain(9372272830712067);
  TryIsRestricted(td, "2018-10-21T09:01", false);
  TryIsRestricted(td, "2018-10-28T09:01", true);
  TryIsRestricted(td, "2019-01-20T16:00", true);
  TryIsRestricted(td, "2019-01-20T16:01", false);
  TryIsRestricted(td, "2019-03-28T11:00", false);
  TryIsRestricted(td, "2019-03-29T11:00", false);

  // Sun 09:00-16:00
  td = TimeDomain(34359740674);
  TryIsRestricted(td, "2018-10-21T09:01", true);
  TryIsRestricted(td, "2018-10-28T09:01", true);
  TryIsRestricted(td, "2018-10-29T09:01", false);
  TryIsRestricted(td, "2019-01-20T16:00", true);
  TryIsRestricted(td, "2019-01-20T16:01", false);
  TryIsRestricted(td, "2019-03-28T11:00", false);
  TryIsRestricted(td, "2019-03-29T11:00", false);

  // Su[1]
  td = TimeDomain(268435459);
  TryIsRestricted(td, "2018-10-07T09:01", true);
  TryIsRestricted(td, "2018-10-14T09:01", false);
  TryIsRestricted(td, "2018-11-04T09:01", true);
  TryIsRestricted(td, "2019-01-06T16:00", true);
  TryIsRestricted(td, "2019-01-13T16:01", false);

  // Dec
  td = TimeDomain(52776564424704);
  TryIsRestricted(td, "2018-12-12T16:01", true);
  TryIsRestricted(td, "2019-12-12T16:01", true);
  TryIsRestricted(td, "2019-11-12T16:01", false);
  TryIsRestricted(td, "2019-01-12T16:01", false);

  // Dec Su[-1] 15:00-17:00
  td = TimeDomain(11311813490642943);
  TryIsRestricted(td, "2018-12-30T15:01", true);
  TryIsRestricted(td, "2018-12-30T14:01", false);
  TryIsRestricted(td, "2018-12-23T14:01", false);
  TryIsRestricted(td, "2019-12-29T14:01", false);
  TryIsRestricted(td, "2019-12-29T15:01", true);

  // Nov 16-Feb 15: 09:00-16:30
  td = TimeDomain(1066423339714816);
  TryIsRestricted(td, "2018-11-15T09:01", false);
  TryIsRestricted(td, "2018-11-16T09:01", true);
  TryIsRestricted(td, "2019-02-16T09:01", false);
  TryIsRestricted(td, "2019-02-15T09:01", true);
  TryIsRestricted(td, "2019-02-15T16:31", false);
  TryIsRestricted(td, "2019-02-15T16:30", true);
  TryIsRestricted(td, "2019-07-04T16:30", false);

  // May 15 09:00-11:30
  td = TimeDomain(1079606730295552);
  TryIsRestricted(td, "2018-05-15T09:01", true);
  TryIsRestricted(td, "2018-05-15T12:00", false);

  // Sep-Jun We 08:15-08:45
  td = TimeDomain(29497840232464);
  TryIsRestricted(td, "2019-01-01T08:30", false);
  TryIsRestricted(td, "2019-01-02T08:30", true);
  TryIsRestricted(td, "2019-07-03T08:30", false);
  TryIsRestricted(td, "2019-06-30T09:30", false);
  TryIsRestricted(td, "2019-06-26T08:30", true);

  // Dec Su[-1]-Mar 3 Sat
  td = TimeDomain(224301728923777);
  TryIsRestricted(td, "2018-12-31T08:30", false);
  TryIsRestricted(td, "2019-01-05T08:30", true);
  TryIsRestricted(td, "2019-01-06T08:30", false);
  TryIsRestricted(td, "2019-03-02T08:30", true);
  TryIsRestricted(td, "2019-03-03T08:30", false);
}

void TestTimezoneDiff() {

  // dst tests
  TryTestTimezoneDiff(true, 1478493271, "2016-11-06T23:34-05:00", "2016-11-06T23:34-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1478410471, "2016-11-06T01:34-04:00", "2016-11-06T01:34-04:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1478419200, "2016-11-06T03:00-05:00", "2016-11-06T03:00-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1457847695, "2016-03-13T00:41-05:00", "2016-03-13T00:41-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1457848800, "2016-03-13T01:00-05:00", "2016-03-13T01:00-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1457851295, "2016-03-13T01:41-05:00", "2016-03-13T01:41-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(true, 1457852400, "2016-03-13T03:00-04:00", "2016-03-13T03:00-04:00",
                      "America/New_York", "America/New_York");

  // crossing tz
  // 2018-04-25T21:09-06:00 = 1524712192
  // if you are in mtn tz and go to la tz, subtract one hour
  TryTestTimezoneDiff(true, 1524712192, "2018-04-25T20:09-06:00", "2018-04-25T19:09-07:00",
                      "America/Denver", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  // if you are in la tz and go to mtn tz, add one hour
  TryTestTimezoneDiff(true, 1524712192, "2018-04-25T21:09-07:00", "2018-04-25T22:09-06:00",
                      "America/Los_Angeles", "America/Denver");

  // 2018-04-25T21:09-06:00 = 1524712192
  TryTestTimezoneDiff(false, 1524712192, "2018-04-25T20:09-06:00", "2018-04-25T19:09-07:00",
                      "America/Denver", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  // if you are in la tz and go to mtn tz, add one hour
  TryTestTimezoneDiff(false, 1524712192, "2018-04-25T21:09-07:00", "2018-04-25T22:09-06:00",
                      "America/Los_Angeles", "America/Denver");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(true, 1524712192, "2018-04-25T20:09-04:00", "2018-04-25T17:09-07:00",
                      "America/New_York", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  TryTestTimezoneDiff(true, 1524712192, "2018-04-25T23:09-07:00", "2018-04-26T02:09-04:00",
                      "America/Los_Angeles", "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(false, 1524712192, "2018-04-25T20:09-04:00", "2018-04-25T17:09-07:00",
                      "America/New_York", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  TryTestTimezoneDiff(false, 1524712192, "2018-04-25T23:09-07:00", "2018-04-26T02:09-04:00",
                      "America/Los_Angeles", "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(true, 1524712192, "2018-04-26T05:09-04:00", "2018-04-26T11:09+02:00",
                      "America/New_York", "Europe/Berlin");
  // 2018-04-26T05:09+02:00 = 1524712192
  TryTestTimezoneDiff(true, 1524712192, "2018-04-25T23:09+02:00", "2018-04-25T17:09-04:00",
                      "Europe/Berlin", "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(false, 1524712192, "2018-04-26T05:09-04:00", "2018-04-26T11:09+02:00",
                      "America/New_York", "Europe/Berlin");
  // 2018-04-26T05:09+02:00 = 1524712192
  TryTestTimezoneDiff(false, 1524712192, "2018-04-25T23:09+02:00", "2018-04-25T17:09-04:00",
                      "Europe/Berlin", "America/New_York");
}

void TestConditionalRestrictions() {

  std::string str = "Mo-Fr 06:00-11:00,17:00-19:00;Sa 03:30-19:00";
  std::vector<std::string> conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // Mo-Fr 06:00-11:00,17:00-19:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(23622321788);
      expected_values.push_back(40802193788);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), expected_values);

      TryConditionalRestrictions(conditions.at(x), 0, 0, 62, 0, 0, 0, 6, 0, 0, 0, 0, 11, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 62, 0, 0, 0, 17, 0, 0, 0, 0, 19, 0);

    } else if (x == 1) { // Sa 03:30-19:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(40802435968);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 0, 0, 0, 3, 30, 0, 0, 0, 19, 0);
    }
  }

  str = "Mo,We,Th,Fr 12:00-18:00; Sa-Su 12:00-17:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // Mo,We,Th,Fr 12:00-18:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(38654708852);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 58, 0, 0, 0, 12, 0, 0, 0, 0, 18, 0);

    } else if (x == 1) { // Sa-Su 12:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(36507225218);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 65, 0, 0, 0, 12, 0, 0, 0, 0, 17, 0);
    }
  }

  str = "July 23-Aug 21 Sa 14:00-20:00;JUL 23-jUl 28 Fr,PH 10:00-20:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // July 23-Aug 21 Sa 14:00-20:00;
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1512971146104448);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 7, 23, 0, 14, 0, 8, 21, 0, 20, 0);

    } else if (x == 1) { // JUL 23-jUl 28 Fr,PH 10:00-20:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2001154308835904);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 32, 7, 23, 0, 10, 0, 7, 28, 0, 20, 0);
    }
  }

  str = "Apr-Sep Mo-Fr 09:00-13:00,14:00-18:00; Apr-Sep Sa 10:00-13:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Apr-Sep Mo-Fr 09:00-13:00,14:00-18:00;

      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337986940);
      expected_values.push_back(39621075406460);
      TryConditionalRestrictions(conditions.at(x), expected_values);

    } else if (x == 1) { // Apr-Sep Sa 10:00-13:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337987200);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 4, 0, 0, 10, 0, 9, 0, 0, 13, 0);
    }
  }

  str = "Apr-Sep: Monday-Fr 09:00-13:00,14:00-18:00; ApRil-Sept: Sa 10:00-13:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Apr-Sep: Monday-Fr 09:00-13:00,14:00-18:00;

      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337986940);
      expected_values.push_back(39621075406460);
      TryConditionalRestrictions(conditions.at(x), expected_values);

    } else if (x == 1) { // ApRil-Sept: Sa 10:00-13:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337987200);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 4, 0, 0, 10, 0, 9, 0, 0, 13, 0);
    }
  }

  str = "06:00-11:00,17:00-19:45";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // 06:00-11:00,17:00-19:45

      std::vector<uint64_t> expected_values;
      expected_values.push_back(23622321664);
      expected_values.push_back(3133178646784);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 11, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0, 19, 45);
    }
  }

  str = " Feb 16-Oct 15 09:00-18:30; Oct 16-Nov 15: 09:00-17:30; Nov 16-Feb 15: 09:00-16:30";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Feb 16-Oct 15 09:00-18:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1101612002052352);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    } else if (x == 1) { // Oct 16-Nov 15: 09:00-17:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1106007905274112);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 10, 16, 0, 9, 0, 11, 15, 0, 17, 30);

    } else if (x == 2) { // Nov 16-Feb 15: 09:00-16:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1066423339714816);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 11, 16, 0, 9, 0, 02, 15, 0, 16, 30);
    }
  }

  str = "th 07:00-08:30; th-friday 06:00-09:30; May 15 09:00-11:30; May 07:00-08:30; May 16-31 "
        "11:00-13:30";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // th 07:00-08:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2078764173088);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 16, 0, 0, 0, 7, 0, 0, 0, 0, 8, 30);
    } else if (x == 1) { // th-friday 06:00-09:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2080911656544);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 48, 0, 0, 0, 6, 0, 0, 0, 0, 9, 30);
    } else if (x == 2) { // May 15 09:00-11:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1079606730295552);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 15, 0, 9, 0, 5, 15, 0, 11, 30);
    } else if (x == 3) { // May 07:00-08:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(24068999350016);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 0, 0, 7, 0, 5, 0, 0, 8, 30);
    } else if (x == 4) { // May 16-31 11:00-13:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2205510940494592);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 16, 0, 11, 0, 5, 31, 0, 13, 30);
    }
  }

  str = "(Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50;Sep-Jun We 08:15-08:45,11:55-12:35)";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(29497840232556);
      expected_values.push_back(29856470044524);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 54, 9, 0, 0, 8, 15, 6, 0, 0, 8, 45);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 54, 9, 0, 0, 15, 20, 6, 0, 0, 15, 50);
    } else if (x == 1) { // Sep-Jun We 08:15-08:45,11:55-12:35
      std::vector<uint64_t> expected_values;
      expected_values.push_back(29497840232464);
      expected_values.push_back(28819235728144);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 8, 9, 0, 0, 8, 15, 6, 0, 0, 8, 45);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 8, 9, 0, 0, 11, 55, 6, 0, 0, 12, 35);
    }
  }

  str = "Oct Su[-1]-Mar th[4] (Su 09:00-16:00; PH 09:00-16:00);Mar Su[-1]-Oct Su[-1] (Su "
        "09:00-18:00; PH 09:00-18:00)";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Oct Su[-1]-Mar th[4] (Su 09:00-16:00; PH 09:00-16:00)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(9372272830712067);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 10, 1, 5, 9, 0, 3, 5, 4, 16, 0);
    } else if (x == 1) { // PH 09:00-16:00  Holidays are tossed for now
      std::vector<uint64_t> expected_values;
      expected_values.push_back(0);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    } else if (x == 2) { // Mar Su[-1]-Oct Su[-1] (Su 09:00-18:00; PH 09:00-18:00)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11373388284561667);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 3, 1, 5, 9, 0, 10, 1, 5, 18, 0);
    } else if (x == 3) { // PH 09:00-18:00  Holidays are tossed for now
      std::vector<uint64_t> expected_values;
      expected_values.push_back(0);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    }
  }

  str = "Dec Fr[-1]-Jan Sa[3] Su,Sat 09:00-16:00, 15:00-17:00; Dec Su[-1] Su-Sa 15:00-17:00";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(7252414455351683);
      expected_values.push_back(7252416602836867);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 65, 12, 6, 5, 9, 0, 1, 7, 3, 16, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 1, 65, 12, 6, 5, 15, 0, 1, 7, 3, 17, 0);
    } else if (x == 1) { // Dec Su[-1] Su-Sa 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642943);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    }
  }

  str = "Sun 09:00-16:00; Su[1]; Dec; Dec Su[-1] 15:00-17:00; Dec Su[-1] Th 15:00-17:00;"
        "Dec Su[-1]; Dec Su[-1]-Mar 3 Sat;Mar 3-Dec Su[-1] Sat;Dec Su[-1]-Mar 3 Sat 15:00-17:00;"
        "Mar 3-Dec Su[-1] Sat 15:00-17:00";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sun 09:00-16:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(34359740674);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 1, 0, 0, 0, 9, 0, 0, 0, 0, 16, 0);
    } else if (x == 1) { // Su[1]
      std::vector<uint64_t> expected_values;
      expected_values.push_back(268435459);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    } else if (x == 2) { // Dec
      std::vector<uint64_t> expected_values;
      expected_values.push_back(52776564424704);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 12, 0, 0, 0, 0, 12, 0, 0, 0, 0);
    } else if (x == 3) { // Dec Su[-1] 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642943);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    } else if (x == 4) { // Dec Su[-1] Th 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642721);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 16, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    } else if (x == 5) { // Dec Su[-1]
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311776983417087);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 0, 0, 12, 0, 5, 0, 0);
    } else if (x == 6) { // Dec Su[-1]-Mar 3 Sat
      std::vector<uint64_t> expected_values;
      expected_values.push_back(224301728923777);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 12, 1, 5, 0, 0, 3, 3, 0, 0, 0);
    } else if (x == 7) { // Mar 3-Dec Su[-1] Sat
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382144397475969);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 0, 0, 12, 1, 5, 0, 0);
    } else if (x == 8) { // Dec Su[-1]-Mar 3 Sat 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(224338236149633);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 12, 1, 5, 15, 0, 3, 3, 0, 17, 0);
    } else if (x == 9) { // Mar 3-Dec Su[-1] Sat 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382180904701825);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 15, 0, 12, 1, 5, 17, 0);
    }
  }
}

void TestDayOfWeek() {
  std::string date = "2018-07-22T10:00";
  uint32_t dow = DateTime::day_of_week(date);
  if (dow != 0) {
    throw std::runtime_error("DateTime::day_of_week failed: 0 expected");
  }

  date = "2018-07-26T10:00";
  dow = DateTime::day_of_week(date);
  if (dow != 4) {
    throw std::runtime_error("DateTime::day_of_week failed: 4 expected");
  }
}

int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));
  suite.test(TEST_CASE(TestDOW));
  suite.test(TEST_CASE(TestDuration));
  suite.test(TEST_CASE(TestIsoDateTime));
  suite.test(TEST_CASE(TestServiceDays));
  suite.test(TEST_CASE(TestIsServiceAvailable));
  suite.test(TEST_CASE(TestIsValid));
  suite.test(TEST_CASE(TestDST));
  suite.test(TEST_CASE(TestConditionalRestrictions));
  suite.test(TEST_CASE(TestIsRestricted));
  suite.test(TEST_CASE(TestTimezoneDiff));
  suite.test(TEST_CASE(TestDayOfWeek));

  return suite.tear_down();
}
