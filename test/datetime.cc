#include "test.h"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>

#include <boost/algorithm/string/split.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "midgard/constants.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Get the iso date and time from a DOW mask and time.
std::string test_iso_date_time(const uint8_t dow_mask,
                               const std::string& time,
                               const date::time_zone* time_zone) {

  std::string iso_date_time;
  std::stringstream ss("");
  if (time.empty() || time.find(':') == std::string::npos || !time_zone) {
    return iso_date_time;
  }

  uint8_t dow;
  switch (dow_mask) {
    case kSunday:
      dow = 0;
      break;
    case kMonday:
      dow = 1;
      break;
    case kTuesday:
      dow = 2;
      break;
    case kWednesday:
      dow = 3;
      break;
    case kThursday:
      dow = 4;
      break;
    case kFriday:
      dow = 5;
      break;
    case kSaturday:
      dow = 6;
      break;
    default:
      return iso_date_time;
      break;
  }

  auto now = date::make_zoned(time_zone, std::chrono::system_clock::now());
  auto date = date::floor<date::days>(now.get_local_time());
  auto d = date::year_month_day(date);
  auto t = date::make_time(now.get_local_time() - date);      // Yields time_of_day type
  std::chrono::minutes current_tod = t.hours() + t.minutes(); // Yields time_of_day type
  std::chrono::minutes desired_tod;

  std::size_t found = time.find(':'); // HH:MM
  if (found != std::string::npos) {
    std::chrono::hours h = std::chrono::hours(std::stoi(time.substr(0, 2)));
    std::chrono::minutes m = std::chrono::minutes(std::stoi(time.substr(3, 2)));
    desired_tod = h + m;
  } else
    return "";

  // will today work?
  if ((date::weekday(date) - date::Sunday).count() == dow) {
    // is the desired time in the past?
    if (desired_tod < current_tod) {
      now = now.get_local_time() + date::days(7);
    }
  } else {
    while ((date::weekday(date) - date::Sunday).count() == dow) {
      now = now.get_local_time() + date::days(1);
      date = date::floor<date::days>(now.get_local_time());
    }
  }

  ss << date::format("%F", now);
  iso_date_time = ss.str();
  iso_date_time += "T" + time;
  return iso_date_time;
}

void TryGetDaysFromPivotDate(const std::string& date_time, uint32_t expected_days) {
  if (DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time)) != expected_days) {
    throw std::runtime_error(
        std::string("Incorrect number of days from ") + date_time + " " +
        std::to_string(DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time))));
  }
}

void TryGetDOW(const std::string& date_time, uint32_t expected_dow) {

  if (DateTime::day_of_week_mask(date_time) != expected_dow) {
    throw std::runtime_error(std::string("Incorrect dow ") + date_time);
  }
}

void TryGetDuration(const std::string& date_time,
                    uint32_t seconds,
                    const std::string& expected_date_time) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  if (DateTime::get_duration(date_time, seconds, tz) != expected_date_time) {
    throw std::runtime_error(std::string("Incorrect duration ") +
                             DateTime::get_duration(date_time, seconds, tz) + std::string(" ") +
                             expected_date_time);
  }
}

void TryGetSecondsFromMidnight(const std::string& date_time, uint32_t expected_seconds) {
  auto secs = DateTime::seconds_from_midnight(date_time);
  if (secs != expected_seconds) {
    throw std::runtime_error(std::string("Incorrect number of seconds from ") + date_time +
                             " got: " + std::to_string(secs));
  }
}

void TryIsoDateTime() {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string current_date_time = DateTime::iso_date_time(tz);
  std::string time;
  std::size_t found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (test_iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/Chicago"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (test_iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("Africa/Porto-Novo"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found + 1);

  if (test_iso_date_time(DateTime::day_of_week_mask(current_date_time), time, tz) !=
      current_date_time) {
    throw std::runtime_error(std::string("Iso date time failed ") + current_date_time);
  }
}

void TryTestIsValid(const std::string& date, bool return_value) {

  auto ret = DateTime::is_iso_valid(date);
  if (ret != return_value)
    throw std::runtime_error("Test is_iso_valid failed: " + date +
                             " locale = " + std::locale("").name());
}

void TryTestDST(const uint64_t origin_seconds,
                const uint64_t dest_seconds,
                const std::string& o_value,
                const std::string& d_value) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string iso_origin, iso_dest;
  DateTime::seconds_to_date(origin_seconds, dest_seconds, tz, tz, iso_origin, iso_dest);

  if (iso_origin != o_value)
    throw std::runtime_error("Test origin DST failed.  Expected: " + o_value + " but received " +
                             iso_origin);

  if (iso_dest != d_value)
    throw std::runtime_error("Test destination DST failed.  Expected: " + d_value + " but received " +
                             iso_dest);
}

void TryIsRestricted(const TimeDomain td, const std::string& date, const bool expected_value) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  if (DateTime::is_restricted(td.type(), td.begin_hrs(), td.begin_mins(), td.end_hrs(), td.end_mins(),
                              td.dow(), td.begin_week(), td.begin_month(), td.begin_day_dow(),
                              td.end_week(), td.end_month(), td.end_day_dow(),
                              DateTime::seconds_since_epoch(date, tz), tz) != expected_value) {

    throw std::runtime_error("Is Restricted " + date +
                             " test failed.  Expected: " + std::to_string(expected_value));
  }
}

void TryTestTimezoneDiff(const uint64_t date_time,
                         const std::string& expected1,
                         const std::string& expected2,
                         const std::string& time_zone1,
                         const std::string& time_zone2) {

  uint64_t dt = date_time;

  auto tz1 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone1));
  auto tz2 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone2));

  dt += DateTime::timezone_diff(dt, tz1, tz2);
  if (DateTime::seconds_to_date(dt, tz1) != expected1)
    throw std::runtime_error("Timezone Diff test #1: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected1 + " but got " +
                             DateTime::seconds_to_date(dt, tz1));

  if (DateTime::seconds_to_date(dt, tz2) != expected2)
    throw std::runtime_error("Timezone Diff test #2: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected2 + " but got " +
                             DateTime::seconds_to_date(dt, tz2));
}

} // namespace

void TestGetDaysFromPivotDate() {
  TryGetDaysFromPivotDate("2014-01-01T07:01", 0);
  TryGetDaysFromPivotDate("2014-01-02T15:00", 1);
  TryGetDaysFromPivotDate("1999-01-01-T:00:00", 0);
  TryGetDaysFromPivotDate("2015-05-06T08:00", 490);
}

void TestDOW() {

  TryGetDOW("2014-01-01T07:01", kWednesday);
  TryGetDOW("2014-01-02T15:00", kThursday);
  TryGetDOW("1999-01-01T:00:00", kDOWNone);
  TryGetDOW("2015-05-09T08:00", kSaturday);
}

void TestDuration() {

  TryGetDuration("2014-01-01T00:00", 30, "2014-01-01T00:00-05:00 EST");
  TryGetDuration("2014-01-02T00:00", 60, "2014-01-02T00:01-05:00 EST");
  TryGetDuration("1999-01-01T00:00", 89, "");
  TryGetDuration("2014-01-01T07:01", 61, "2014-01-01T07:02-05:00 EST");
  TryGetDuration("2014-01-02T15:00", 61, "2014-01-02T15:01-05:00 EST");
  TryGetDuration("2014-01-02T15:00", 86400, "2014-01-03T15:00-05:00 EST");
  TryGetDuration("2016-07-14T00:00", 60, "2016-07-14T00:01-04:00 EDT");
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
  TryTestIsValid("2015-05-06T25:59", false);
  TryTestIsValid("2015-05-06T23:60", false);
  TryTestIsValid("2015-05-06T26:02", false);
  TryTestIsValid("2015-05-06T23:59", true);
  TryTestIsValid("2015-05-06T-3:-9", false);

  // TODO: This test fails on OSX - says it is a valid ISO time?
  //  TryTestIsValid("2015-05-06T01:0A", false);

  TryTestIsValid("2015-05-06T01", false);
  TryTestIsValid("01:00", false);
  TryTestIsValid("aefopijafepij", false);

  TryTestIsValid("ABCDEFG", false);
  TryTestIsValid("2018--26T10:00", false);
  TryTestIsValid("11800--26T10:00", false);
  TryTestIsValid("2018-25-26T10:00", false);
  TryTestIsValid("2018:25:26T10:00", false);
  TryTestIsValid("2018-1-26M10:00", false);
  TryTestIsValid("2018-1-26T1000:00", false);
  TryTestIsValid("2018-1-26T:01:1000", false);
  TryTestIsValid("2018-1-26T:00", false);
  TryTestIsValid("2018-07-22T10:89", false);
}

void TestDST() {

  // bunch of tests for the start and end of dst using startat and arriveby

  TryTestDST(1457845200, 1457858104, "2016-03-13T00:00-05:00", "2016-03-13T04:35-04:00");
  TryTestDST(1457848800, 1457860508, "2016-03-13T01:00-05:00", "2016-03-13T05:15-04:00");
  TryTestDST(1457852100, 1457853188, "2016-03-13T01:55-05:00", "2016-03-13T03:13-04:00");
  TryTestDST(1457852400, 1457853488, "2016-03-13T03:00-04:00", "2016-03-13T03:18-04:00");
  TryTestDST(1457854200, 1457859493, "2016-03-13T03:30-04:00", "2016-03-13T04:58-04:00");
  TryTestDST(1457855700, 1457856788, "2016-03-13T03:55-04:00", "2016-03-13T04:13-04:00");
  TryTestDST(1457852400, 1457853488, "2016-03-13T03:00-04:00", "2016-03-13T03:18-04:00");
  TryTestDST(1457853300, 1457854388, "2016-03-13T03:15-04:00", "2016-03-13T03:33-04:00");

  TryTestDST(1478406600, 1478407484, "2016-11-06T00:30-04:00", "2016-11-06T00:44-04:00");
  TryTestDST(1478408340, 1478409215, "2016-11-06T00:59-04:00", "2016-11-06T01:13-04:00");
  TryTestDST(1478408340, 1478413633, "2016-11-06T00:59-04:00", "2016-11-06T01:27-05:00");
  TryTestDST(1478413800, 1478414684, "2016-11-06T01:30-05:00", "2016-11-06T01:44-05:00");
  TryTestDST(1478413800, 1478418104, "2016-11-06T01:30-05:00", "2016-11-06T02:41-05:00");
  TryTestDST(1478415600, 1478420893, "2016-11-06T02:00-05:00", "2016-11-06T03:28-05:00");
  TryTestDST(1478412000, 1478417293, "2016-11-06T01:00-05:00", "2016-11-06T02:28-05:00");
  TryTestDST(1478419200, 1478424493, "2016-11-06T03:00-05:00", "2016-11-06T04:28-05:00");
  TryTestDST(1478408340, 1478420602, "2016-11-06T00:59-04:00", "2016-11-06T03:23-05:00");
  TryTestDST(1479016740, 1479029002, "2016-11-13T00:59-05:00", "2016-11-13T04:23-05:00");

  TryTestDST(1457847695, 1457848800, "2016-03-13T00:41-05:00", "2016-03-13T01:00-05:00");
  TryTestDST(1457851295, 1457852400, "2016-03-13T01:41-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(1457853095, 1457854200, "2016-03-13T03:11-04:00", "2016-03-13T03:30-04:00");
  TryTestDST(1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(1457846553, 1457852400, "2016-03-13T00:22-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(1457847453, 1457852700, "2016-03-13T00:37-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(1457933853, 1457939100, "2016-03-14T01:37-04:00", "2016-03-14T03:05-04:00");
  TryTestDST(1457848559, 1457856300, "2016-03-13T00:55-05:00", "2016-03-13T04:05-04:00");
  TryTestDST(1457847978, 1457856000, "2016-03-13T00:46-05:00", "2016-03-13T04:00-04:00");
  TryTestDST(1457934378, 1457942400, "2016-03-14T01:46-04:00", "2016-03-14T04:00-04:00");

  TryTestDST(1478406871, 1478415600, "2016-11-06T00:34-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(1478493271, 1478502000, "2016-11-06T23:34-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(1478410471, 1478419200, "2016-11-06T01:34-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(1478496871, 1478505600, "2016-11-07T00:34-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(1478414071, 1478422800, "2016-11-06T01:34-05:00", "2016-11-06T04:00-05:00");
  TryTestDST(1478500471, 1478509200, "2016-11-07T01:34-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(1478410406, 1478422800, "2016-11-06T01:33-04:00", "2016-11-06T04:00-05:00");
  TryTestDST(1478496806, 1478509200, "2016-11-07T00:33-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(1478406806, 1478419200, "2016-11-06T00:33-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(1478493206, 1478505600, "2016-11-06T23:33-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(1478403206, 1478415600, "2016-11-05T23:33-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(1478489606, 1478502000, "2016-11-06T22:33-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(1478399606, 1478412000, "2016-11-05T22:33-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(1478486006, 1478498400, "2016-11-06T21:33-05:00", "2016-11-07T01:00-05:00");
  TryTestDST(1478409968, 1478412000, "2016-11-06T01:26-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(1478410268, 1478412300, "2016-11-06T01:31-04:00", "2016-11-06T01:05-05:00");
  TryTestDST(1478413868, 1478415900, "2016-11-06T01:31-05:00", "2016-11-06T02:05-05:00");
  TryTestDST(1478417468, 1478419500, "2016-11-06T02:31-05:00", "2016-11-06T03:05-05:00");
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
  TryTestTimezoneDiff(1478493271, "2016-11-06T23:34-05:00", "2016-11-06T23:34-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1478410471, "2016-11-06T01:34-04:00", "2016-11-06T01:34-04:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1478419200, "2016-11-06T03:00-05:00", "2016-11-06T03:00-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1457847695, "2016-03-13T00:41-05:00", "2016-03-13T00:41-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1457848800, "2016-03-13T01:00-05:00", "2016-03-13T01:00-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1457851295, "2016-03-13T01:41-05:00", "2016-03-13T01:41-05:00",
                      "America/New_York", "America/New_York");
  TryTestTimezoneDiff(1457852400, "2016-03-13T03:00-04:00", "2016-03-13T03:00-04:00",
                      "America/New_York", "America/New_York");

  // crossing tz
  // 2018-04-25T21:09-06:00 = 1524712192
  // if you are in mtn tz and go to la tz, subtract one hour
  TryTestTimezoneDiff(1524712192, "2018-04-25T20:09-06:00", "2018-04-25T19:09-07:00",
                      "America/Denver", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  // if you are in la tz and go to mtn tz, add one hour
  TryTestTimezoneDiff(1524712192, "2018-04-25T21:09-07:00", "2018-04-25T22:09-06:00",
                      "America/Los_Angeles", "America/Denver");

  // 2018-04-25T21:09-06:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T20:09-06:00", "2018-04-25T19:09-07:00",
                      "America/Denver", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  // if you are in la tz and go to mtn tz, add one hour
  TryTestTimezoneDiff(1524712192, "2018-04-25T21:09-07:00", "2018-04-25T22:09-06:00",
                      "America/Los_Angeles", "America/Denver");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T20:09-04:00", "2018-04-25T17:09-07:00",
                      "America/New_York", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T23:09-07:00", "2018-04-26T02:09-04:00",
                      "America/Los_Angeles", "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T20:09-04:00", "2018-04-25T17:09-07:00",
                      "America/New_York", "America/Los_Angeles");
  // 2018-04-25T20:09-07:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T23:09-07:00", "2018-04-26T02:09-04:00",
                      "America/Los_Angeles", "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-26T05:09-04:00", "2018-04-26T11:09+02:00",
                      "America/New_York", "Europe/Berlin");
  // 2018-04-26T05:09+02:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T23:09+02:00", "2018-04-25T17:09-04:00", "Europe/Berlin",
                      "America/New_York");

  // 2018-04-25T23:09-04:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-26T05:09-04:00", "2018-04-26T11:09+02:00",
                      "America/New_York", "Europe/Berlin");
  // 2018-04-26T05:09+02:00 = 1524712192
  TryTestTimezoneDiff(1524712192, "2018-04-25T23:09+02:00", "2018-04-25T17:09-04:00", "Europe/Berlin",
                      "America/New_York");
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

  date = "2019-11-06T17:15";
  dow = DateTime::day_of_week(date);
  if (dow != 3) {
    throw std::runtime_error("DateTime::day_of_week failed: 3 expected");
  }
}

void TestSecondOfWeek() {
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  // 2019-11-06T17:15
  auto a = DateTime::second_of_week(1573078500, tz);
  auto e = 3 * valhalla::midgard::kSecondsPerDay + 17 * valhalla::midgard::kSecondsPerHour +
           15 * valhalla::midgard::kSecondsPerMinute;
  if (a != e)
    throw std::logic_error("Wrong second of week");

  // 1982-12-08T06:07:42
  a = DateTime::second_of_week(408193662, tz);
  e = 3 * valhalla::midgard::kSecondsPerDay + 6 * valhalla::midgard::kSecondsPerHour +
      7 * valhalla::midgard::kSecondsPerMinute + 42;
  if (a != e)
    throw std::logic_error("Wrong second of week");

  // 2077-02-14T11:11:11
  a = DateTime::second_of_week(3380544671, tz);
  e = 0 * valhalla::midgard::kSecondsPerDay + 11 * valhalla::midgard::kSecondsPerHour +
      11 * valhalla::midgard::kSecondsPerMinute + 11;
  if (a != e)
    throw std::logic_error("Wrong second of week");
}

int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));
  suite.test(TEST_CASE(TestDOW));
  suite.test(TEST_CASE(TestDuration));
  suite.test(TEST_CASE(TestIsoDateTime));
  suite.test(TEST_CASE(TestIsValid));
  suite.test(TEST_CASE(TestIsRestricted));
  suite.test(TEST_CASE(TestDST));
  suite.test(TEST_CASE(TestTimezoneDiff));
  suite.test(TEST_CASE(TestDayOfWeek));
  suite.test(TEST_CASE(TestSecondOfWeek));

  return suite.tear_down();
}
