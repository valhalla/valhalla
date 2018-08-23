#include "test.h"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>

#include <boost/algorithm/string/split.hpp>

#include "baldr/datetime2.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"

using namespace std;
using namespace valhalla::baldr;

namespace {
/*
// Get the iso date and time from a DOW mask and time.
std::string test_iso_date_time(const uint8_t dow_mask,
                               const std::string& time,
                               const boost::local_time::time_zone_ptr& time_zone) {

  std::string iso_date_time;
  std::stringstream ss("");
  if (time.empty() || time.find(':') == std::string::npos || !time_zone) {
    return iso_date_time;
  }

  uint8_t dow;
  switch (dow_mask) {
    case kSunday:
      dow = boost::date_time::Sunday;
      break;
    case kMonday:
      dow = boost::date_time::Monday;
      break;
    case kTuesday:
      dow = boost::date_time::Tuesday;
      break;
    case kWednesday:
      dow = boost::date_time::Wednesday;
      break;
    case kThursday:
      dow = boost::date_time::Thursday;
      break;
    case kFriday:
      dow = boost::date_time::Friday;
      break;
    case kSaturday:
      dow = boost::date_time::Saturday;
      break;
    default:
      return iso_date_time;
      break;
  }

  try {
    boost::local_time::local_time_input_facet* input_facet =
        new boost::local_time::local_time_input_facet();
    input_facet->format("%H:%M");
    ss.imbue(std::locale(ss.getloc(), input_facet));

    boost::local_time::local_date_time desired_time(boost::local_time::not_a_date_time);
    ss.str(time);
    ss >> desired_time;

    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    boost::local_time::local_date_time local_date_time(pt, time_zone);

    pt = local_date_time.local_time();
    boost::gregorian::date date = pt.date();
    uint8_t desired_tod =
        (3600 * desired_time.time_of_day().hours()) + (60 * desired_time.time_of_day().minutes());
    uint8_t current_tod = (3600 * pt.time_of_day().hours()) + (60 * pt.time_of_day().minutes());

    // will today work?
    if (date.day_of_week().as_enum() == dow) {
      // is the desired time in the past?
      if (desired_tod < current_tod) {
        date += boost::gregorian::days(7);
      }
    } else {
      while (date.day_of_week().as_enum() != dow) {
        date += boost::gregorian::days(1);
      }
    }
    iso_date_time = to_iso_extended_string(date) + "T" + time;
  } catch (std::exception& e) {}
  return iso_date_time;
}

std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value, std::bind1st(std::equal_to<char>(), delim),
                          boost::algorithm::token_compress_on);
  return tokens;
}
*/
void TryTestTimeZoneAtt() {

  //2016-03-13T01:00
/*
  auto& db = DateTime::get_tz_db().db;

  //auto ny = date::make_zoned("America/New_York", date::local_days{date::mar/13/2016} + std::chrono::hours{2});


  auto utc = date::sys_days{date::mar/13/2016} + std::chrono::hours{7};
  auto ny = date::make_zoned("America/New_York", utc);
  std::cout << date::format("%F %T %Z\n", ny);

  uint32_t index = DateTime::get_tz_db().to_index("America/New_York");
  auto test = date::make_zoned(DateTime::get_tz_db().from_index(index),
                               date::local_days{date::mar/13/2016} + std::chrono::hours{2} + std::chrono::minutes{2},date::choose::latest);
  std::cout << date::format("%F %T %Z\n", test);

  std::istringstream in{"2018-10-16T05:59"};
  date::sys_seconds tp;
  in >> date::parse("%FT%R", tp);
  using date::operator<<;
  std::cout << tp << '\n';
*/

  /*
  for (auto const& z : db.zones)
       {
           auto begin = date::sys_days{date::jan/1/date::year::min()} + std::chrono::seconds{0};
           auto end   = date::sys_days{date::jan/1/2035} + std::chrono::seconds{0};
           do
           {
               auto info = z.get_info(begin);
               if (info.save != std::chrono::hours{0} && info.save != std::chrono::hours{1})
               {
                   std::cout << z.name() << " has a daylight savings offset of "
                             << info.save.count() << "min from " //<< //info.begin
                             << " UTC to " //<< //info.end << " UTC with the abbreviation "
                             << info.abbrev << '\n';
               }
               begin = info.end;
           } while (begin < end);
       }
       */

}

void TryGetDaysFromPivotDate(const std::string& date_time, uint32_t expected_days) {
  if (DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time)) != expected_days) {
    throw std::runtime_error(std::string("Incorrect number of days from ") + date_time + " " + std::to_string(DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time))));
  }
}
/*
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

void TryTestDST(const bool is_depart_at,
                const uint64_t origin_seconds,
                const uint64_t dest_seconds,
                const std::string& o_value,
                const std::string& d_value) {

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

// Convert seconds to a date string (for test evaluation only). DateTime holds a more general
// method.
std::string seconds_to_date(const uint64_t seconds, const boost::local_time::time_zone_ptr& tz) {

  std::string iso_date;
  if (seconds == 0 || !tz) {
    return iso_date;
  }

  try {
    std::string tz_string;
    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime pt = time_epoch + boost::posix_time::seconds(seconds);
    boost::local_time::local_date_time date_time(pt, tz);
    pt = date_time.local_time();

    boost::gregorian::date date = pt.date();
    std::stringstream ss_time;
    ss_time << pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      time = time.substr(0, found);
    }

    ss_time.str("");
    if (date_time.is_dst()) {
      ss_time << tz->dst_offset() + tz->base_utc_offset();
    } else {
      ss_time << tz->base_utc_offset();
    }

    // positive tz
    if (ss_time.str().find('+') == std::string::npos &&
        ss_time.str().find('-') == std::string::npos) {
      iso_date = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    } else {
      iso_date = to_iso_extended_string(date) + "T" + time + ss_time.str();
    }

    found = iso_date.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      iso_date = iso_date.substr(0, found);
    }

  } catch (std::exception& e) {}
  return iso_date;
}

void TryTestTimezoneDiff(const bool is_depart,
                         const uint64_t date_time,
                         const std::string& expected1,
                         const std::string& expected2,
                         const std::string& time_zone1,
                         const std::string& time_zone2) {

  uint64_t dt = date_time;

  auto tz1 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone1));
  auto tz2 = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index(time_zone2));

  std::cout << seconds_to_date(dt, tz1) << std::endl;

  dt += DateTime::timezone_diff(is_depart, dt, tz1, tz2);
  if (seconds_to_date(dt, tz1) != expected1)
    throw std::runtime_error("Timezone Diff test #1: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected1 + " but got " +
                             seconds_to_date(dt, tz1));

  if (seconds_to_date(dt, tz2) != expected2)
    throw std::runtime_error("Timezone Diff test #2: " + std::to_string(date_time) +
                             " test failed.  Expected: " + expected2 + " but got " +
                             seconds_to_date(dt, tz2));
}
*/
} // namespace

void TestTimeZoneAtt() {
  TryTestTimeZoneAtt();
}

void TestGetDaysFromPivotDate() {
  TryGetDaysFromPivotDate("2014-01-01T07:01", 0);
  TryGetDaysFromPivotDate("2014-01-02T15:00", 1);
  TryGetDaysFromPivotDate("1999-01-01-T:00:00", 0);
  TryGetDaysFromPivotDate("2015-05-06T08:00", 490);
}
/*
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

void TestISOToTm() {
  std::string date = "2018-07-22T10:09";
  std::tm t = DateTime::iso_to_tm(date);
  if (t.tm_year != 118) {
    throw std::runtime_error("DateTime::iso_to_tm year: 118 expected");
  }
  // Remember, tm_mon is 0 based
  if (t.tm_mon != 6) {
    throw std::runtime_error("DateTime::iso_to_tm month: 6 expected, got: " +
                             std::to_string(t.tm_mon));
  }
  if (t.tm_mday != 22) {
    throw std::runtime_error("DateTime::iso_to_tm month: 22 expected");
  }
  if (t.tm_hour != 10) {
    throw std::runtime_error("DateTime::iso_to_tm hour: 10 expected, got: " +
                             std::to_string(t.tm_hour));
  }
  if (t.tm_min != 9) {
    throw std::runtime_error("DateTime::iso_to_tm min: 9 expected");
  }
}
*/
int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  /*
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));
  suite.test(TEST_CASE(TestDOW));
  suite.test(TEST_CASE(TestDuration));
  suite.test(TEST_CASE(TestIsoDateTime));
  suite.test(TEST_CASE(TestIsValid));
  suite.test(TEST_CASE(TestIsRestricted));
  suite.test(TEST_CASE(TestDST));
  suite.test(TEST_CASE(TestTimezoneDiff));
  suite.test(TEST_CASE(TestDayOfWeek));

  suite.test(TEST_CASE(TestISOToTm));*/
  suite.test(TEST_CASE(TestTimeZoneAtt));

  return suite.tear_down();
}
