#include "test.h"

#include <string>
#include <bitset>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryGetDaysFromPivotDate(std::string date_time, uint32_t expected_days) {
  if (DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time)) != expected_days) {
    throw std::runtime_error(
        std::string("Incorrect number of days from ")
    + date_time);
  }
}

void TryGetDOW(std::string date_time, uint32_t expected_dow) {

  if (DateTime::day_of_week_mask(date_time) != expected_dow) {
    throw std::runtime_error(
        std::string("Incorrect dow ") + date_time);
  }
}

void TryGetDuration(std::string date_time, uint32_t seconds, std::string expected_date_time) {

  if (DateTime::get_duration(date_time,seconds) != expected_date_time) {
    throw std::runtime_error(
        std::string("Incorrect duration ") + DateTime::get_duration(date_time,seconds) +
        std::string(" ") + expected_date_time);
  }
}

void TryGetSecondsFromMidnight(std::string date_time, uint32_t expected_seconds) {
  if (DateTime::seconds_from_midnight(date_time) != expected_seconds) {
    throw std::runtime_error(
        std::string("Incorrect number of seconds from ")
    + date_time);
  }
}

void TryIsoDateTime() {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string current_date_time = DateTime::iso_date_time(tz);
  std::string time;
  std::size_t found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time, tz) != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/Chicago"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time,tz) != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }

  tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("Africa/Porto-Novo"));
  current_date_time = DateTime::iso_date_time(tz);
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time,tz) != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }
}

void TryGetServiceDays(std::string begin_date, std::string end_date, uint32_t dow_mask, uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  uint64_t days = DateTime::get_service_days(b, e, DateTime::days_from_pivot_date(b), dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date + " " + std::to_string(days));
}

void TryIsServiceAvailable(std::string begin_date, std::string date, std::string end_date,uint64_t days, bool value) {

  auto b = DateTime::days_from_pivot_date(DateTime::get_formatted_date(begin_date));
  auto e = DateTime::days_from_pivot_date(DateTime::get_formatted_date(end_date));
  auto d = DateTime::days_from_pivot_date(DateTime::get_formatted_date(date));

  if (value != DateTime::is_service_available(days, b, d, e))
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date + " " + std::to_string(days));
}

void TryIsServiceDaysUsingShift(std::string begin_date, std::string date, std::string end_date,uint64_t days, bool value) {

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
    throw std::runtime_error("Invalid bits set for service days using shift.  " + begin_date + " " + end_date + " " + std::to_string(days));
}

void TryGetServiceDays(bool check_b_date, std::string begin_date, std::string date, std::string end_date, uint32_t dow_mask, uint64_t value) {

  std::string edate = end_date;
  std::string bdate = begin_date;
  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto tile_date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = DateTime::get_service_days(b, e, tile_date, dow_mask);

  if (check_b_date) {
    if (value != days && begin_date != date && end_date != edate)
      throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date + " " + std::to_string(days));
  } else {
    if (value != days && begin_date != bdate && end_date != date)
      throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date + " " + std::to_string(days));
  }
}

void TryRejectFeed(std::string begin_date, std::string end_date, uint32_t dow_mask, uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  auto tile_date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  uint64_t days = DateTime::get_service_days(b, e, tile_date, dow_mask);

  if (value != days)
    throw std::runtime_error("Feed should of been rejected. " + begin_date + " " + end_date + " " + std::to_string(days));
}

void TryAddServiceDays(uint64_t days, std::string begin_date, std::string end_date, std::string added_date, uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto a = DateTime::get_formatted_date(added_date);
  uint64_t result = DateTime::add_service_day(days, b, e, a);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + added_date);
}

void TryRemoveServiceDays(uint64_t days, std::string begin_date, std::string end_date, std::string removed_date, uint64_t value) {

  auto b = DateTime::get_formatted_date(begin_date);
  auto e = DateTime::get_formatted_date(end_date);
  auto r = DateTime::get_formatted_date(removed_date);
  uint64_t result = DateTime::remove_service_day(days, b, e, r);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + removed_date);
}

void TryTestServiceEndDate(std::string begin_date, std::string end_date, std::string new_end_date, uint32_t dow_mask) {

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

void TryTestDST(const bool is_depart_at, const uint64_t origin_seconds, const uint64_t dest_seconds, std::string o_value, std::string d_value) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  std::string iso_origin, iso_dest;
  DateTime::seconds_to_date(is_depart_at, origin_seconds, dest_seconds, tz, tz, iso_origin, iso_dest);

  if (iso_origin != o_value)
    throw std::runtime_error("Test origin DST failed.  Expected: " + o_value + " but received " + iso_origin);

  if (iso_dest != d_value)
    throw std::runtime_error("Test destination DST failed.  Expected: " + d_value + " but received " + iso_dest);
}

}

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

  TryGetDuration("20140101",30,"2014-01-01T00:00");
  TryGetDuration("20140102",60,"2014-01-02T00:01");
  TryGetDuration("2014-01-02",60,"2014-01-02T00:01");
  TryGetDuration("19990101",89, "");
  TryGetDuration("20140101T07:01",61,"2014-01-01T07:02");
  TryGetDuration("20140102T15:00",61,"2014-01-02T15:01");
  TryGetDuration("20140102T15:00",86400,"2014-01-03T15:00");

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

  //Test just the weekend for 4 days.
  //bits 2 and 3
  dow_mask |= kSaturday;
  dow_mask |= kSunday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 6);

  //Test just the weekend and Friday for 4 days.
  //bits 2 and 3
  dow_mask |= kFriday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 7);

  //Test just the weekend and Friday and Monday for 4 days.
  //bits 2 and 3
  dow_mask |= kMonday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 15);

  //Test just the weekend and Friday and Monday for 4 days.
  //Add Tuesday; however, result should be still 15 as we are only testing 4 days.
  //bits 2 and 3
  dow_mask |= kTuesday;
  TryGetServiceDays("2015-09-25", "2015-09-28", dow_mask, 15);

  //Test everyday for 60 days.
  dow_mask |= kWednesday;
  dow_mask |= kThursday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 1152921504606846975);

  //Test using a date far in the past.  Feed will be rejected.
  TryRejectFeed("2014-09-25", "2014-09-28", dow_mask, 0);
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));

  boost::gregorian::date today =  DateTime::get_formatted_date(
      DateTime::iso_date_time(tz));

  boost::gregorian::date startdate = today - boost::gregorian::days(30);
  boost::gregorian::date enddate = today + boost::gregorian::days(59);
  //Test getting the service days from today - 30 days.  Start date should change to today's date.
  TryGetServiceDays(true,to_iso_extended_string(startdate),to_iso_extended_string(today),
                    to_iso_extended_string(enddate), dow_mask, 1152921504606846975);

  startdate = today;
  enddate = today + boost::gregorian::days(100);
  //Test getting the service days from today.  end date should change to today's date + 59.
  TryGetServiceDays(false,to_iso_extended_string(startdate),to_iso_extended_string(today + boost::gregorian::days(59)),
                    to_iso_extended_string(enddate), dow_mask, 1152921504606846975);

  //Test weekends for 60 days.
  dow_mask = kDOWNone;
  dow_mask |= kSaturday;
  dow_mask |= kSunday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 435749860008887046);

  //Test weekends for 60 days plus Columbus Day
  TryAddServiceDays(435749860008887046,"2015-09-25", "2017-09-28", "2015-10-12", 435749860009018118);

  //Try to add a date out of the date range
  TryAddServiceDays(435749860008887046,"2015-09-25", "2017-09-28", "2018-10-12", 435749860008887046);

  //Test weekends for 60 days remove Columbus Day
  TryRemoveServiceDays(435749860009018118,"2015-09-25", "2017-09-28", "2015-10-12", 435749860008887046);

  //Try to remove a date out of the date range
  TryRemoveServiceDays(435749860009018118,"2015-09-25", "2017-09-28", "2018-10-12", 435749860009018118);

  //Test weekdays for 60 days.
  dow_mask = kDOWNone;
  dow_mask |= kMonday;
  dow_mask |= kTuesday;
  dow_mask |= kWednesday;
  dow_mask |= kThursday;
  dow_mask |= kFriday;
  TryGetServiceDays("2015-09-25", "2017-09-28", dow_mask, 717171644597959929);

  //Test to confirm that enddate is cut off at 60 days
  TryTestServiceEndDate("2015-09-25", "2017-09-28", "2015-11-23", dow_mask);

}

void TestIsServiceAvailable() {
  TryIsServiceAvailable("2015-11-11", "2016-01-09", "2016-01-09",580999813345182728, true);
  TryIsServiceAvailable("2015-11-11", "2016-01-10", "2016-01-09",580999813345182728, false);

  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-09", "2016-01-09",580999813345182728, true);
  TryIsServiceDaysUsingShift("2015-11-11", "2016-01-10", "2016-01-09",580999813345182728, false);

}

void TestIsValid(){
  TryTestIsValid("2015-05-06T01:00",true);
  TryTestIsValid("2015/05-06T01:00",false);
  TryTestIsValid("2015-05/06T01:00",false);
  TryTestIsValid("2015-05-06X01:00",false);
  TryTestIsValid("2015-05-06T01-00",false);
  TryTestIsValid("AAAa-05-06T01:00",false);
  TryTestIsValid("2015-05-06T24:00",false);

  TryTestIsValid("1983-02-30T24:01",false);
  TryTestIsValid("2015-13-06T24:01",false);
  TryTestIsValid("2015-05-06T24:60",false);
  TryTestIsValid("2015-05-06T26:02",false);
  TryTestIsValid("2015-05-06T23:59",true);
  TryTestIsValid("2015-05-06T-3:-9",false);

  TryTestIsValid("2015-05-06T01:0A",false);
  TryTestIsValid("2015-05-06T01",false);
  TryTestIsValid("01:00",false);
  TryTestIsValid("aefopijafepij",false);
}

void TestDST(){

  //bunch of tests for the start and end of dst using startat and arriveby

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

  TryTestDST(false,1457847695, 1457848800, "2016-03-13T00:41-05:00", "2016-03-13T01:00-05:00");
  TryTestDST(false,1457851295, 1457852400, "2016-03-13T01:41-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(false,1457853095, 1457854200, "2016-03-13T03:11-04:00", "2016-03-13T03:30-04:00");
  TryTestDST(false,1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false,1457851595, 1457852700, "2016-03-13T01:46-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false,1457846553, 1457852400, "2016-03-13T00:22-05:00", "2016-03-13T03:00-04:00");
  TryTestDST(false,1457847453, 1457852700, "2016-03-13T00:37-05:00", "2016-03-13T03:05-04:00");
  TryTestDST(false,1457933853, 1457939100, "2016-03-14T01:37-04:00", "2016-03-14T03:05-04:00");
  TryTestDST(false,1457848559, 1457856300, "2016-03-13T00:55-05:00", "2016-03-13T04:05-04:00");
  TryTestDST(false,1457847978, 1457856000, "2016-03-13T00:46-05:00", "2016-03-13T04:00-04:00");
  TryTestDST(false,1457934378, 1457942400, "2016-03-14T01:46-04:00", "2016-03-14T04:00-04:00");

  TryTestDST(false,1478406871, 1478415600, "2016-11-06T00:34-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(false,1478493271, 1478502000, "2016-11-06T23:34-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(false,1478410471, 1478419200, "2016-11-06T01:34-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(false,1478496871, 1478505600, "2016-11-07T00:34-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(false,1478414071, 1478422800, "2016-11-06T01:34-05:00", "2016-11-06T04:00-05:00");
  TryTestDST(false,1478500471, 1478509200, "2016-11-07T01:34-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(false,1478410406, 1478422800, "2016-11-06T01:33-04:00", "2016-11-06T04:00-05:00");
  TryTestDST(false,1478496806, 1478509200, "2016-11-07T00:33-05:00", "2016-11-07T04:00-05:00");
  TryTestDST(false,1478406806, 1478419200, "2016-11-06T00:33-04:00", "2016-11-06T03:00-05:00");
  TryTestDST(false,1478493206, 1478505600, "2016-11-06T23:33-05:00", "2016-11-07T03:00-05:00");
  TryTestDST(false,1478403206, 1478415600, "2016-11-05T23:33-04:00", "2016-11-06T02:00-05:00");
  TryTestDST(false,1478489606, 1478502000, "2016-11-06T22:33-05:00", "2016-11-07T02:00-05:00");
  TryTestDST(false,1478399606, 1478412000, "2016-11-05T21:33-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(false,1478486006, 1478498400, "2016-11-06T21:33-05:00", "2016-11-07T01:00-05:00");
  TryTestDST(false,1478409968, 1478412000, "2016-11-06T00:26-04:00", "2016-11-06T01:00-05:00");
  TryTestDST(false,1478410268, 1478412300, "2016-11-06T00:31-04:00", "2016-11-06T01:05-05:00");
  TryTestDST(false,1478413868, 1478415900, "2016-11-06T01:31-05:00", "2016-11-06T02:05-05:00");
  TryTestDST(false,1478417468, 1478419500, "2016-11-06T02:31-05:00", "2016-11-06T03:05-05:00");

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

  return suite.tear_down();
}
