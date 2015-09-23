#include "test.h"

#include <string>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryGetDaysFromPivotDate(std::string date_time, uint32_t expected_days) {
  if (DateTime::days_from_pivot_date(date_time) != expected_days) {
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

void TryGetTime(std::string date_time, std::string expected_date_time) {
  if (DateTime::time(date_time) != expected_date_time) {
    throw std::runtime_error(
        std::string("Incorrect Time ") + DateTime::time(date_time));
  }
}

void TryGetDate(std::string date_time, std::string expected_date_time) {
  if (DateTime::date(date_time) != expected_date_time) {
    throw std::runtime_error(
        std::string("Incorrect Date ") + DateTime::time(date_time));
  }
}

void TryIsoDateTime() {

  std::string current_date_time = DateTime::iso_date_time();
  std::string time;
  std::size_t found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time) != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }

  current_date_time = DateTime::iso_date_time("America/Chicago");
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time,"America/Chicago") != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }

  current_date_time = DateTime::iso_date_time("Africa/Porto-Novo");
  found = current_date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    time = current_date_time.substr(found+1);

  if (DateTime::iso_date_time(DateTime::day_of_week_mask(current_date_time),time,"Africa/Porto-Novo") != current_date_time) {
    throw std::runtime_error(
        std::string("Iso date time failed ") + current_date_time);
  }

}

void TryGetServiceDays(std::string begin_date, std::string end_date, uint32_t dow_mask, uint64_t value) {

  uint64_t days = DateTime::get_service_days(begin_date, end_date, dow_mask);

  if (value != days)
    throw std::runtime_error("Invalid bits set for service days. " + begin_date + " " + end_date + " " + std::to_string(days));
}

void TryAddServiceDays(uint64_t days, std::string begin_date, std::string end_date, std::string added_date, uint64_t value) {

  uint64_t result = DateTime::add_service_day(days, begin_date, end_date, added_date);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + added_date);
}

void TryRemoveServiceDays(uint64_t days, std::string begin_date, std::string end_date, std::string removed_date, uint64_t value) {

  uint64_t result = DateTime::remove_service_day(days, begin_date, end_date, removed_date);
  if (value != result)
    throw std::runtime_error("Invalid bits set for added service day. " + removed_date);
}

void TryTestServiceEndDate(std::string begin_date, std::string end_date, std::string new_end_date, uint32_t dow_mask) {

  DateTime::get_service_days(begin_date, end_date, dow_mask);

  if (end_date != new_end_date)
    throw std::runtime_error("End date not cut off at 60 days.");

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

void TestTime() {

  TryGetTime("20140101","");
  TryGetTime("Blah","");
  TryGetTime("2014-01-01T07:01","7:01 AM");
  TryGetTime("2014-01-02T15:00","3:00 PM");
  TryGetTime("2014-01-02T23:59","11:59 PM");
  TryGetTime("2014-01-02T24:00","12:00 AM");
  TryGetTime("2014-01-02T12:00","12:00 PM");

}

void TestDate() {

  TryGetDate("20140101","");
  TryGetDate("Blah","");
  TryGetDate("2014-01-01T07:01","20140101");
  TryGetDate("2015-07-05T15:00","20150705");

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

int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));
  suite.test(TEST_CASE(TestDOW));
  suite.test(TEST_CASE(TestDuration));
  suite.test(TEST_CASE(TestTime));
  suite.test(TEST_CASE(TestDate));
  suite.test(TEST_CASE(TestIsoDateTime));
  suite.test(TEST_CASE(TestServiceDays));

  return suite.tear_down();
}
