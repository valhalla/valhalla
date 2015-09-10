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

int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));
  suite.test(TEST_CASE(TestDOW));
  suite.test(TEST_CASE(TestDuration));
  suite.test(TEST_CASE(TestTime));
  suite.test(TEST_CASE(TestDate));
  suite.test(TEST_CASE(TestIsoDateTime));

  return suite.tear_down();
}
