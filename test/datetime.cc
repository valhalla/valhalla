#include "test.h"

#include <string>
#include <valhalla/baldr/datetime.h>

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

void TryGetSecondsFromMidnight(std::string date_time, uint32_t expected_seconds) {
  if (DateTime::seconds_from_midnight(date_time) != expected_seconds) {
    throw std::runtime_error(
        std::string("Incorrect number of seconds from ")
    + date_time);
  }
}

}

void TestGetDaysFromPivotDate() {
  TryGetDaysFromPivotDate("20140101", 0);
  TryGetDaysFromPivotDate("20140102", 1);
  TryGetDaysFromPivotDate("19990101", 0);
  TryGetDaysFromPivotDate("20150506", 490);

  TryGetDaysFromPivotDate("20140101T07:01", 0);
  TryGetDaysFromPivotDate("20140102T15:00", 1);
  TryGetDaysFromPivotDate("19990101T:00:00", 0);
  TryGetDaysFromPivotDate("2015-05-06T08:00", 490);
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

  return suite.tear_down();
}
