#include "test.h"

#include "valhalla/baldr/datetime.h"

#include <string>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryGetDaysFromPivotDate(std::string date, uint32_t expected_days) {
  DateTime dt;
  if (dt.getDaysFromPivotDate(date) != expected_days) {
    throw std::runtime_error(
        std::string("Incorrect number of days from ")
    + date);
  }
}

void TryGetSecondsFromMidnight(std::string time, uint32_t expected_seconds) {
  DateTime dt;
  if (dt.getSecondsFromMidnight(time) != expected_seconds) {
    throw std::runtime_error(
        std::string("Incorrect number of seconds from ")
    + time);
  }
}

void TestGetDaysFromPivotDate() {
  TryGetDaysFromPivotDate("20140101", 0);
  TryGetDaysFromPivotDate("20140102", 1);
  TryGetDaysFromPivotDate("19990101", 0);
  TryGetDaysFromPivotDate("20150506", 490);
}

void TestGetSecondsFromMidnight() {
  TryGetSecondsFromMidnight("00:00:00", 0);
  TryGetSecondsFromMidnight("01:00:00", 3600);
  TryGetSecondsFromMidnight("05:34:34", 20074);
  TryGetSecondsFromMidnight("26:16:01", 94561);
  TryGetSecondsFromMidnight("36:16:01", 130561);
  TryGetSecondsFromMidnight("24:01:01", 86461);
}
}

int main(void) {
  test::suite suite("datetime");

  suite.test(TEST_CASE(TestGetDaysFromPivotDate));
  suite.test(TEST_CASE(TestGetSecondsFromMidnight));

  return suite.tear_down();
}
