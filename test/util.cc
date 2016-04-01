#include "test.h"
#include "odin/util.h"

using namespace valhalla::odin;


namespace {

  void test_get_locales() {
    const auto& init = get_locales();
    if(init.size() < 1)
      throw std::runtime_error("Should be at least one parsable test json file");
    if(init.find("en-US") == init.cend())
      throw std::runtime_error("Should find 'en-US' locales file");
  }

  void try_get_formatted_time(std::string date_time, std::string expected_date_time,
                              std::string locale) {
    if (get_localized_time(date_time, locale) != expected_date_time) {
      throw std::runtime_error("Incorrect Time: " + date_time + " ---> " +
                               expected_date_time + " for locale: " + locale);
    }
  }

  void try_get_formatted_date(std::string date_time, std::string expected_date_time,
                              std::string locale) {
    if (get_localized_date(date_time, locale) != expected_date_time) {
      throw std::runtime_error("Incorrect Date: " + date_time + " ---> " +
                               expected_date_time + " for locale: " + locale);
    }
  }

  void test_time() {

    std::string locale = "blah";
    try_get_formatted_time("2014-01-02T23:59","11:59 PM",locale);

    locale = "en_US.utf8";
    try_get_formatted_time("20140101","",locale);
    try_get_formatted_time("Blah","",locale);
    try_get_formatted_time("2014-01-02T23:59","11:59 PM",locale);
    try_get_formatted_time("2014-01-01T07:01","7:01 AM",locale);
    try_get_formatted_time("2014-01-02T15:00","3:00 PM",locale);
    try_get_formatted_time("2014-01-02T24:00","12:00 AM",locale);
    try_get_formatted_time("2014-01-02T12:00","12:00 PM",locale);

    locale = "de_DE.utf8";
    try_get_formatted_time("20140101","",locale);
    try_get_formatted_time("Blah","",locale);
    try_get_formatted_time("2014-01-02T23:59","23:59",locale);
    try_get_formatted_time("2014-01-01T07:01","07:01",locale);
    try_get_formatted_time("2014-01-02T15:00","15:00",locale);
    try_get_formatted_time("2014-01-02T24:00","00:00",locale);
    try_get_formatted_time("2014-01-02T12:00","12:00",locale);
  }

  void test_date() {

    std::string locale = "blah";
    try_get_formatted_date("2014-01-01T07:01","01/01/2014",locale);

    locale = "en_US.utf8";
    try_get_formatted_date("20140101","",locale);
    try_get_formatted_date("Blah","",locale);
    try_get_formatted_date("2014-01-01T07:01","01/01/2014",locale);
    try_get_formatted_date("2015-07-05T15:00","07/05/2015",locale);

    locale = "de_DE.utf8";
    try_get_formatted_date("20140101","",locale);
    try_get_formatted_date("Blah","",locale);
    try_get_formatted_date("2014-01-01T07:01","01.01.2014",locale);
    try_get_formatted_date("2015-07-05T15:00","05.07.2015",locale);
  }
}

int main() {
  test::suite suite("util");

  // initializing and getting locales
  suite.test(TEST_CASE(test_get_locales));
  suite.test(TEST_CASE(test_time));
  suite.test(TEST_CASE(test_date));

  return suite.tear_down();
}
