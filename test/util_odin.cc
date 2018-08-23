#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "odin/util.h"
#include "test.h"
#include <locale>
#include <regex>
#include <set>
#include <stdexcept>

using namespace valhalla::odin;

namespace {

void test_get_locales() {
  const auto& init = get_locales();
  if (init.size() < 1)
    throw std::runtime_error("Should be at least one parsable test json file");
  if (init.find("en-US") == init.cend())
    throw std::runtime_error("Should find 'en-US' locales file");
}

void try_get_formatted_time(const std::string& date_time,
                            const std::string& expected_date_time,
                            const std::locale& locale) {
  std::string localized_time = get_localized_time(date_time, locale);
  if (localized_time != expected_date_time) {
    throw std::runtime_error("Incorrect Time: " + localized_time + " ---> " + expected_date_time +
                             " for locale: " + locale.name());
  }
}

void try_get_formatted_date(const std::string& date_time,
                            const std::string& expected_date_time,
                            const std::locale& locale) {
  std::string localized_date = get_localized_date(date_time, locale);
  if (localized_date != expected_date_time) {
    throw std::runtime_error("Incorrect Date: " + localized_date + " ---> " + expected_date_time +
                             " for locale: " + locale.name());
  }
}

void test_time() {

  try_get_formatted_time("2014-01-02T23:59-05:00", "23:59", std::locale());

  std::locale locale("en_US.UTF-8");
  try_get_formatted_time("20140101", "", locale);
  try_get_formatted_time("Blah", "", locale);
  try_get_formatted_time("2014-01-02T23:59-05:00", "11:59 PM", locale);
  try_get_formatted_time("2014-01-01T07:01-05:00", "7:01 AM", locale);
  try_get_formatted_time("2014-01-02T15:00-05:00", "3:00 PM", locale);
  try_get_formatted_time("2014-01-02T24:00-05:00", "12:00 AM", locale);
  try_get_formatted_time("2014-01-02T12:00-05:00", "12:00 PM", locale);

  locale = std::locale("de_DE.UTF-8");
  try_get_formatted_time("20140101", "", locale);
  try_get_formatted_time("Blah", "", locale);
  try_get_formatted_time("2014-01-02T23:59+01:00", "23:59", locale);
  try_get_formatted_time("2014-01-01T07:01+01:00", "07:01", locale);
  try_get_formatted_time("2014-01-02T15:00+01:00", "15:00", locale);
  try_get_formatted_time("2014-01-02T24:00+01:00", "00:00", locale);
  try_get_formatted_time("2014-01-02T12:00+01:00", "12:00", locale);

  locale = std::locale("cs_CZ.UTF-8");
  try_get_formatted_time("20140101", "", locale);
  try_get_formatted_time("Blah", "", locale);
  try_get_formatted_time("2014-01-02T23:59+01:00", "23:59", locale);
  try_get_formatted_time("2014-01-01T07:01+01:00", "07:01", locale);
  try_get_formatted_time("2014-01-02T15:00+01:00", "15:00", locale);
  try_get_formatted_time("2014-01-02T24:00+01:00", "00:00", locale);
  try_get_formatted_time("2014-01-02T12:00+01:00", "12:00", locale);

  locale = std::locale("it_IT.UTF-8");
  try_get_formatted_time("20140101", "", locale);
  try_get_formatted_time("Blah", "", locale);
  try_get_formatted_time("2014-01-02T23:59+01:00", "23:59", locale);
  try_get_formatted_time("2014-01-01T07:01+01:00", "07:01", locale);
  try_get_formatted_time("2014-01-02T15:00+01:00", "15:00", locale);
  try_get_formatted_time("2014-01-02T24:00+01:00", "00:00", locale);
  try_get_formatted_time("2014-01-02T12:00+01:00", "12:00", locale);

  locale = std::locale("ru_RU.UTF-8");
  try_get_formatted_time("20140101", "", locale);
  try_get_formatted_time("Blah", "", locale);
  try_get_formatted_time("2014-01-02T23:59+01:00", "23:59", locale);
  try_get_formatted_time("2014-01-01T07:01+01:00", "07:01", locale);
  try_get_formatted_time("2014-01-02T15:00+01:00", "15:00", locale);
  try_get_formatted_time("2014-01-02T24:00+01:00", "00:00", locale);
  try_get_formatted_time("2014-01-02T12:00+01:00", "12:00", locale);
}

void test_date() {

  try_get_formatted_date("2014-01-01T07:01-05:00", "01/01/14", std::locale());

  std::locale locale("en_US.UTF-8");
  try_get_formatted_date("20140101", "", locale);
  try_get_formatted_date("Blah", "", locale);
  try_get_formatted_date("2014-01-01T07:01-05:00", "01/01/2014", locale);
  try_get_formatted_date("2015-07-05T15:00-05:00", "07/05/2015", locale);

  locale = std::locale("de_DE.UTF-8");
  try_get_formatted_date("20140101", "", locale);
  try_get_formatted_date("Blah", "", locale);
  try_get_formatted_date("2014-01-01T07:01+01:00", "01.01.2014", locale);
  try_get_formatted_date("2015-07-05T15:00+01:00", "05.07.2015", locale);

  locale = std::locale("cs_CZ.UTF-8");
  try_get_formatted_date("20140101", "", locale);
  try_get_formatted_date("Blah", "", locale);
  try_get_formatted_date("2014-01-01T07:01+01:00", "1.1.2014", locale);
  try_get_formatted_date("2015-07-05T15:00+01:00", "5.7.2015", locale);
  try_get_formatted_date("2015-12-13T15:00+01:00", "13.12.2015", locale);

  locale = std::locale("it_IT.UTF-8");
  try_get_formatted_date("20140101", "", locale);
  try_get_formatted_date("Blah", "", locale);
  try_get_formatted_date("2014-01-01T07:01+01:00", "01/01/2014", locale);
  try_get_formatted_date("2015-07-05T15:00+01:00", "05/07/2015", locale);
  try_get_formatted_date("2015-12-13T15:00+01:00", "13/12/2015", locale);

  locale = std::locale("ru_RU.UTF-8");
  try_get_formatted_date("20140101", "", locale);
  try_get_formatted_date("Blah", "", locale);
  try_get_formatted_date("2014-01-01T07:01+01:00", "01.01.2014", locale);
  try_get_formatted_date("2015-07-05T15:00+01:00", "05.07.2015", locale);
  try_get_formatted_date("2015-12-13T15:00+01:00", "13.12.2015", locale);
}

void test_supported_locales() {
  // crack open english
  const auto& jsons = get_locales_json();
  const auto en_us_json = jsons.find("en-US");
  if (en_us_json == jsons.cend())
    throw std::runtime_error("No en-US found!");
  boost::property_tree::ptree en_us;
  std::stringstream ss;
  ss << en_us_json->second;
  rapidjson::read_json(ss, en_us);

  // look at each one
  for (const auto& locale : jsons) {
    if (locale.first == "en-US")
      continue;
    boost::property_tree::ptree other;
    std::stringstream other_ss;
    other_ss << locale.second;
    rapidjson::read_json(other_ss, other);

    // check the locale is supported
    std::string posix_locale = other.get<std::string>("posix_locale");
    LOG_TRACE("Verify supported locale for posix_locale=" + posix_locale);
    std::locale l;
    try {
      l = std::locale(posix_locale.c_str());
    } catch (std::runtime_error& rte) {
      throw std::runtime_error("Locale not found for: " + posix_locale);
      ;
    }

    // check each instruction
    for (const auto& instruction : en_us.get_child("instructions")) {
      const auto& other_inst = other.get_child("instructions." + instruction.first);
      // check the number of things in each thing
      for (const auto& sub : instruction.second) {
        auto other_sub = other_inst.get_child_optional(sub.first);
        if (!other_sub)
          throw std::runtime_error("Missing: " + locale.first + "::" + instruction.first + "." +
                                   sub.first);
        // Check for "transit_stop_count_labels.other" - it must be present for all
        if (sub.first == "transit_stop_count_labels") {
          auto transit_stop_count_label_other = other_sub->get_child_optional("other");
          if (!transit_stop_count_label_other)
            throw std::runtime_error("Missing: " + locale.first + "::" + instruction.first + "." +
                                     sub.first + ".other");
          continue; // Skip the other checks
        }
        if (sub.second.size() != other_sub->size())
          throw std::runtime_error("Wrong number of elements in " + locale.first +
                                   "::" + instruction.first + "." + sub.first);
        // check the keys
        std::set<std::string> keys, other_keys;
        for (const auto& kv : sub.second)
          keys.insert(kv.first);
        for (const auto& kv : *other_sub)
          other_keys.insert(kv.first);
        if (keys != other_keys)
          throw std::runtime_error("Wrong keys in " + locale.first + "::" + instruction.first + "." +
                                   sub.first);
      }
      // check the phrases
      for (const auto& phrase : instruction.second.get_child("phrases")) {
        const auto& other_phrase = other_inst.get<std::string>("phrases." + phrase.first);
        // parse out tags from phrase, and check for them
        std::smatch m;
        std::regex e("(<[A-Z_0-9]+>)");
        auto str = phrase.second.get_value<std::string>();
        if (std::regex_search(str, m, e))
          for (const auto& tag : m)
            if (other_phrase.find(tag.str()) == std::string::npos)
              throw std::runtime_error("Couldn't find " + tag.str() + " in " + locale.first +
                                       "::" + instruction.first + ".phrases." + phrase.first);
      }
    }
  }
}
} // namespace

int main() {
  test::suite suite("util");

  suite.test(TEST_CASE(test_supported_locales));
  suite.test(TEST_CASE(test_get_locales));
  suite.test(TEST_CASE(test_time));
  suite.test(TEST_CASE(test_date));

  return suite.tear_down();
}
