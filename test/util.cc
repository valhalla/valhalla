#include "test.h"
#include "odin/util.h"

#include <set>
#include <locale>
#include <regex>
#include <boost/property_tree/json_parser.hpp>

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

  void test_supported_locales() {
    //crack open english
    const auto& jsons = get_locales_json();
    const auto en_us_json = jsons.find("en-US");
    if(en_us_json == jsons.cend())
      throw std::runtime_error("No en-US found!");
    boost::property_tree::ptree en_us;
    std::stringstream ss; ss << en_us_json->second;
    boost::property_tree::read_json(ss, en_us);

    //look at each one
    for(const auto& locale : jsons) {
      if(locale.first == "en-US")
        continue;
      boost::property_tree::ptree other;
      std::stringstream other_ss; other_ss << locale.second;
      boost::property_tree::read_json(other_ss, other);

      //check the locale is supported
      std::locale l(other.get<std::string>("posix_locale").c_str());

      //check each instruction
      for(const auto& instruction : en_us.get_child("instructions")) {
        const auto& other_inst = other.get_child("instructions." + instruction.first);
        //check the number of things in each thing
        for(const auto& sub : instruction.second) {
          auto other_sub = other_inst.get_child_optional(sub.first);
          if(!other_sub)
            throw std::runtime_error("Missing: " + locale.first + "::" + instruction.first + "." + sub.first);
          if(sub.second.size() != other_sub->size())
            throw std::runtime_error("Wrong number of elements in " +
              locale.first + "::" + instruction.first + "." + sub.first);
          //check the keys
          std::set<std::string> keys, other_keys;
          for(const auto& kv : sub.second) keys.insert(kv.first);
          for(const auto& kv : *other_sub) other_keys.insert(kv.first);
          if(keys != other_keys)
            throw std::runtime_error("Wrong keys in " +
              locale.first + "::" + instruction.first + "." + sub.first);
        }
        //check the phrases
        for(const auto& phrase : instruction.second.get_child("phrases")) {
          const auto& other_phrase = other_inst.get<std::string>("phrases." + phrase.first);
          //parse out tags from phrase, and check for them
          std::smatch m;
          std::regex e("(<[A-Z_0-9]+>)");
          if(std::regex_search(phrase.second.get_value<std::string>(), m, e))
            for(const auto& tag : m)
              if(other_phrase.find(tag.str()) == std::string::npos)
                throw std::runtime_error("Couldn't find " + tag.str() + " in " +
                  locale.first + "::" + instruction.first + ".phrases." + phrase.first);
        }

      }
    }
  }
}

int main() {
  test::suite suite("util");

  suite.test(TEST_CASE(test_supported_locales));
  suite.test(TEST_CASE(test_get_locales));
  suite.test(TEST_CASE(test_time));
  suite.test(TEST_CASE(test_date));

  return suite.tear_down();
}
