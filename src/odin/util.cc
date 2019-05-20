#include "baldr/rapidjson_utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <chrono>
#include <sstream>

#include <date/date.h>
#include <date/tz.h>

#include "locales.h"
#include "midgard/logging.h"
#include "odin/util.h"

namespace {

valhalla::odin::locales_singleton_t load_narrative_locals() {
  valhalla::odin::locales_singleton_t locales;
  // for each locale
  for (const auto& json : locales_json) {
    LOG_TRACE("LOCALES");
    LOG_TRACE("-------");
    LOG_TRACE("- " + json.first);
    // load the json
    boost::property_tree::ptree narrative_pt;
    std::stringstream ss;
    ss << json.second;
    rapidjson::read_json(ss, narrative_pt);
    LOG_TRACE("JSON read");
    // parse it into an object and add it to the map
    auto narrative_dictionary =
        std::make_shared<valhalla::odin::NarrativeDictionary>(json.first, narrative_pt);
    LOG_TRACE("NarrativeDictionary created");
    locales.insert(std::make_pair(json.first, narrative_dictionary));
    // insert all the aliases as this same object
    auto aliases = narrative_pt.get_child("aliases");
    for (const auto& alias : aliases) {
      auto name = alias.second.get_value<std::string>();
      auto inserted = locales.insert(std::make_pair(name, narrative_dictionary));
      if (!inserted.second) {
        throw std::logic_error("Alias '" + name + "' in json locale '" + json.first +
                               "' has duplicate with posix_locale '" +
                               inserted.first->second->GetLocale().name());
      }
    }
  }
  return locales;
}

} // namespace

namespace valhalla {
namespace odin {

// Returns the specified item surrounded with quotes.
std::string GetQuotedString(const std::string& item) {
  std::string str;
  str += "\"";
  str += item;
  str += "\"";
  return str;
}

bool IsSimilarTurnDegree(uint32_t path_turn_degree,
                         uint32_t intersecting_turn_degree,
                         bool is_right,
                         uint32_t turn_degree_threshold) {
  int32_t turn_degree_delta = 0;
  if (is_right) {
    turn_degree_delta = (((intersecting_turn_degree - path_turn_degree) + 360) % 360);
  } else {
    turn_degree_delta = (((path_turn_degree - intersecting_turn_degree) + 360) % 360);
  }

  return (turn_degree_delta <= turn_degree_threshold);
}

// Get the time from the inputed date.
// date_time is in the format of 2015-05-06T08:00-05:00
std::string get_localized_time(const std::string& date_time, const std::locale& locale) {
  if (date_time.find('T') == std::string::npos) {
    return "";
  }

  date::local_time<std::chrono::minutes> local_tp;
  std::istringstream in(date_time);
  in >> date::parse("%FT%R%z", local_tp);
  std::string time = date::format(locale, "%X", local_tp);

  // seconds is too granular so we try to remove
  if (time.find("PM") == std::string::npos && time.find("AM") == std::string::npos) {
    size_t found = time.find_last_of(':');
    if (found != std::string::npos) {
      time = time.substr(0, found);
    } else {
      found = time.find_last_of("00");
      if (found != std::string::npos) {
        time = time.substr(0, found - 1);
      }
    }
  } else {
    boost::replace_all(time, ":00 ", " ");
    if (time.substr(0, 1) == "0") {
      time = time.substr(1, time.size());
    }
  }

  boost::algorithm::trim(time);
  return time;
}

// Get the date from the inputed date.
// date_time is in the format of 2015-05-06T08:00-05:00
std::string get_localized_date(const std::string& date_time, const std::locale& locale) {
  if (date_time.find('T') == std::string::npos) {
    return "";
  }

  date::local_time<std::chrono::minutes> local_tp;
  std::istringstream in(date_time);
  in >> date::parse("%FT%R%z", local_tp);
  return date::format(locale, "%x", local_tp);
}

const locales_singleton_t& get_locales() {
  // thread safe static initializer for singleton
  static locales_singleton_t locales(load_narrative_locals());
  return locales;
}

const std::unordered_map<std::string, std::string>& get_locales_json() {
  return locales_json;
}

} // namespace odin
} // namespace valhalla
