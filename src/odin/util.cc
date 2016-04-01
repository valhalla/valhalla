#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <valhalla/midgard/logging.h>

#include "proto/directions_options.pb.h"
#include "odin/util.h"
#include "odin/narrative_dictionary.h"
#include "locales.h"

namespace {

  valhalla::odin::locales_singleton_t load_narrative_locals() {
    valhalla::odin::locales_singleton_t locales;
    //for each locale
    for(const auto& json : locales_json) {
      LOG_TRACE("LOCALES");
      LOG_TRACE("-------");
      LOG_TRACE("- " + json.first);
      try {
        boost::property_tree::ptree narrative_pt;
        std::stringstream ss; ss << json.second;
        boost::property_tree::read_json(ss, narrative_pt);
        LOG_TRACE("JSON read");
        valhalla::odin::NarrativeDictionary narrative_dictionary(narrative_pt);
        LOG_TRACE("NarrativeDictionary created");
        locales.emplace(json.first, std::move(narrative_dictionary));
      }
      catch(...) {
        LOG_WARN("Failed to parse narrative for locale: " + json.first);
      }
    }
    return locales;
  }

}


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
                         uint32_t intersecting_turn_degree, bool is_right,
                         uint32_t turn_degree_threshold) {
  int32_t turn_degree_delta = 0;
  if (is_right) {
    turn_degree_delta = (((intersecting_turn_degree - path_turn_degree) + 360)
        % 360);
  } else {
    turn_degree_delta = (((path_turn_degree - intersecting_turn_degree) + 360)
        % 360);
  }

  return (turn_degree_delta <= turn_degree_threshold);
}

DirectionsOptions GetDirectionsOptions(const boost::property_tree::ptree& pt) {
  valhalla::odin::DirectionsOptions directions_options;

  // TODO: validate values coming soon...

  auto units_ptr = pt.get_optional<std::string>("units");
  if (units_ptr) {
    std::string units = *units_ptr;
    if ((units == "miles") || (units == "mi")) {
      directions_options.set_units(DirectionsOptions_Units_kMiles);
    } else {
      directions_options.set_units(DirectionsOptions_Units_kKilometers);
    }
  }

  auto lang_ptr = pt.get_optional<std::string>("language");
  if (lang_ptr) {
    directions_options.set_language(*lang_ptr);
  }

  auto narr_ptr = pt.get_optional<bool>("narrative");
  if (narr_ptr) {
    directions_options.set_narrative(*narr_ptr);
  }

  return directions_options;
}

//Get the time from the inputed date.
//date_time is in the format of 2015-05-06T08:00
std::string get_localized_time(const std::string& date_time, const std::string& locale) {
  std::stringstream ss("");
  try {
    if (date_time.find("T") == std::string::npos)
      return ss.str();

    boost::local_time::local_time_input_facet* input_facet =
        new boost::local_time::local_time_input_facet("%Y-%m-%dT%H:%M");
    boost::posix_time::time_facet* output_facet = new boost::posix_time::time_facet("%X");

    ss.imbue(std::locale(ss.getloc(), input_facet));
    boost::posix_time::ptime pt;
    ss.str(date_time);
    ss >> pt; // read in with the format of "%Y-%m-%dT%H:%M"
    ss.str("");
    try {
      ss.imbue(std::locale(std::locale(locale.c_str()), output_facet));  // output in the locale requested
      ss << pt;
      std::string time = ss.str();

      if (time.find("PM") == std::string::npos && time.find("AM") == std::string::npos) {//AM or PM
        std::size_t found = time.find_last_of(":"); // remove seconds.
        if (found != std::string::npos)
          time = time.substr(0,found);
        else {
          found = time.find_last_of("00"); // remove seconds.
          if (found != std::string::npos)
            time = time.substr(0,found-1);
        }
      } else { // has AM or PM
        boost::replace_all(time, ":00 ", " ");
        if (time.substr(0,1) == "0")
          time = time.substr(1,time.size());
      }
      ss.str(time);
    } catch (std::exception& e) { //Locale is not installed!  Return default.
      output_facet = new boost::posix_time::time_facet("%l:%M %p");
      ss.imbue(std::locale(std::locale::classic(), output_facet));
      ss << pt;
      std::cout << ss.str() << std::endl;

    }
  } catch (std::exception& e){}
  std::string result = ss.str();
  boost::algorithm::trim(result);
  return result;
}

//Get the date from the inputed date.
//date_time is in the format of 2015-05-06T08:00
std::string get_localized_date(const std::string& date_time, const std::string& locale) {
  std::stringstream ss("");
  try {
    if (date_time.find("T") == std::string::npos)
      return ss.str();

    boost::local_time::local_time_input_facet* input_facet =
        new boost::local_time::local_time_input_facet("%Y-%m-%dT%H:%M");
    boost::posix_time::time_facet* output_facet = new boost::posix_time::time_facet("%x");

    ss.imbue(std::locale(ss.getloc(), input_facet));
    boost::posix_time::ptime pt;
    ss.str(date_time);
    ss >> pt; // read in with the format of "%Y-%m-%dT%H:%M"
    ss.str("");
    try {
      ss.imbue(std::locale(std::locale(locale.c_str()), output_facet));  // output in the locale requested
    } catch (std::exception& e) { //Locale is not installed!  Return default.
      output_facet = new boost::posix_time::time_facet("%Y%m%d");
      ss.imbue(std::locale(std::locale::classic(), output_facet));
    }
    ss << pt;
  } catch (std::exception& e){}
  std::string result = ss.str();
  boost::algorithm::trim(result);
  return result;
}

const locales_singleton_t& get_locales() {
  //thread safe static initializer for singleton
  static locales_singleton_t locales(load_narrative_locals());
  return locales;
}

}
}
