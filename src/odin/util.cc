#include "odin/util.h"
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/midgard/logging.h>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace {

  valhalla::odin::locales_singleton_t load_narrative_locals(const std::string& directory) {
    boost::filesystem::recursive_directory_iterator locale_file_itr(directory);
    boost::filesystem::recursive_directory_iterator end_file_itr;
    valhalla::odin::locales_singleton_t locales;
    //for each file
    for(; locale_file_itr != end_file_itr; ++locale_file_itr) {
      //is it a file and looking like json
      if(boost::filesystem::is_regular(locale_file_itr->path()) &&
         locale_file_itr->path().extension() == ".json") {
        //TODO: validate the locale string in some way
        std::string locale = locale_file_itr->path().stem().string();
#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE("LOCALES");
  LOG_TRACE("-------");
  LOG_TRACE("- " + locale);
#endif
        try {
          boost::property_tree::ptree narrative;
          boost::property_tree::read_json(locale_file_itr->path().string(), narrative);
          locales.emplace(std::move(locale), std::move(narrative));

        }
        catch(...) {
          LOG_WARN("Failed to parse narrative for locale: " + locale);
        }
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

const locales_singleton_t& get_locales(const std::string& locales_directory) {
  //thread safe static initializer for singleton
  static locales_singleton_t locales(load_narrative_locals(locales_directory));
  return locales;
}

}
}
