#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>

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

const locales_singleton_t& get_locales() {
  //thread safe static initializer for singleton
  static locales_singleton_t locales(load_narrative_locals());
  return locales;
}

}
}
