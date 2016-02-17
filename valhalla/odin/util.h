#ifndef VALHALLA_ODIN_UTIL_H_
#define VALHALLA_ODIN_UTIL_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/odin/narrative_dictionary.h>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace odin {

/**
 * Returns the specified item surrounded with quotes.
 * @param item  specified text to surround with quotes
 * @return the specified item surrounded with quotes.
 */
std::string GetQuotedString(const std::string& item);

bool IsSimilarTurnDegree(uint32_t path_turn_degree,
                         uint32_t intersecting_turn_degree, bool is_right,
                         uint32_t turn_degree_threshold = 30);

DirectionsOptions GetDirectionsOptions(const boost::property_tree::ptree& pt);

/**
 * Returns a list of locales mapped to NarrativeDictionary's containing parsed narrative information
 *
 * @return the map of locale to ptree of parse narrative json/yml
 */
using locales_singleton_t = std::unordered_map<std::string, NarrativeDictionary>;
const locales_singleton_t& get_locales();

}
}
#endif  // VALHALLA_ODIN_UTIL_H_
