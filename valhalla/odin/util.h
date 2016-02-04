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
 * Returns a list of locales mapped to ptrees containing parsed narrative information
 * Call this once from the main thread with a directory argument to initialize it.
 * After that, from any thread, just call it without arguments
 *
 * @param narrative_locales_directory  the directory where the narrative locale files are located
 * @return the map of locale to ptree of parse narrative json/yml
 */
using locales_singleton_t = std::unordered_map<std::string, NarrativeDictionary>;
const locales_singleton_t& get_locales(const std::string& locales_directory = "");

}
}
#endif  // VALHALLA_ODIN_UTIL_H_
