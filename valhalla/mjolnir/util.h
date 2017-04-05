#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <memory>
#include <vector>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphtilestorage.h>

namespace valhalla {
namespace mjolnir {

/**
 * Creates the tile storage handler for specified configuration.
 * @param  pt The configuration to use.
 * @return The storage handler instance.
 */
std::shared_ptr<valhalla::baldr::GraphTileStorage> CreateTileStorage(const boost::property_tree::ptree& pt);

/**
 * Splits a tag into a vector of strings.
 * @param  tag_value  tag to split
 * @param  delim      defaults to ;
 * @return the vector of strings
*/
std::vector<std::string> GetTagTokens(const std::string& tag_value,
                                      char delim = ';');

/**
 * Remove double quotes.
 * @param  s
 * @return string string with no quotes.
*/std::string remove_double_quotes(const std::string& s);

}
}
#endif  // VALHALLA_MJOLNIR_UTIL_H_
