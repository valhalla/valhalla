#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {


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

/**
 * Build an entire valhalla tileset give a config file and some input pbfs
 * @param config              used to tell the function where and how to build the tiles
 * @param input_files         tells what osm pbf files to build the tiles from
 * @param bin_file_prefix     name prefix for mmapped flat files used when parsing the osm pbf
 * @param free_protobuf       whether or not to unload the protobuffer lib, you cant use libpbf after doing this
 *
 */
void build_tile_set(const boost::property_tree::ptree& config,
    const std::vector<std::string>& input_files, const std::string& bin_file_prefix = "",
    bool free_protobuf = true);

}
}
#endif  // VALHALLA_MJOLNIR_UTIL_H_
