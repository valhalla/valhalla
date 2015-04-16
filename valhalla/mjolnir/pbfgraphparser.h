#ifndef VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
#define VALHALLA_MJOLNIR_PBFGRAPHPARSER_H

#include <string>
#include <vector>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/mjolnir/osmdata.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to parse OSM protocol buffer extracts.
 */
class PBFGraphParser {
 public:

  /**
   * Loads given input files
   * @param  pt             properties file
   * @param  input_files    the protobuf files to parse
   * @param  ways_file      where to store the ways so they arent in memory
   * @param  way_nodes_file where to store the nodes so they arent in memory
   */
  static OSMData Parse(const boost::property_tree::ptree& pt, const std::vector<std::string>& input_files,
      const std::string& ways_file, const std::string& way_nodes_file);

};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
