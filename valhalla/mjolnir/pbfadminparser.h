#ifndef VALHALLA_MJOLNIR_PBFADMINPARSER_H
#define VALHALLA_MJOLNIR_PBFADMINPARSER_H

#include <cstdint>
#include <string>
#include <vector>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/mjolnir/osmdata.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to parse OSM administrative protocol buffer extracts.
 */
class PBFAdminParser {
 public:

  /**
   * Loads given input files
   */
  static OSMData Parse(const boost::property_tree::ptree& pt, const std::vector<std::string>& input_files);

};

}
}

#endif  // VALHALLA_MJOLNIR_PBFADMINPARSER_H
