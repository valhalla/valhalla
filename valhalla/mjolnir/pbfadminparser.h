#ifndef VALHALLA_MJOLNIR_PBFADMINPARSER_H
#define VALHALLA_MJOLNIR_PBFADMINPARSER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/mjolnir/osmadmindata.h>

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
  static OSMAdminData Parse(const boost::property_tree::ptree& pt,
                            const std::vector<std::string>& input_files);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFADMINPARSER_H
