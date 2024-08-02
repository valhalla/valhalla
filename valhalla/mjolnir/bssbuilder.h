#ifndef VALHALLA_MJOLNIR_BSSBUILDER_H
#define VALHALLA_MJOLNIR_BSSBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <valhalla/mjolnir/osmdata.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to build bike share station from OSM within the graph tiles.
 */
class BssBuilder {
public:
  /**
   * Add bike share stations information to the graph tiles.
   * @param pt   Property tree containing the hierarchy configuration
   *             and other configuration needed to build bss.
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const OSMData& osmdata,
                    const std::string& bss_nodes_bin);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_BSSBUILDER_H
