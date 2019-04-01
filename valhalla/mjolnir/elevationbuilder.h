#ifndef VALHALLA_MJOLNIR_ELEVATIONBUILDER_H
#define VALHALLA_MJOLNIR_ELEVATIONBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to add elevation data to the Valhalla graph tiles.
 */
class ElevationBuilder {
public:
  /**
   * Add elevation information to the graph tiles.
   */
  static void Build(const boost::property_tree::ptree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_ELEVATIONBUILDER_H
