#ifndef VALHALLA_MJOLNIR_TRANSITBUILDER_H
#define VALHALLA_MJOLNIR_TRANSITBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to build transit data within the graph tiles.
 */
class TransitBuilder {
public:
  /**
   * Add transit information to the graph tiles.
   * @param pt   Property tree containing the hierarchy configuration
   *             and other configuration needed to build transit.
   */
  static void Build(const boost::property_tree::ptree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_TRANSITBUILDER_H
