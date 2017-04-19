#ifndef VALHALLA_MJOLNIR_TRANSITBUILDER_H
#define VALHALLA_MJOLNIR_TRANSITBUILDER_H

#include <cstdint>
#include <boost/property_tree/ptree.hpp>

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

}
}

#endif  // VALHALLA_MJOLNIR_TRANSITBUILDER_H
