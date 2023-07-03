#ifndef VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
#define VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to validate the graph. Creates opposing edge indexes -
 * this is an excellent way to validate proper connectivity.
 * TODO - generate statistics, quality measures.
 */
class GraphValidator {
public:
  /**
   * Validate the graph tiles.
   */
  static void Validate(const boost::property_tree::ptree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
