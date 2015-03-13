#ifndef VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
#define VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to optimize the graph. Creates opposing edge indexes.
 * TODO - intersection costing? elevation factors?
 */
class GraphOptimizer {
 public:
  /**
   * Optimize the graph tiles.
   */
  static void Optimize(const boost::property_tree::ptree& pt);
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
