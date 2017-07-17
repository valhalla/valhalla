#ifndef VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
#define VALHALLA_MJOLNIR_HIERARCHYBUILDER_H

#include <cstdint>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class HierarchyBuilder {
 public:

  /**
   * Build the set of hierarchies based on the TileHierarchy configuration
   * and the current local hierarchy.
   */
  static void Build(const boost::property_tree::ptree& pt);
};

}
}

#endif  // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
