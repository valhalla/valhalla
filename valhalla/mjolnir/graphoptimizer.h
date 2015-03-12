#ifndef VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
#define VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/mjolnir/graphtilebuilder.h>

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
