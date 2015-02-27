#ifndef VALHALLA_MJOLNIR_GRAPHENHANCER_H
#define VALHALLA_MJOLNIR_GRAPHENHANCER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/mjolnir/graphtilebuilder.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to enhance graph tile information at the local level.
 */
class GraphEnhancer {
 public:
  /**
   * Constructor
   * @param  pt  Property tree with tile/level information.
   */
  GraphEnhancer(const boost::property_tree::ptree& pt);

  /**
   * Enhance the local level graph tile information.
   */
  bool Enhance();

 private:
  // Tile hierarchy/level information
  baldr::TileHierarchy tile_hierarchy_;
  uint32_t local_level_;

  // Tile logic
  Tiles tiles_;

  // Graphreader
  baldr::GraphReader graphreader_;

  // Maximum density (km/km2)
  float maxdensity_;

  // Get the density at the node.
  uint32_t GetNodeDensity(NodeInfoBuilder& nodeinfo);
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHENHANCER_H
