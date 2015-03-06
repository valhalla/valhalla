#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>
#include <memory>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/signinfo.h>

#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/sequence.h>
#include <valhalla/mjolnir/dataquality.h>
#include <valhalla/mjolnir/edgeinfobuilder.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
 public:

  /**
   * Tell the builder to build the tiles from the provided datasource
   * and configs
   * @param  config
   * @param  osmdata  OSM data used to build the graph.
   */
  static void Build(const boost::property_tree::ptree& pt, OSMData& osmdata);

};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
