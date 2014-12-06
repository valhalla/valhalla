#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include "pbfgraphbuilder.h"
#include "osmpbfreader.h"
#include "geo/pointll.h"

using namespace CanalTP;  // For OSM pbf reader
//using namespace std;

namespace valhalla{
namespace mjolnir{

// Node map
typedef std::map<uint64_t, OSMNode> node_map_type;

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
 public:
  /**
   * Constructor
   */
  GraphBuilder() {
    // Initialize counts
    relationcount = 0;
    nodecount = 0;
  }

  /**
   * Callback method for OSMPBFReader. Called when a node is parsed.
   */
  void node_callback(uint64_t osmid, double lng, double lat,
                     const Tags &/*tags*/);

  /**
   * Callback method for OSMPBFReader. Called when a way is parsed.
   */
  void way_callback(uint64_t osmid, const Tags &tags,
                    const std::vector<uint64_t> &refs);

  /**
   * Callback method for OSMPBFReader. Called when a relation is parsed.
   */
  void relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                          const References & /*refs*/);

  /**
   * Debug method to print the number of nodes, ways, and relations that
   * were read and parsed from the pbf input file.
   */
  void PrintCounts();

  /**
   * This method computes how many times a node is used. Nodes with use count
   * > 1 are intersections (or ends of a way) and become nodes in the graph.
   */
  void SetNodeUses();

  /**
   * Construct edges in the graph.
   */
  void ConstructEdges();

 private:
  unsigned int relationcount;
  unsigned int nodecount;

  // Map that stores all the nodes read
  node_map_type nodes;

  // Stores all the nodes of all the ways that are part of the road network
  std::vector<OSMWay> ways_;

  // Stores all the edges
  std::vector<Edge> edges_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
