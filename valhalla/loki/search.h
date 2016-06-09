#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/sif/dynamiccost.h>

#include <functional>

namespace valhalla{
namespace loki{

/**
 * A callable element which returns true if an edge should be
 * filtered out of the correlated set and false if the edge is usable
 *
 * TODO: remove the filtering of transit edges when they get proper
 * opposing edges added to the graph
 */
const sif::EdgeFilter PassThroughEdgeFilter = [](const baldr::DirectedEdge* edge){ return edge->trans_up() || edge->trans_down() || edge->IsTransitLine(); };

/**
 * A callable element which returns true if a node should be
 * filtered out of a graph traversal
 */
const sif::NodeFilter PassThroughNodeFilter = [](const baldr::NodeInfo* node){ return false; };

/**
 * Find an location within the route network given an input location
 * same tiled route data and a search strategy
 *
 * @param location       the position which needs to be correlated to the route network
 * @param reader         and object used to access tiled route data TODO: switch this out for a proper cache
 * @param edge_filter    a function/functor to be used in the rejection of edges. defaults to a pass through filter
 * @param node_filter    a function/functor to be used in the rejection of nodes used in graph traversal. defaults to a pass through filter
 * @return pathLocation  the correlated data with in the tile that matches the input
 */
baldr::PathLocation Search(const baldr::Location& location, baldr::GraphReader& reader,
  const sif::EdgeFilter& edge_filter = PassThroughEdgeFilter, const sif::NodeFilter& node_filter = PassThroughNodeFilter);

}
}

#endif  // VALHALLA_LOKI_SEARCH_H_
