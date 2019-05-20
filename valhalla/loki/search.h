#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>

#include <functional>

namespace valhalla {
namespace loki {

/**
 * A callable element which returns a value between 0 and 1 based on
 * how desirable the edge is for use as a location. 0 indicated the
 * edge should not be used as a location (inaccessible).
 *
 * TODO: remove the filtering of transit edges when they get proper
 * opposing edges added to the graph
 */
const sif::EdgeFilter PassThroughEdgeFilter = [](const baldr::DirectedEdge* edge) -> float {
  return !(edge->is_shortcut() || edge->IsTransitLine());
};

/**
 * A callable element which returns true if a node should be
 * filtered out of a graph traversal
 */
const sif::NodeFilter PassThroughNodeFilter = [](const baldr::NodeInfo* node) { return false; };

/**
 * Find an location within the route network given an input location
 * same tiled route data and a search strategy
 *
 * @param locations      the positions which need to be correlated to the route network
 * @param reader         and object used to access tiled route data TODO: switch this out for a
 * proper cache
 * @param edge_filter    a function/functor to be used in the rejection of edges. defaults to a pass
 * through filter
 * @param node_filter    a function/functor to be used in the rejection of nodes used in graph
 * traversal. defaults to a pass through filter
 * @return pathLocations the correlated data with in the tile that matches the inputs. If a
 * projection is not found, it will not have any entry in the returned value.
 */
std::unordered_map<baldr::Location, baldr::PathLocation>
Search(const std::vector<baldr::Location>& locations,
       baldr::GraphReader& reader,
       const sif::EdgeFilter& edge_filter = PassThroughEdgeFilter,
       const sif::NodeFilter& node_filter = PassThroughNodeFilter);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_SEARCH_H_
