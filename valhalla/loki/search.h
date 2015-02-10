#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>

#include <functional>

namespace valhalla{
namespace loki{

/**
 * Notifies the search which type of precision should be used
 * Should it find the closest node to the input position
 * or instead try to use the geometry of the edges to find
 * an intermediate position along the edge. The former is
 * quite a bit faster but can be extremely inaccurate in
 * low density route networks
 */
enum class SearchStrategy : bool { NODE, EDGE };

/**
 * A callable element which returns true if an edge should be
 * filtered out of the correlated set and false if the edge is usable
 */
using EdgeFilter = std::function<bool (const baldr::DirectedEdge*)>;
const EdgeFilter PathThroughFilter = [](const baldr::DirectedEdge* edge){ return edge->trans_up() || edge->trans_down(); };

/**
 * Find an location within the route network given an input location
 * same tiled route data and a search strategy
 *
 * @param location  the position which needs to be correlated to the route network
 * @param reader    and object used to access tiled route data TODO: switch this out for a proper cache
 * @param filter    a function/functor to be used in the rejection of edges. defaults to a pass through filter
 * @param strategy  what type of search to do, defaults to edge based searching
 * @return pathLocation  the correlated data with in the tile that matches the input
 */
baldr::PathLocation Search(const baldr::Location& location, baldr::GraphReader& reader,
  EdgeFilter filter = PathThroughFilter,
  const SearchStrategy strategy = SearchStrategy::EDGE);

}
}

#endif  // VALHALLA_LOKI_SEARCH_H_
