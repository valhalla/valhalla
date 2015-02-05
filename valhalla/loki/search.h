#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>

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
 * A function pointer which returns true if an edge should be
 * filtered out of the correlated set and false if the edge is usable
 * TODO: if ever we need to hold state when filtring we'll need to
 * update this to a functor and use something like std::function<>
 * in place of straight up function pointers
 */
using EdgeFilter = bool (*)(const valhalla::baldr::DirectedEdge*);
bool PassThroughFilter(const valhalla::baldr::DirectedEdge*) { return false; }

/**
 * Find an location within the route network given an input location
 * same tiled route data and a search strategy
 *
 * @param location  the position which needs to be correlated to the route network
 * @param reader    and object used to access tiled route data TODO: switch this out for a proper cache
 * @param strategy  what type of search to do
 * @param filter    a function to be used in the rejection of edges
 * @return pathLocation  the correlated data with in the tile that matches the input
 */
baldr::PathLocation Search(const baldr::Location& location, baldr::GraphReader& reader,
  const SearchStrategy strategy = SearchStrategy::EDGE, EdgeFilter filter = PassThroughFilter);

}
}

#endif  // VALHALLA_LOKI_SEARCH_H_
