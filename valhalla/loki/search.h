#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

/**
 * Find an location within the route network given an input location
 * same tiled route data and a search strategy
 *
 * @param locations      the positions which need to be correlated to the route network
 * @param reader         and object used to access tiled route data TODO: switch this out for a
 * proper cache
 * @param costing        a costing object by which we can determine which portions of the graph are
 *                       accessible and therefor potential candidates
 * @return pathLocations the correlated data with in the tile that matches the inputs. If a
 * projection is not found, it will not have any entry in the returned value.
 */
std::unordered_map<baldr::Location, baldr::PathLocation>
Search(const std::vector<baldr::Location>& locations,
       baldr::GraphReader& reader,
       const std::shared_ptr<sif::DynamicCost>& costing);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_SEARCH_H_
