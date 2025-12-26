#ifndef VALHALLA_LOKI_SEARCH_H_
#define VALHALLA_LOKI_SEARCH_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

/**
 * Search class for finding locations within the route network
 */
class Search {
public:
  /**
   * Constructor
   * @param reader  an object used to access tiled route data
   */
  explicit Search(baldr::GraphReader& reader);

  /**
   * Destructor
   */
  ~Search();

  /**
   * Find locations within the route network given input locations
   *
   * @param locations  the positions which need to be correlated to the route network
   * @param costing    a costing object by which we can determine which portions of the graph are
   *                   accessible and therefor potential candidates
   * @return pathLocations the correlated data within the tile that matches the inputs. If a
   * projection is not found, it will not have any entry in the returned value.
   */
  std::unordered_map<baldr::Location, baldr::PathLocation>
  search(const std::vector<baldr::Location>& locations, const sif::cost_ptr_t& costing);

  void edges_in_bounds(const midgard::AABB2<midgard::PointLL>& bounds,
                       std::unordered_set<baldr::GraphId>& edge_container);

private:
  baldr::GraphReader& reader_;

  struct bin_handler_t;
  std::unique_ptr<bin_handler_t> handler_;
};

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_SEARCH_H_
