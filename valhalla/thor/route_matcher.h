#ifndef VALHALLA_THOR_ROUTE_MATCHER_H_
#define VALHALLA_THOR_ROUTE_MATCHER_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/pathinfo.h>

#include <vector>

namespace valhalla {
namespace thor {

class RouteMatcher {
public:
  /**
   * Form a path by matching shape with graph edges (edge walking). Also sets the path_edges
   * on the appropriate locations in the shape so that trip leg builder will have that info
   *
   * @param mode_costing                     Dynamic costing methods used to determine allowed edges
   * and costs.
   * @param mode                             Travel mode (indexes the costing methods).
   * @param reader                           Access to the tiled graph data
   * @param options                          proto object containing the shape and correlated start
   * and end location
   * @param whether to use timestamps        proto object containing the shape and correlated start
   * and end location
   * @param legs                             Vector of vector of path_info, edge-wise cost/time
   * information
   * @return Returns true if the edge walk forms a path, false if shape does not match.
   */
  template <typename Options>
  static bool FormPath(const sif::mode_costing_t& mode_costing,
                       const sif::TravelMode& mode,
                       baldr::GraphReader& reader,
                       Options& options,
                       const bool use_timestamps,
                       const bool use_shortcuts,
                       std::vector<std::vector<PathInfo>>& legs);
};

extern template bool RouteMatcher::FormPath(const sif::mode_costing_t&,
                                            const sif::TravelMode&,
                                            baldr::GraphReader&,
                                            valhalla::Options&,
                                            const bool,
                                            const bool,
                                            std::vector<std::vector<PathInfo>>&);

extern template bool RouteMatcher::FormPath(const sif::mode_costing_t&,
                                            const sif::TravelMode&,
                                            baldr::GraphReader&,
                                            valhalla::LinearFeatureCost&,
                                            const bool,
                                            const bool,
                                            std::vector<std::vector<PathInfo>>&);

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ROUTE_MATCHER_H_
