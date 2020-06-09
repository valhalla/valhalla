#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/proto/trip.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

// Allows you to trim shape for when you need to cut an edge for a discontinuity in map matching
// or for a uturn in the middle of the edge
struct EdgeTrimmingInfo {
  bool trim;
  midgard::PointLL vertex;
  float distance_along;
};

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripLegBuilder {
public:
  /**
   * Form a trip leg out of a path (sequence of path infos)
   *
   * @param controller            Which meta data attributes to include in the trip leg
   * @param graphreader           A way of accessing graph information
   * @param mode_costing          A costing object
   * @param path_begin            The first path info in the path
   * @param path_end              One past the last path info in the path
   * @param origin                The origin location with path edges filled in from loki
   * @param dest                  The destination location with path edges filled in from loki
   * @param through_loc           The list of through locations along this leg if any
   * @param trip_path             The leg we will fill out
   * @param interrupt_callback    A way to abort the processing in case the request was cancelled
   * @param edge_trimming         Markers on edges with information on how to trim their shape
   * @param trim_begin            For map matching we have one long sequence of path infos regardless
   *                              of legs so we must supply an amount of elapsed time which we trim
   *                              from the beginning
   * @param trim_end              Similarly to trim_begin, we must also trim at the end of a map
   *                              matched edge
   * @return
   */
  static void Build(const AttributesController& controller,
                    baldr::GraphReader& graphreader,
                    const std::shared_ptr<sif::DynamicCost>* mode_costing,
                    const std::vector<PathInfo>::const_iterator path_begin,
                    const std::vector<PathInfo>::const_iterator path_end,
                    valhalla::Location& origin,
                    valhalla::Location& dest,
                    const std::list<valhalla::Location>& through_loc,
                    TripLeg& trip_path,
                    const std::function<void()>* interrupt_callback = nullptr,
                    std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>>*
                        edge_trimming = nullptr,
                    float trim_begin = 0,
                    float trim_end = 0);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TRIPPATHBUILDER_H_
