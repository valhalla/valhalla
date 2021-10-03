#pragma once

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
#include <valhalla/proto_conversions.h>
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
  double distance_along;
};

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripLegBuilder {
public:
  /**
   * Form a trip leg out of a path (sequence of path infos)
   *
   * @param options               Request options
   * @param controller            Which meta data attributes to include in the trip leg
   * @param graphreader           A way of accessing graph information
   * @param mode_costing          A costing object
   * @param path_begin            The first path info in the path
   * @param path_end              One past the last path info in the path
   * @param origin                The origin location with path edges filled in from loki
   * @param dest                  The destination location with path edges filled in from loki
   * @param trip_path             The leg we will fill out
   * @param alagorithms           The list of graph search algorithm names used to create the path
   * @param interrupt_callback    A way to abort the processing in case the request was cancelled
   * @param edge_trimming         Markers on edges with information on how to trim their shape
   * @param intermediate          The locations between the origin and dest (through or via types)
   * @return
   */
  static void Build(const valhalla::Options& options,
                    const AttributesController& controller,
                    baldr::GraphReader& graphreader,
                    const sif::mode_costing_t& mode_costing,
                    const std::vector<PathInfo>::const_iterator path_begin,
                    const std::vector<PathInfo>::const_iterator path_end,
                    valhalla::Location& origin,
                    valhalla::Location& dest,
                    TripLeg& trip_path,
                    const std::vector<std::string>& algorithms,
                    const std::function<void()>* interrupt_callback = nullptr,
                    const std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>>&
                        edge_trimming = {},
                    const std::vector<valhalla::Location>& intermediates = {});
};

} // namespace thor
} // namespace valhalla
