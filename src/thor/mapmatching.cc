#include <vector>
#include <algorithm>
#include <exception>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/errorcode_util.h>

#include "thor/mapmatching.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::meili;

namespace valhalla {
namespace thor {

// Form the path from the map-matching results. This path gets sent to
// TripPathBuilder.
std::vector<PathInfo> MapMatching::FormPath(
    MapMatcher* matcher, const std::vector<meili::MatchResult>& results,
    const std::shared_ptr<sif::DynamicCost>* mode_costing,
    const sif::TravelMode mode) {
  // Set the mode and costing
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  // Iterate through the matched path. Form PathInfo - populate elapsed time
  // Return an empty path (or throw exception) if path is not connected.
  float elapsed_time = 0;
  std::vector<PathInfo> path;
  GraphId prior_edge, prior_node;
  const NodeInfo* nodeinfo = nullptr;
  const DirectedEdge* directededge;
  EdgeLabel pred;

  try {
    auto edge_segments = ConstructRoute(matcher->mapmatching(), results.begin(),
                                        results.end());
    for (const auto& edge_segment : edge_segments) {
      // Skip edges that are the same as the prior edge
      if (edge_segment.edgeid == prior_edge) {
        continue;
      }

      // Get the directed edge
      GraphId edge_id = edge_segment.edgeid;
      const GraphTile* tile = matcher->graphreader().GetGraphTile(edge_id);
      directededge = tile->directededge(edge_id);

      // TODO: slight difference in time between route and trace_route
      if (nodeinfo) {
        // Get transition cost
        elapsed_time += costing->TransitionCost(directededge, nodeinfo, pred)
                .secs;

        // Get time along the edge, handling partial distance along
        // the first and last edge
        elapsed_time += costing->EdgeCost(directededge).secs
            * (edge_segment.target - edge_segment.source);
      } else {
        // Get time along the edge, handling partial distance along
        // the first and last edge
        elapsed_time += costing->EdgeCost(directededge).secs
                * (edge_segment.target - edge_segment.source);
      }


      // Update the prior_edge and nodeinfo. TODO (protect against invalid tile)
      prior_edge = edge_id;
      prior_node = directededge->endnode();
      const GraphTile* end_tile = matcher->graphreader().GetGraphTile(
          prior_node);
      nodeinfo = end_tile->node(prior_node);

      // Create a predecessor EdgeLabel (for transition costing)
      pred = {kInvalidLabel, edge_id, directededge, {}, 0, 0, mode, 0};

      // Add to the PathInfo
      path.emplace_back(mode, elapsed_time, edge_id, 0);
    }
  }
  catch(const std::exception& e) {
    throw valhalla_exception_t { 400, 442 };
  }
  return path;
}

}
}
