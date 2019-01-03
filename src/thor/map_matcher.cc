#include "midgard/logging.h"
#include <algorithm>
#include <exception>
#include <vector>

#include "thor/map_matcher.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

struct interpolation_t {
  baldr::GraphId edge;   // edge id
  float total_distance;  // distance along the path
  float edge_distance;   // ratio of the distance along the edge
  size_t original_index; // index into the original measurements
  double epoch_time;     // seconds from epoch
};

std::list<std::vector<interpolation_t>>
interpolate_matches(const std::vector<meili::MatchResult>& matches,
                    const std::vector<meili::EdgeSegment>& edges,
                    meili::MapMatcher* matcher) {
  // TODO: backtracking could have happened. maybe it really happened but maybe there were
  // positional inaccuracies. for now we should detect when there are backtracks and give up
  // otherwise the the timing reported here might be suspect
  // find each set of continuous edges
  std::list<std::vector<interpolation_t>> interpolations;
  size_t idx = 0;
  for (auto begin_edge = edges.cbegin(), end_edge = edges.cbegin() + 1; begin_edge != edges.cend();
       begin_edge = end_edge, end_edge += 1) {
    // find the end of the this block
    while (end_edge != edges.cend()) {
      if (!matcher->graphreader().AreEdgesConnectedForward(std::prev(end_edge)->edgeid,
                                                           end_edge->edgeid)) {
        break;
      }
      ++end_edge;
    }

    // go through each edge and each match keeping the distance each point is along the entire trace
    std::vector<interpolation_t> interpolated;
    size_t last_idx = idx;
    for (auto segment = begin_edge; segment != end_edge; ++segment) {
      float edge_length = matcher->graphreader()
                              .GetGraphTile(segment->edgeid)
                              ->directededge(segment->edgeid)
                              ->length();
      float total_length = segment == begin_edge ? -edges.front().source * edge_length
                                                 : interpolated.back().total_distance;
      // get the distance and match result for the begin node of the edge
      interpolated.emplace_back(interpolation_t{segment->edgeid, total_length, 0.f, last_idx, -1});

      // add distances for all the match points that happened on this edge
      for (; idx < matches.size(); ++idx) {
        // skip unroutable ones, we dont know what edge they were on
        if (!matches[idx].edgeid.Is_Valid()) {
          continue;
          // if its a valid one that doesnt match we move on
        } else if (matches[idx].edgeid != segment->edgeid) {
          break;
        }
        // it was the right thing we were looking for
        interpolated.emplace_back(
            interpolation_t{segment->edgeid, matches[idx].distance_along * edge_length + total_length,
                            matches[idx].distance_along, idx, matches[idx].epoch_time});
        last_idx = idx;
      }
      // add the end node of the edge
      interpolated.emplace_back(
          interpolation_t{segment->edgeid, edge_length + total_length, 1.f, last_idx, -1});
    }

    // finally backfill the time information for those points that dont have it
    auto backfill = interpolated.begin();
    while (backfill != interpolated.end()) {
      // this one is done already
      if (backfill->epoch_time != -1) {
        ++backfill;
        continue;
      }
      // find the range that have values (or the ends of the range
      auto left = backfill != interpolated.begin() ? std::prev(backfill) : backfill;
      auto right = std::next(backfill);
      for (; right != interpolated.end() && right->epoch_time == -1; ++right) {
        ;
      }
      // backfill between left and right
      while (backfill != right) {
        // if both indices are valid we interpolate
        if (left != interpolated.begin() && right != interpolated.end()) {
          double time_diff = right->epoch_time - left->epoch_time;
          float distance_diff = right->total_distance - left->total_distance;
          float distance_ratio =
              distance_diff > 0 ? (backfill->total_distance - left->total_distance) / distance_diff
                                : 0.f;
          backfill->epoch_time = left->epoch_time + distance_ratio * time_diff;
        } // if left index is valid we carry it forward if we can
        else if (left != interpolated.begin() && backfill->total_distance == left->total_distance) {
          backfill->epoch_time = left->epoch_time;
          backfill->original_index = left->original_index;
        }
        // right index is valid we carry it forward if we can
        else if (right != interpolated.end() && backfill->total_distance == right->total_distance) {
          backfill->epoch_time = right->epoch_time;
          backfill->original_index = right->original_index;
        }
        // next backfill
        ++backfill;
      }
    }

    // keep this set of interpolations
    interpolations.emplace_back(std::move(interpolated));
  }

  // give back the distances and updated match results
  return interpolations;
}

// Form the path from the map-matching results. This path gets sent to
// TripPathBuilder.
std::vector<PathInfo>
MapMatcher::FormPath(meili::MapMatcher* matcher,
                     const std::vector<meili::MatchResult>& results,
                     const std::vector<meili::EdgeSegment>& edge_segments,
                     const std::shared_ptr<sif::DynamicCost>* mode_costing,
                     const sif::TravelMode mode,
                     std::vector<std::pair<GraphId, GraphId>>& disconnected_edges,
                     const bool use_timestamps) {
  // Set costing based on the mode
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];

  // Iterate through the matched path. Form PathInfo - populate elapsed time
  // Return an empty path (or throw exception) if path is not connected.
  float elapsed_time = 0;
  std::vector<PathInfo> path;
  GraphId prior_edge, prior_node;
  const NodeInfo* nodeinfo = nullptr;
  const DirectedEdge* directededge;
  EdgeLabel pred;

  // Interpolate match results if using timestamps for elapsed time
  bool use_costing = !use_timestamps;
  std::list<std::vector<interpolation_t>> interpolations;
  size_t interpolated_index = 0;
  size_t last_interp_index = 0;
  if (use_timestamps) {
    interpolations = interpolate_matches(results, edge_segments, matcher);
    if (interpolations.size() > 1) {
      // This means there is a discontinuity. If so, fallback to using costing
      use_costing = true;
    }
    last_interp_index = interpolations.front().back().original_index;
  }

  for (const auto& edge_segment : edge_segments) {

    // Skip edges that are the same as the prior edge
    if (edge_segment.edgeid == prior_edge) {
      continue;
    }

    // Get the directed edge
    GraphId edge_id = edge_segment.edgeid;
    const GraphTile* tile = matcher->graphreader().GetGraphTile(edge_id);
    directededge = tile->directededge(edge_id);

    // Check if connected to prior edge
    if (prior_edge.Is_Valid() && !matcher->graphreader().AreEdgesConnected(prior_edge, edge_id)) {
      disconnected_edges.emplace_back(prior_edge, edge_id);
    }

    if (use_costing) {
      // TODO: slight difference in time between route and trace_route
      if (nodeinfo) {
        // Get time along the edge, handling partial distance along the first and last edge.
        // Add the transition cost at the begin node.
        elapsed_time += costing->EdgeCost(directededge, tile->GetSpeed(directededge)).secs *
                            (edge_segment.target - edge_segment.source) +
                        costing->TransitionCost(directededge, nodeinfo, pred).secs;
      } else {
        // Get time along the edge, handling partial distance along the first and last edge
        elapsed_time += costing->EdgeCost(directededge, tile->GetSpeed(directededge)).secs *
                        (edge_segment.target - edge_segment.source);
      }
    } else {
      // Use timestamps to update elapsed time. Use the timestamp at the interpolation
      // that no longer matches the edge_id (or the last interpolation if the edge id
      // matches the rest of the interpolations).
      size_t idx = last_interp_index;
      for (size_t i = interpolated_index; i < interpolations.front().size(); ++i) {
        if (interpolations.front()[i].edge != edge_id) {
          // Set the index into the match results, update the interpolated index to
          // start search for next edge id
          idx = interpolations.front()[i].original_index;
          interpolated_index = i;
          break;
        }
      }
      elapsed_time = results[idx].epoch_time - results[0].epoch_time;
    }

    // Update the prior_edge and nodeinfo. TODO (protect against invalid tile)
    prior_edge = edge_id;
    prior_node = directededge->endnode();
    const GraphTile* end_tile = matcher->graphreader().GetGraphTile(prior_node);
    nodeinfo = end_tile->node(prior_node);

    // Create a predecessor EdgeLabel (for transition costing)
    pred = {kInvalidLabel, edge_id, directededge, {}, 0, 0, mode, 0};

    // Add to the PathInfo
    path.emplace_back(mode, elapsed_time, edge_id, 0);
  }

  return path;
}

} // namespace thor
} // namespace valhalla
