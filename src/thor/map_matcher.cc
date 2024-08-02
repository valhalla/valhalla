#include "midgard/logging.h"
#include <algorithm>
#include <vector>

#include "baldr/datetime.h"
#include "baldr/time_info.h"
#include "thor/map_matcher.h"
#include "thor/worker.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {
struct interpolation_t {
  valhalla::baldr::GraphId edge; // edge id
  double total_distance;         // distance along the path
  double edge_distance;          // ratio of the distance along the edge
  size_t original_index;         // index into the original measurements
  double epoch_time;             // seconds from epoch
};

valhalla::baldr::TimeInfo
init_time_info(const std::vector<valhalla::meili::EdgeSegment>& edge_segments,
               valhalla::meili::MapMatcher* matcher,
               valhalla::Options& options,
               valhalla::baldr::DateTime::tz_sys_info_cache_t* tz_cache) {
  graph_tile_ptr tile = nullptr;
  const DirectedEdge* directededge = nullptr;
  const NodeInfo* nodeinfo = nullptr;

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  for (const auto& s : edge_segments) {
    if (!s.edgeid.Is_Valid() || !matcher->graphreader().GetGraphTile(s.edgeid, tile))
      continue;
    directededge = tile->directededge(s.edgeid);
    if (matcher->graphreader().GetGraphTile(directededge->endnode(), tile)) {
      // get the timezone
      nodeinfo = tile->node(directededge->endnode());
      const auto* tz = DateTime::get_tz_db().from_index(nodeinfo->timezone());
      if (!tz)
        continue;
      // if its timestamp based we need to convert that to a date time string on the location
      if (options.shape(0).date_time().empty() && options.shape(0).time() != -1.0) {
        options.mutable_shape(0)->set_date_time(
            DateTime::seconds_to_date(options.shape(0).time(), tz, false));
      }
      return valhalla::baldr::TimeInfo::make(*options.mutable_shape(0), matcher->graphreader(),
                                             tz_cache, nodeinfo->timezone());
    }
  }
  return valhalla::baldr::TimeInfo::invalid();
}

std::vector<std::vector<interpolation_t>>
interpolate_matches(const std::vector<valhalla::meili::MatchResult>& matches,
                    const std::vector<valhalla::meili::EdgeSegment>& edges,
                    valhalla::meili::MapMatcher* matcher) {
  // TODO: backtracking could have happened. maybe it really happened but maybe there were
  // positional inaccuracies. for now we should detect when there are backtracks and give up
  // otherwise the the timing reported here might be suspect
  // find each set of continuous edges
  std::vector<std::vector<interpolation_t>> interpolations;
  interpolations.reserve(edges.size());
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
    interpolated.reserve(matches.size());
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

} // namespace

namespace valhalla {
namespace thor {

// We can return multiple paths from here. Any time a MatchResult (trace point) is marked as a
// break point or break_through point we will make a new leg and hence a new vector of path infos
// also whenever there is a discontinuity there will be a new vector of path infos created
std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>
MapMatcher::FormPath(meili::MapMatcher* matcher,
                     const std::vector<meili::MatchResult>& results,
                     const std::vector<meili::EdgeSegment>& edge_segments,
                     const sif::mode_costing_t& mode_costing,
                     const sif::TravelMode mode,
                     Options& options) {

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  valhalla::baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto time_info = init_time_info(edge_segments, matcher, options, &tz_cache);

  // Interpolate match results if using timestamps for elapsed time
  std::vector<std::vector<interpolation_t>> interpolations;
  size_t last_interp_index = 0;
  bool use_timestamps = options.use_timestamps();
  if (use_timestamps) {
    interpolations = interpolate_matches(results, edge_segments, matcher);
    // This means there is a discontinuity. If so, fallback to using costing
    if (interpolations.size() != 1)
      use_timestamps = false;
    else
      last_interp_index = interpolations.front().back().original_index;
  }

  // Set costing based on the mode
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  // Iterate through the matched path. Form PathInfo - populate elapsed time
  // Return an empty path (or throw exception) if path is not connected.
  Cost elapsed{};
  Cost accumulated_elapsed{};
  std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>> paths;
  GraphId prior_node;
  EdgeLabel pred;
  const meili::EdgeSegment* prev_segment = nullptr;
  graph_tile_ptr tile = nullptr;
  const DirectedEdge* directededge = nullptr;
  const NodeInfo* nodeinfo = nullptr;

  // Build the path
  size_t interpolated_index = 0;
  size_t num_segments = edge_segments.size();
  for (const auto& edge_segment : edge_segments) {

    // Get the directed edge
    GraphId edge_id = edge_segment.edgeid;
    matcher->graphreader().GetGraphTile(edge_id, tile);
    directededge = tile->directededge(edge_id);

    // Check if connected to prior edge
    bool disconnected =
        prev_segment != nullptr && prev_segment->edgeid.Is_Valid() && prev_segment->discontinuity;

    bool break_point =
        edge_segment.first_match_idx >= 0 && results[edge_segment.first_match_idx].is_break_point;
    bool new_leg = disconnected || !prev_segment || break_point;

    // Figure out what time it is right now, the first iteration is a no-op
    auto offset_time_info =
        nodeinfo ? time_info.forward(accumulated_elapsed.secs + elapsed.secs, nodeinfo->timezone())
                 : time_info;

    // if this is the first leg or the first edge after a discontinuity or a requested break point
    if (new_leg) {
      // set the date on the leg if we need to
      if (offset_time_info.valid && prev_segment) {
        options.mutable_shape(edge_segment.first_match_idx)
            ->set_date_time(offset_time_info.date_time());
      }

      // make a new leg
      paths.emplace_back();
      paths.back().first.reserve(num_segments);
      paths.back().second.reserve(num_segments);

      // remember the total elapse time so far
      accumulated_elapsed += elapsed;
      elapsed = {};
    }

    // get the cost of traversing the node, there is no turn cost the first time
    Cost transition_cost{};
    if (elapsed.secs > 0) {
      transition_cost = costing->TransitionCost(directededge, nodeinfo, pred);
      elapsed += transition_cost;
    }

    uint8_t flow_sources;
    // Get time along the edge, handling partial distance along the first and last edge.
    elapsed += costing->EdgeCost(directededge, tile, offset_time_info, flow_sources) *
               (edge_segment.target - edge_segment.source);

    // Use timestamps to update elapsed time. Use the timestamp at the interpolation
    // that no longer matches the edge_id (or the last interpolation if the edge id
    // matches the rest of the interpolations).
    if (use_timestamps) {
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
      elapsed.secs = results[idx].epoch_time - results[0].epoch_time;
    }

    InternalTurn turn = nodeinfo ? costing->TurnType(pred.opp_local_idx(), nodeinfo, directededge)
                                 : InternalTurn::kNoTurn;

    // Update the predecessor EdgeLabel (for transition costing in the next round);
    pred = {kInvalidLabel,
            edge_id,
            directededge,
            elapsed,
            0,
            mode,
            0,
            baldr::kInvalidRestriction,
            true,
            static_cast<bool>(flow_sources & kDefaultFlowMask),
            turn,
            0,
            directededge->destonly() || (costing->is_hgv() && directededge->destonly_hgv())};
    paths.back().first.emplace_back(
        PathInfo{mode, elapsed, edge_id, 0, 0, edge_segment.restriction_idx, transition_cost});
    paths.back().second.emplace_back(&edge_segment);
    --num_segments;

    // Update the prior_edge and nodeinfo. TODO (protect against invalid tile)
    prev_segment = &edge_segment;
    prior_node = directededge->endnode();
    graph_tile_ptr end_tile = matcher->graphreader().GetGraphTile(prior_node);
    nodeinfo = end_tile->node(prior_node);
  }

  return paths;
}

} // namespace thor
} // namespace valhalla
