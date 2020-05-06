#include "midgard/logging.h"
#include <algorithm>
#include <exception>
#include <vector>

#include "baldr/datetime.h"
#include "thor/map_matcher.h"
#include "thor/worker.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {
struct interpolation_t {
  valhalla::baldr::GraphId edge; // edge id
  float total_distance;          // distance along the path
  float edge_distance;           // ratio of the distance along the edge
  size_t original_index;         // index into the original measurements
  double epoch_time;             // seconds from epoch
};

static uint32_t compute_origin_epoch(const std::vector<valhalla::meili::EdgeSegment>& edge_segments,
                                     valhalla::meili::MapMatcher* matcher,
                                     valhalla::Options& options) {
  const GraphTile* tile = nullptr;
  const DirectedEdge* directededge = nullptr;
  const NodeInfo* nodeinfo = nullptr;

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  uint32_t origin_epoch = 0;
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
      // if its timestamp based need to signal that out to trip leg builder
      if (!options.shape(0).has_date_time() && options.shape(0).time() != -1.0) {
        options.mutable_shape(0)->set_date_time(
            DateTime::seconds_to_date(options.shape(0).time(), tz, false));
      }
      // remember where we are starting
      if (options.shape(0).has_date_time()) {
        if (options.shape(0).date_time() == "current")
          options.mutable_shape(0)->set_date_time(DateTime::iso_date_time(tz));
        origin_epoch = DateTime::seconds_since_epoch(options.shape(0).date_time(), tz);
      }
      break;
    }
  }

  return origin_epoch;
}
static std::vector<std::vector<interpolation_t>>
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
                     const std::shared_ptr<sif::DynamicCost>* mode_costing,
                     const sif::TravelMode mode,
                     Options& options) {

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  uint32_t origin_epoch = compute_origin_epoch(edge_segments, matcher, options);
  std::string date_time = options.shape(0).has_date_time() ? options.shape(0).date_time() : "";

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
  Cost elapsed;
  std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>> paths;
  GraphId prior_edge, prior_node;
  EdgeLabel pred;
  const meili::EdgeSegment* prev_segment = nullptr;
  const GraphTile* tile = nullptr;
  const DirectedEdge* directededge = nullptr;
  const NodeInfo* nodeinfo = nullptr;

  // Build the path
  size_t interpolated_index = 0;
  size_t num_segments = edge_segments.size();
  Cost accumulated_elapsed;
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
    // if this is the first route or the first edge after the discontinuity or
    // user requested a new leg here
    if (new_leg) {
      if (!date_time.empty() && prev_segment) {
        date_time = thor_worker_t::offset_date(matcher->graphreader(), date_time,
                                               paths.back().first.front().edgeid, elapsed.secs,
                                               paths.back().first.back().edgeid);
        options.mutable_shape(edge_segment.first_match_idx)->set_date_time(date_time);
      }

      paths.emplace_back();
      paths.back().first.reserve(num_segments);
      paths.back().second.reserve(num_segments);

      accumulated_elapsed += elapsed;
      elapsed = {};
    }

    // Get seconds from beginning of the week accounting for any changes to timezone on the path
    uint32_t second_of_week = kInvalidSecondsOfWeek;
    if (origin_epoch != 0 && nodeinfo) {
      second_of_week =
          DateTime::second_of_week(origin_epoch +
                                       static_cast<uint32_t>(accumulated_elapsed.secs + elapsed.secs),
                                   DateTime::get_tz_db().from_index(nodeinfo->timezone()));
    }

    // get the cost of traversing the node, there is no turn cost the first time
    Cost transition_cost{};
    if (elapsed.secs > 0) {
      transition_cost = costing->TransitionCost(directededge, nodeinfo, pred);
      elapsed += transition_cost;
    }

    // Get time along the edge, handling partial distance along the first and last edge.
    elapsed += costing->EdgeCost(directededge, tile, second_of_week) *
               (edge_segment.target - edge_segment.source);

    if (use_timestamps) {
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
      elapsed.secs = results[idx].epoch_time - results[0].epoch_time;
    }

    // Update the predecessor EdgeLabel (for transition costing in the next round);
    pred = {kInvalidLabel, edge_id, directededge, elapsed, 0, 0, mode, 0, {}};

    paths.back().first.emplace_back(
        PathInfo{mode, elapsed.secs, edge_id, 0, elapsed.cost, false, transition_cost.secs});
    paths.back().second.emplace_back(&edge_segment);
    --num_segments;

    // Update the prior_edge and nodeinfo. TODO (protect against invalid tile)
    prev_segment = &edge_segment;
    prior_edge = edge_id;
    prior_node = directededge->endnode();
    const GraphTile* end_tile = matcher->graphreader().GetGraphTile(prior_node);
    nodeinfo = end_tile->node(prior_node);
  }

  return paths;
}

} // namespace thor
} // namespace valhalla
