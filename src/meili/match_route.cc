#include <cassert>
#include <vector>

#include "meili/geometry_helpers.h"
#include "meili/map_matcher.h"
#include "midgard/logging.h"

namespace {

using namespace valhalla;
using namespace valhalla::meili;

// Check if all edge segments of the route are successive, and contain no loops
bool ValidateRoute(baldr::GraphReader& graphreader,
                   std::vector<EdgeSegment>::const_iterator segment_begin,
                   std::vector<EdgeSegment>::const_iterator segment_end,
                   graph_tile_ptr& tile) {
  if (segment_begin == segment_end) {
    return true;
  }

  for (auto prev_segment = segment_begin, segment = std::next(segment_begin); segment != segment_end;
       prev_segment = segment, segment++) {

    // Successive segments must be adjacent and no loops
    if (prev_segment->edgeid == segment->edgeid) {
      if (prev_segment->target != segment->source) {
        LOG_ERROR("CASE 1: Found disconnected segments at " +
                  std::to_string(segment - segment_begin));
        LOG_ERROR(static_cast<std::stringstream&&>(std::stringstream() << *prev_segment).str());
        LOG_ERROR(static_cast<std::stringstream&&>(std::stringstream() << *segment).str());
        return false;
      }
    } else {
      // Make sure edges are connected (could be on different levels)
      if (!(prev_segment->target == 1.f && segment->source == 0.f &&
            graphreader.AreEdgesConnectedForward(prev_segment->edgeid, segment->edgeid, tile))) {
        LOG_ERROR("CASE 2: Found disconnected segments at " +
                  std::to_string(segment - segment_begin));
        LOG_ERROR(static_cast<std::stringstream&&>(std::stringstream() << *prev_segment).str());
        LOG_ERROR(static_cast<std::stringstream&&>(std::stringstream() << *segment).str());
        return false;
      }
    }
  }
  return true;
}

}; // namespace

namespace valhalla {
namespace meili {

EdgeSegment::EdgeSegment(baldr::GraphId the_edgeid,
                         double the_source,
                         double the_target,
                         int the_first_match_idx,
                         int the_last_match_idx,
                         bool disconnect,
                         int the_restriction_idx)
    : edgeid(the_edgeid), source(the_source), target(the_target),
      first_match_idx(the_first_match_idx), last_match_idx(the_last_match_idx),
      discontinuity(disconnect), restriction_idx(the_restriction_idx) {
  if (!edgeid.Is_Valid()) {
    throw std::invalid_argument("Invalid edgeid");
  }

  if (!(0 <= source && source <= target && target <= 1)) {
    throw std::invalid_argument("Expect 0 <= source <= target <= 1, but you got source = " +
                                std::to_string(source) + " and target = " + std::to_string(target));
  }
}

/**
 * Here we return the vector of edge segments between the source and target states. If its a node to
 * node route (meaning no realy edge is traversed) then we use the target_result to say what edge the
 * segment should use
 *
 * TODO: we should modify this function to take the range of MatchResults between the source and
 *  target. we should then use those to property initialize the segments source/target/indices to
 *  match those of the MatchResults that lie on them rather than doing it downstream in an otherwise
 *  superfluous operation
 * @param source         source state to use to find the route
 * @param target         target state which candidate in the next column to fetch the route for
 * @param route          a place to put the edge segments as we create them
 * @param target_result  in case we have a node to node route we have a no-op edge segment to return
 * @return  the vector of segments representing the route between source and target
 */
bool MergeRoute(const State& source,
                const State& target,
                std::vector<EdgeSegment>& route,
                const MatchResult& target_result) {
  // Discontinuity either from invalid state id or no paths on either side of this states candidates
  const auto route_rbegin = source.RouteBegin(target), route_rend = source.RouteEnd();
  if (route_rbegin == route_rend) {
    return false;
  }

  // Here we dont iterate to the very last label (first edge of the route) because it is either
  // a dummy (if source and target are the 0th and 1st states) or because its the last label of
  // the previous pair of states
  std::vector<EdgeSegment> segments;
  auto label = route_rbegin;
  for (; std::next(label) != route_rend; label++) {
    segments.emplace_back(label->edgeid(), label->source(), label->target(), -1, -1, false,
                          label->restriction_idx());
  }

  // If we looped all the way to the beginning then there should be no previous labels
  assert(label->predecessor() == baldr::kInvalidLabel);

  // TODO: why doesnt routing.cc return trivial routes? move this logic there
  // This is route where the source and target are the same location so we make a trivial route
  if (segments.empty()) {
    assert(target_result.edgeid.Is_Valid());
    segments.emplace_back(target_result.edgeid, target_result.distance_along,
                          target_result.distance_along);
  }

  route.insert(route.end(), segments.crbegin(), segments.crend());
  return true;
}

/**
 * We need this method to cut segments at interpolated input trace points which were marked as break
 * points. This method will split edge segments at those points.
 * @param match_results  The matched points representing the matched input trace points
 * @param first_idx      The index of the first match in the range
 * @param last_idx       The index of the second match in the range
 * @param segments       The segments over the range
 * @param new_segments   The new vector of segments over the range (should be the same or more
 * segments). This is a reference parameter to avoid constant allocation
 */
void cut_segments(const std::vector<MatchResult>& match_results,
                  int first_idx,
                  int last_idx,
                  std::vector<EdgeSegment>& segments,
                  std::vector<EdgeSegment>& new_segments) {
  auto first_segment = segments.begin();
  auto search_segment = first_segment;
  int prev_idx = first_idx;
  for (int curr_idx = first_idx + 1; curr_idx <= last_idx; ++curr_idx) {
    const MatchResult& curr_match = match_results[curr_idx];
    const MatchResult& prev_match = match_results[prev_idx];

    // does the previous match point look further along its edge than the current
    bool prev_gt_curr = prev_match.distance_along > curr_match.distance_along + 1e-3;

    // if they are on the same edge and previous comes after then its a loop
    bool loop = (prev_match.edgeid == curr_match.edgeid) && prev_gt_curr;

    // if it is a loop, we start the search after the first edge
    for (search_segment += static_cast<size_t>(loop); search_segment != segments.end();
         ++search_segment) {
      if (search_segment->edgeid == curr_match.edgeid)
        break;
    }

    if (search_segment == segments.cend()) {
      throw std::logic_error("In meili::cutsegments(), unexpectedly unable to locate target edge.");
    }

    // if its not a break point nor the last segment, then we dont need to split it into two
    if (!curr_match.is_break_point && curr_idx != last_idx) {
      // but if it isnt a breakpoint and its not the last stateful match point then its interpolated
      // and we still need to mark the segments in such a way that we which match points lie on them
      if (search_segment->first_match_idx < 0)
        search_segment->first_match_idx = curr_idx;
      if (search_segment->last_match_idx < 0)
        search_segment->last_match_idx = curr_idx;
      // we are done marking it no need to split this one
      continue;
    }

    // we need to close the previous edge
    size_t old_size = new_segments.size();
    new_segments.insert(new_segments.cend(), first_segment, search_segment + 1);
    new_segments[old_size].first_match_idx = prev_idx;
    // when the points got interpolated, we want to use the match results' distance along
    // otherwise, we use the segment's source or target because if it is a node snap, the
    // match result can only hold one candidate, we need either side of the node.
    new_segments[old_size].source =
        prev_match.HasState() ? first_segment->source : prev_match.distance_along;
    new_segments.back().last_match_idx = curr_idx;
    new_segments.back().target =
        curr_match.HasState() ? search_segment->target : curr_match.distance_along;

    first_segment = search_segment;
    prev_idx = curr_idx;
  }
}

/**
 * This loops over all of the states in the match and returns the vector of edge segments representing
 * the matched path.
 * @param mapmatcher     The matcher with which the match was computed
 * @param match_results  The matched points
 * @return  The vector of edge segments representing the match path
 */
std::vector<EdgeSegment> ConstructRoute(const MapMatcher& mapmatcher,
                                        const std::vector<MatchResult>& match_results) {
  if (match_results.empty()) {
    return {};
  }

  std::vector<EdgeSegment> route;
  graph_tile_ptr tile;

  // Merge segments into route
  // std::deque<int> match_indices;
  std::vector<EdgeSegment> segments;
  std::vector<EdgeSegment> new_segments;
  const MatchResult* prev_match{nullptr};
  int prev_idx = -1;
  for (int curr_idx = 0, n = static_cast<int>(match_results.size()); curr_idx < n; ++curr_idx) {
    const MatchResult& match = match_results[curr_idx];

    // unmatched or interpolated
    if (!match.edgeid.Is_Valid() || !match.HasState()) {
      continue;
    }

    if (prev_match && prev_match->HasState()) {
      const auto &prev_state = mapmatcher.state_container().state(prev_match->stateid),
                 state = mapmatcher.state_container().state(match.stateid);

      // get the route between the two states by walking edge labels backwards
      // then reverse merge the segments together which are on the same edge so we have a
      // minimum number of segments. in this case we could at minimum end up with 1 segment
      segments.clear();
      if (!MergeRoute(prev_state, state, segments, match)) {
        // we are only discontinuous if this isnt the beginning of the route
        if (!route.empty())
          route.back().discontinuity = true;
        // next pair
        prev_idx = curr_idx;
        prev_match = &match;
        continue;
      }

      // we need to cut segments where a match result is marked as a break
      new_segments.clear();
      cut_segments(match_results, prev_idx, curr_idx, segments, new_segments);

      // have to merge route's last segment and segments' first segment together if its not a
      // discontinuity or a break
      if (!prev_match->is_break_point && !route.empty() && !route.back().discontinuity &&
          route.back().edgeid == new_segments.front().edgeid) {
        // we modify the first segment of the new_segments accordingly to replace the previous
        // one in the route.
        new_segments.front().source = route.back().source;
        // Prefer first_match_idx from previous segments but do not replace valid value with invalid.
        if (route.back().first_match_idx != -1)
          new_segments.front().first_match_idx = route.back().first_match_idx;
        // Prefer last_match_idx from new segments but do not replace valid value with invalid.
        if (new_segments.front().last_match_idx == -1)
          new_segments.front().last_match_idx = route.back().last_match_idx;
        route.pop_back();
      }

      // debug builds check that the route is valid
      assert(ValidateRoute(mapmatcher.graphreader(), new_segments.begin(), new_segments.end(), tile));

      // after we figured out whether or not we need to merge the last one we keep the rest
      route.insert(route.end(), new_segments.cbegin(), new_segments.cend());
    }

    prev_match = &match;
    prev_idx = curr_idx;
  }

  return route;
}

} // namespace meili
} // namespace valhalla
