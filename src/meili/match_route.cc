#include <cassert>
#include <vector>

#include "meili/geometry_helpers.h"
#include "meili/map_matcher.h"
#include "midgard/logging.h"

namespace {

using namespace valhalla;
using namespace valhalla::meili;

template <typename segment_iterator_t>
std::string RouteToString(baldr::GraphReader& graphreader,
                          segment_iterator_t segment_begin,
                          segment_iterator_t segment_end,
                          const baldr::GraphTile*& tile) {
  // The string will look like: [dummy] [source/startnodeid edgeid target/endnodeid] ...
  std::ostringstream route;

  for (auto segment = segment_begin; segment != segment_end; segment++) {
    if (segment->edgeid.Is_Valid()) {
      route << "[";

      // Get the end nodes of the directed edge.
      auto edge_nodes = graphreader.GetDirectedEdgeNodes(segment->edgeid, tile);
      if (segment->source == 0.f) {
        const auto startnodeid = edge_nodes.first;
        if (startnodeid.Is_Valid()) {
          route << startnodeid;
        } else {
          route << "InvalidId";
        }
      } else {
        route << segment->source;
      }

      if (segment->edgeid.Is_Valid()) {
        route << " " << segment->edgeid << " ";
      } else {
        route << " "
              << "InvalidId"
              << " ";
      }

      if (segment->target == 1.f) {
        const auto endnodeid = edge_nodes.second;
        if (endnodeid.Is_Valid()) {
          route << endnodeid;
        } else {
          route << "InvalidId";
        }
      } else {
        route << segment->target;
      }

      route << "]";
    } else {
      route << "[dummy]";
    }
    route << " ";
  }

  auto route_str = route.str();
  if (!route_str.empty()) {
    route_str.pop_back();
  }
  return route_str;
}

// Check if all edge segments of the route are successive, and
// contain no loop
template <typename segment_iterator_t>
bool ValidateRoute(baldr::GraphReader& graphreader,
                   segment_iterator_t segment_begin,
                   segment_iterator_t segment_end,
                   const baldr::GraphTile*& tile) {
  if (segment_begin == segment_end) {
    return true;
  }

  for (auto prev_segment = segment_begin, segment = std::next(segment_begin); segment != segment_end;
       prev_segment = segment, segment++) {

    // Successive segments must be adjacent and no loop absolutely!
    if (prev_segment->edgeid == segment->edgeid) {
      if (prev_segment->target != segment->source) {
        LOG_ERROR("CASE 1: Found disconnected segments at " +
                  std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));

        // A temporary fix here: this exception is due to a few of
        // loops in the graph. The error message below is one example
        // of the fail case: the edge 2/698780/4075 is a loop since it
        // ends and starts at the same node 2/698780/1433:

        // [ERROR] Found disconnected segments at 2
        // [ERROR] [dummy] [0.816102 2/698780/4075 2/698780/1433] [2/698780/1433 2/698780/4075
        // 0.460951]

        // We should remove this block of code when this issue is
        // solved from upstream
        const auto endnodeid = graphreader.edge_endnode(prev_segment->edgeid, tile);
        const auto startnodeid = graphreader.edge_startnode(segment->edgeid, tile);
        if (endnodeid == startnodeid) {
          LOG_ERROR("This is a loop. Let it go");
          return true;
        }
        // End of the fix

        return false;
      }
    } else {
      // Make sure edges are connected (could be on different levels)
      if (!(prev_segment->target == 1.f && segment->source == 0.f &&
            graphreader.AreEdgesConnectedForward(prev_segment->edgeid, segment->edgeid, tile))) {
        LOG_ERROR("CASE 2: Found disconnected segments at " +
                  std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
        return false;
      }
    }
  }
  return true;
}

template <typename segment_iterator_t>
void MergeEdgeSegments(std::vector<EdgeSegment>& route,
                       segment_iterator_t segment_begin,
                       segment_iterator_t segment_end) {
  for (auto segment = segment_begin; segment != segment_end; segment++) {
    if (!route.empty()) {
      auto& last_segment = route.back();
      if (last_segment.edgeid == segment->edgeid && last_segment.target == segment->source) {
        // Extend last segment
        last_segment.target = segment->target;
      } else {
        route.push_back(*segment);
      }
    } else {
      route.push_back(*segment);
    }
  }
}

}; // namespace

namespace valhalla {
namespace meili {

EdgeSegment::EdgeSegment(baldr::GraphId the_edgeid,
                         float the_source,
                         float the_target,
                         int the_first_match_idx,
                         int the_last_match_idx,
                         bool disconnect)
    : edgeid(the_edgeid), source(the_source), target(the_target),
      first_match_idx(the_first_match_idx), last_match_idx(the_last_match_idx),
      discontinuity(disconnect) {
  if (!edgeid.Is_Valid()) {
    throw std::invalid_argument("Invalid edgeid");
  }

  if (!(0.f <= source && source <= target && target <= 1.f)) {
    throw std::invalid_argument("Expect 0.f <= source <= target <= 1.f, but you got source = " +
                                std::to_string(source) + " and target = " + std::to_string(target));
  }
}

std::vector<midgard::PointLL> EdgeSegment::Shape(baldr::GraphReader& graphreader) const {
  const baldr::GraphTile* tile = nullptr;
  const auto edge = graphreader.directededge(edgeid, tile);
  if (edge) {
    const auto edgeinfo = tile->edgeinfo(edge->edgeinfo_offset());
    const auto& shape = edgeinfo.shape();
    if (edge->forward()) {
      return midgard::trim_polyline(shape.cbegin(), shape.cend(), source, target);
    } else {
      return midgard::trim_polyline(shape.crbegin(), shape.crend(), source, target);
    }
  }
  return {};
}

bool EdgeSegment::Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const {
  if (edgeid != other.edgeid) {
    if (target == 1.f && other.source == 0.f) {
      return graphreader.AreEdgesConnectedForward(edgeid, other.edgeid);
    } else {
      return false;
    }
  } else {
    return target == other.source;
  }
}

bool MergeRoute(const State& source, const State& target, std::vector<EdgeSegment>& route) {
  const auto route_rbegin = source.RouteBegin(target), route_rend = source.RouteEnd();

  // No route, discontinuity
  if (route_rbegin == route_rend) {
    return false;
  }

  std::vector<EdgeSegment> segments;

  auto label = route_rbegin;

  // Skip the first dummy edge std::prev(route_rend)
  for (; std::next(label) != route_rend; label++) {
    segments.emplace_back(label->edgeid(), label->source(), label->target());
  }

  // Make sure the first edge has an invalid predecessor
  if (label->predecessor() != baldr::kInvalidLabel) {
    throw std::logic_error("The first edge must be an origin (invalid predecessor)");
  }

  // TODO: why doesnt routing.cc return trivial routes? move this logic there
  // Might be a trivial route
  if (segments.empty()) {
    // If we have no chance of making a trivial route bail
    if (source.candidate().edges.empty() ||
        source.stateid().id() >= source.candidate().edges.size()) {
      return false;
    }
    // Make sure the identical candidate exists in both states
    const auto& candidate = source.candidate().edges[source.stateid().id()];
    if (std::find_if(target.candidate().edges.begin(), target.candidate().edges.end(),
                     [&candidate](const baldr::PathLocation::PathEdge& e) {
                       return e.id == candidate.id && e.percent_along == candidate.percent_along;
                     }) == target.candidate().edges.end()) {
      return false;
    }
    // Make a trivial route
    segments.emplace_back(candidate.id, candidate.percent_along, candidate.percent_along);
  }

  route.insert(route.end(), segments.crbegin(), segments.crend());
  return true;
}

void cut_segments(const std::vector<MatchResult>& match_results,
                  int first_idx,
                  int last_idx,
                  const std::vector<EdgeSegment>& segments,
                  std::vector<EdgeSegment>& new_segments) {
  auto first_segment = segments.begin();
  int prev_idx = first_idx;
  for (int curr_idx = first_idx + 1; curr_idx <= last_idx; ++curr_idx) {
    const MatchResult& curr_match = match_results[curr_idx];
    const MatchResult& prev_match = match_results[prev_idx];

    // skip if we do not need to cut
    if (!curr_match.is_break_point && curr_idx != last_idx) {
      continue;
    }
    // we want to handle to loop by locating the correct target edge by comparing the distance alone
    bool loop = prev_match.edgeid == curr_match.edgeid &&
                prev_match.distance_along > curr_match.distance_along;
    // if it is a loop, we start the search after the first edge
    auto last_segment = std::find_if(first_segment + static_cast<size_t>(loop), segments.end(),
                                     [&curr_match](const EdgeSegment& segment) {
                                       return (segment.edgeid == curr_match.edgeid);
                                     });
    assert(last_segment != segments.cend());

    // we need to close the previous edge
    size_t old_size = new_segments.size();
    new_segments.insert(new_segments.cend(), first_segment, last_segment + 1);
    new_segments[old_size].first_match_idx = prev_idx;
    // when the points got interpolated, we want to use the match results' distance along
    // otherwise, we use the segment's source or target because if it is a node snap, the
    // match result can only hold one candidate, we need either side of the node.
    new_segments[old_size].source =
        prev_match.HasState() ? first_segment->source : prev_match.distance_along;
    new_segments.back().last_match_idx = curr_idx;
    new_segments.back().target =
        curr_match.HasState() ? last_segment->target : curr_match.distance_along;

    first_segment = last_segment;
    prev_idx = curr_idx;
  }
}

std::vector<EdgeSegment> ConstructRoute(const MapMatcher& mapmatcher,
                                        const std::vector<MatchResult>& match_results,
                                        baldr::GraphReader& reader) {
  if (match_results.empty()) {
    return {};
  }

  std::vector<EdgeSegment> route;
  const baldr::GraphTile* tile = nullptr;

  // Merge segments into route
  // std::deque<int> match_indices;
  std::vector<EdgeSegment> segments;
  std::vector<EdgeSegment> new_segments;
  const MatchResult* prev_match{nullptr};
  int prev_idx = -1;
  for (int curr_idx = 0, n = static_cast<int>(match_results.size()); curr_idx < n; ++curr_idx) {
    const MatchResult& match = match_results[curr_idx];

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
      if (!MergeRoute(prev_state, state, segments)) {
        // we are only discontinuous but only if this isnt the beginning of the route
        route.back().discontinuity = !route.empty();
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

        EdgeSegment& first_half = route.back();
        EdgeSegment& second_half = new_segments.front();

        // Before merge:               -1          prev_idx      prev_idx, last_match_idx
        //                first_half    ------------------>      ------------------>  second_half
        // After merge :               -1                                  last_match_idx
        //                             ------------------------------------------->  second_half
        if (first_half.first_match_idx == -1) {
          second_half.source = 0.f;
          second_half.first_match_idx = -1;
        }
        // Before merge:            first_match_idx, prev_idx   prev_idx,        -1
        //                first_half    ------------------>      ------------------>  second_half
        // After merge :             first_match_idx                             -1
        //                              ------------------------------------------->  second_half
        if (second_half.last_match_idx == -1) {
          second_half.target = 1.f;
          second_half.first_match_idx = route.back().first_match_idx;
        }
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

template <typename segment_iterator_t>
std::vector<std::vector<midgard::PointLL>> ConstructRouteShapes(baldr::GraphReader& graphreader,
                                                                segment_iterator_t begin,
                                                                segment_iterator_t end) {
  if (begin == end) {
    return {};
  }

  std::vector<std::vector<midgard::PointLL>> shapes;

  for (auto segment = begin, prev_segment = end; segment != end; segment++) {
    const auto& shape = segment->Shape(graphreader);
    if (shape.empty()) {
      continue;
    }

    auto shape_begin = shape.begin();
    if (prev_segment != end && prev_segment->Adjoined(graphreader, *segment)) {
      // The beginning vertex has been written. Hence skip
      std::advance(shape_begin, 1);
    } else {
      // Disconnected. Hence a new start
      shapes.emplace_back();
    }

    for (auto vertex = shape_begin; vertex != shape.end(); vertex++) {
      shapes.back().push_back(*vertex);
    }

    prev_segment = segment;
  }

  return shapes;
}

template std::vector<std::vector<midgard::PointLL>>
ConstructRouteShapes<std::vector<EdgeSegment>::const_iterator>(
    baldr::GraphReader&,
    std::vector<EdgeSegment>::const_iterator,
    std::vector<EdgeSegment>::const_iterator);

template std::vector<std::vector<midgard::PointLL>>
ConstructRouteShapes<std::vector<EdgeSegment>::iterator>(baldr::GraphReader&,
                                                         std::vector<EdgeSegment>::iterator,
                                                         std::vector<EdgeSegment>::iterator);
} // namespace meili
} // namespace valhalla
