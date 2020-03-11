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
    std::cout << graphreader.encoded_edge_shape(prev_segment->edgeid) << " " << prev_segment->target
              << " " << graphreader.encoded_edge_shape(segment->edgeid) << " " << segment->source
              << std::endl;

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

bool MergeRoute(std::vector<EdgeSegment>& route, const State& source, const State& target) {
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

  // MergeEdgeSegments(route, segments.rbegin(), segments.rend());
  route.insert(route.end(), segments.crbegin(), segments.crend());
  return true;
}

std::vector<EdgeSegment> MergeRoute(const State& source, const State& target) {
  std::vector<EdgeSegment> route;
  MergeRoute(route, source, target);
  return route;
}

void reorder_segments(const std::vector<MatchResult>& match_results,
                      const std::deque<int>& match_indices,
                      baldr::GraphReader& reader,
                      std::vector<EdgeSegment>& segments) {
  if (segments.empty()) {
    return;
  } else if (match_indices.size() == 2) {
    segments.front().first_match_idx = match_indices.front();
    segments.back().last_match_idx = match_indices.back();
    return;
  }
  // I have to match it to the segments, since there could be intersection matching, transition
  // matching etc

  segments.clear();

  MatchResult prev_match = match_results.front();
  int prev_idx = match_indices.front();
  for (int i = 1, n = static_cast<int>(match_indices.size()); i < n; ++i) {
    int curr_idx = match_indices[i];
    const MatchResult& curr_match = match_results[curr_idx];

    if (prev_match.edgeid != curr_match.edgeid &&
        !reader.AreEdgesConnectedForward(prev_match.edgeid, curr_match.edgeid)) {
      continue;
    }

    if (prev_match.edgeid != curr_match.edgeid) {
      segments.push_back({prev_match.edgeid, prev_match.distance_along, 1.f, prev_idx, -1});
      segments.push_back({curr_match.edgeid, 0.f, curr_match.distance_along, -1, curr_idx});
    } else {
      segments.push_back({prev_match.edgeid, prev_match.distance_along, curr_match.distance_along,
                          prev_idx, curr_idx});
    }

    prev_match = curr_match;
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
  std::deque<int> match_indices;
  std::vector<EdgeSegment> segments;
  const MatchResult* prev_match{nullptr};
  int prev_idx = -1;
  for (int curr_idx = 0, n = static_cast<int>(match_results.size()); curr_idx < n; ++curr_idx) {
    const MatchResult& match = match_results[curr_idx];
    if (!match.HasState()) {
      if (match.edgeid.Is_Valid()) {
        match_indices.push_back(curr_idx);
      }
      continue;
    }

    if (prev_match && prev_match->HasState()) {
      const auto &prev_state = mapmatcher.state_container().state(prev_match->stateid),
                 state = mapmatcher.state_container().state(match.stateid);

      // get the route between the two states by walking edge labels backwards
      // then reverse merge the segments together which are on the same edge so we have a
      // minimum number of segments. in this case we could at minimum end up with 1 segment

      segments.clear();
      if (!MergeRoute(segments, prev_state, state) && !route.empty()) {
        route.back().discontinuity = true;
      }

      match_indices.push_front(prev_idx);
      match_indices.push_back(curr_idx);

      reorder_segments(match_results, match_indices, reader, segments);

      // TODO remove: the code is pretty mature we dont need this check its wasted cpu
      if (!ValidateRoute(mapmatcher.graphreader(), segments.begin(), segments.end(), tile)) {
        throw std::runtime_error("Found invalid route");
      }

      // from this match to the last match we may be on the same edge, we call merge here
      // instead of just appending this vector to the end of the route vector because
      // we may merge the last segment of route with the beginning segment of segments
      // MergeEdgeSegments(route, segments.begin(), segments.end());
      route.insert(route.end(), segments.cbegin(), segments.cend());
    }

    prev_match = &match;
    prev_idx = curr_idx;
    match_indices.clear();
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
