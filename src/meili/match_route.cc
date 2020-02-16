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

EdgeSegment::EdgeSegment(baldr::GraphId the_edgeid, float the_source, float the_target)
    : edgeid(the_edgeid), source(the_source), target(the_target) {
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

  MergeEdgeSegments(route, segments.rbegin(), segments.rend());
  return true;
}

std::vector<EdgeSegment> MergeRoute(const State& source, const State& target) {
  std::vector<EdgeSegment> route;
  MergeRoute(route, source, target);
  return route;
}

template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(const MapMatcher& mapmatcher, match_iterator_t begin, match_iterator_t end) {
  if (begin == end) {
    return {};
  }

  std::vector<EdgeSegment> route;
  const baldr::GraphTile* tile = nullptr;

  // Merge segments into route
  for (auto prev_match = end, match = begin; match != end; match++) {
    if (!match->HasState()) {
      continue;
    }

    if (prev_match != end) {
      const auto &prev_state = mapmatcher.state_container().state(prev_match->stateid),
                 state = mapmatcher.state_container().state(match->stateid);

      // get the route between the two states by walking edge labels backwards
      // then reverse merge the segments together which are on the same edge so we have a
      // minimum number of segments. in this case we could at minimum end up with 1 segment
      std::vector<EdgeSegment> segments;
      MergeRoute(segments, prev_state, state);

      // TODO remove: the code is pretty mature we dont need this check its wasted cpu
      if (!ValidateRoute(mapmatcher.graphreader(), segments.begin(), segments.end(), tile)) {
        throw std::runtime_error("Found invalid route");
      }

      // from this match to the last match we may be on the same edge, we call merge here
      // instead of just appending this vector to the end of the route vector because
      // we may merge the last segment of route with the beginning segment of segments
      MergeEdgeSegments(route, segments.begin(), segments.end());
    }

    prev_match = match;
  }
  return route;
}

// explicit instantiations
template std::vector<EdgeSegment>
ConstructRoute<std::vector<MatchResult>::iterator>(const MapMatcher&,
                                                   std::vector<MatchResult>::iterator,
                                                   std::vector<MatchResult>::iterator);

template std::vector<EdgeSegment>
ConstructRoute<std::vector<MatchResult>::const_iterator>(const MapMatcher&,
                                                         std::vector<MatchResult>::const_iterator,
                                                         std::vector<MatchResult>::const_iterator);

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
