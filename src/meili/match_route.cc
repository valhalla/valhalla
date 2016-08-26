#include <vector>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>

#include "meili/geometry_helpers.h"
#include "meili/match_route.h"
#include "meili/graph_helpers.h"


namespace {

using namespace valhalla;
using namespace valhalla::meili;


template <typename segment_iterator_t>
std::string
RouteToString(baldr::GraphReader& graphreader,
              segment_iterator_t segment_begin,
              segment_iterator_t segment_end,
              const baldr::GraphTile*& tile)
{
  // The string will look like: [dummy] [source/startnodeid edgeid target/endnodeid] ...
  std::ostringstream route;

  for (auto segment = segment_begin; segment != segment_end; segment++) {
    if (segment->edgeid.Is_Valid()) {
      route << "[";

      if (segment->source == 0.f) {
        const auto startnodeid = meili::helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
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
        route << " " << "InvalidId" << " ";
      }

      if (segment->target == 1.f) {
        const auto endnodeid = meili::helpers::edge_endnodeid(graphreader, segment->edgeid, tile);
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


// Validate a route. It check if all edge segments of the route are
// valid and successive, and no loop
template <typename segment_iterator_t>
bool ValidateRoute(baldr::GraphReader& graphreader,
                   segment_iterator_t segment_begin,
                   segment_iterator_t segment_end,
                   const baldr::GraphTile*& tile)
{
  if (segment_begin == segment_end) {
    return true;
  }

  // The first segment must be dummy
  if (!(!segment_begin->edgeid.Is_Valid()
        && segment_begin->source == 0.f
        && segment_begin->target == 0.f)) {
    LOG_ERROR("Found the first edge segment is not dummy");
    LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
    return false;
  }

  for (auto segment = std::next(segment_begin);  // Skip the first dummy segment
       segment != segment_end; segment++) {
    // The rest of segments must have valid edgeid
    if (!segment->edgeid.Is_Valid()) {
      LOG_ERROR("Found invalid edgeid at segment " + std::to_string(segment - segment_begin));
      LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
      return false;
    }

    // Skip the first non-dummy segment
    const auto prev_segment = std::prev(segment);
    if (prev_segment == segment_begin) {
      continue;
    }

    // Successive segments must be adjacent and no loop absolutely!
    if (prev_segment->edgeid == segment->edgeid) {
      if (prev_segment->target != segment->source) {
        LOG_ERROR("Found disconnected segments at " + std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));

        // A temporary fix here: this exception is due to a few of
        // loops in the graph. The error message below is one example
        // of the fail case: the edge 2/698780/4075 is a loop since it
        // ends and starts at the same node 2/698780/1433:

        // [ERROR] Found disconnected segments at 2
        // [ERROR] [dummy] [0.816102 2/698780/4075 2/698780/1433] [2/698780/1433 2/698780/4075 0.460951]

        // We should remove this block of code when this issue is
        // solved from upstream
        const auto endnodeid = meili::helpers::edge_endnodeid(graphreader, prev_segment->edgeid, tile);
        const auto startnodeid = meili::helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
        if (endnodeid == startnodeid) {
          LOG_ERROR("This is a loop. Let it go");
          return true;
        }
        // End of the fix

        return false;
      }
    } else {
      const auto endnodeid = meili::helpers::edge_endnodeid(graphreader, prev_segment->edgeid, tile),
               startnodeid = meili::helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
      if (!(prev_segment->target == 1.f
            && segment->source == 0.f
            && endnodeid == startnodeid)) {
        LOG_ERROR("Found disconnected segments at " + std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
        return false;
      }
    }
  }

  return true;
}


template <typename segment_iterator_t>
void MergeRoute(std::vector<meili::EdgeSegment>& route,
                segment_iterator_t segment_begin,
                segment_iterator_t segment_end)
{
  if (segment_begin == segment_end) {
    return;
  }

  for (auto segment = std::next(segment_begin);  // Skip the first dummy segment
       segment != segment_end; segment++) {
    if (!segment->edgeid.Is_Valid()) {
      throw std::runtime_error("Still found an invalid edgeid in route segments");
    }
    if(!route.empty()) {
      auto& last_segment = route.back();
      if (last_segment.edgeid == segment->edgeid) {
        if (last_segment.target != segment->source
            && segment != std::next(segment_begin)) {
          // TODO should throw runtime error. See the temporary fix
          LOG_ERROR("Still found a disconnected route in which segment "
                    + std::to_string(segment - segment_begin) + " ends at "
                    + std::to_string(last_segment.target)
                    + " but the next segment starts at "
                    + std::to_string(segment->source));
        }
        // and here we should extend last_segment.target =
        // segment->target since last_segment.target <=
        // segment->target but see the temporary fix
        last_segment.target = std::max(last_segment.target, segment->target);
      } else {
        route.push_back(*segment);
      }
    } else {
      route.push_back(*segment);
    }
  }
}


//explicit instantiations
template std::string RouteToString<std::vector<EdgeSegment>::iterator>(
    baldr::GraphReader&, std::vector<EdgeSegment>::iterator, std::vector<EdgeSegment>::iterator, const baldr::GraphTile*&);
template std::string RouteToString<std::vector<EdgeSegment>::const_iterator>(
    baldr::GraphReader&, std::vector<EdgeSegment>::const_iterator, std::vector<EdgeSegment>::const_iterator, const baldr::GraphTile*&);

template bool ValidateRoute<std::vector<EdgeSegment>::iterator>(
    baldr::GraphReader&, std::vector<EdgeSegment>::iterator, std::vector<EdgeSegment>::iterator, const baldr::GraphTile*&);
template bool ValidateRoute<std::vector<EdgeSegment>::const_iterator>(
    baldr::GraphReader&, std::vector<EdgeSegment>::const_iterator, std::vector<EdgeSegment>::const_iterator, const baldr::GraphTile*&);

template void MergeRoute<std::vector<EdgeSegment>::iterator>(
    std::vector<EdgeSegment>&, std::vector<EdgeSegment>::iterator, std::vector<EdgeSegment>::iterator);
template void MergeRoute<std::vector<EdgeSegment>::const_iterator>(
    std::vector<EdgeSegment>&, std::vector<EdgeSegment>::const_iterator, std::vector<EdgeSegment>::const_iterator);

}


namespace valhalla {
namespace meili {

EdgeSegment::EdgeSegment(baldr::GraphId the_edgeid,
                         float the_source,
                         float the_target)
    : edgeid(the_edgeid),
      source(the_source),
      target(the_target)
{
  if (!(0.f <= source && source <= target && target <= 1.f)) {
    throw std::invalid_argument("Expect 0.f <= source <= target <= 1.f, but you got source = "
                                + std::to_string(source)
                                + " and target = "
                                + std::to_string(target));
  }
}


std::vector<midgard::PointLL>
EdgeSegment::Shape(baldr::GraphReader& graphreader) const
{
  const baldr::GraphTile* tile = nullptr;
  const auto edge = helpers::edge_directededge(graphreader, edgeid, tile);
  if (edge) {
    const auto edgeinfo = tile->edgeinfo(edge->edgeinfo_offset());
    const auto& shape = edgeinfo->shape();
    if (edge->forward()) {
      return helpers::ClipLineString(shape.cbegin(), shape.cend(), source, target);
    } else {
      return helpers::ClipLineString(shape.crbegin(), shape.crend(), source, target);
    }
  }

  return {};
}


bool EdgeSegment::Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const
{
  // Skip dummy segments
  if (!edgeid.Is_Valid() || !other.edgeid.Is_Valid()) {
    return false;
  }

  if (edgeid != other.edgeid) {
    if (target == 1.f && other.source == 0.f) {
      const auto endnode = helpers::edge_endnodeid(graphreader, edgeid);
      return endnode.Is_Valid() && endnode == helpers::edge_startnodeid(graphreader, other.edgeid);
    } else {
      return false;
    }
  } else {
    return target == other.source;
  }
}


template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(baldr::GraphReader& graphreader,
               const MapMatcher& mapmatcher,
               match_iterator_t begin,
               match_iterator_t end)
{
  std::vector<EdgeSegment> route;
  match_iterator_t previous_match = end;
  const auto& mapmatching = mapmatcher.mapmatching();
  const baldr::GraphTile* tile = nullptr;

  for (auto match = begin; match != end; match++) {
    if (!match->HasState()) {
      continue;
    }

    if (previous_match != end) {
      std::vector<EdgeSegment> segments;
      const auto& previous_state = mapmatching.state(previous_match->stateid());
      for (auto segment = previous_state.RouteBegin(mapmatching.state(match->stateid())),
                    end = previous_state.RouteEnd();
           segment != end; segment++) {
        segments.emplace_back(segment->edgeid, segment->source, segment->target);
      }
      if (ValidateRoute(graphreader, segments.rbegin(), segments.rend(), tile)) {
        MergeRoute(route, segments.rbegin(), segments.rend());
      } else {
        throw std::runtime_error("Found invalid route");
      }
    }
    previous_match = match;
  }

  return route;
}


//explicit instantiations
template std::vector<EdgeSegment>
ConstructRoute<std::vector<MatchResult>::iterator>(
    baldr::GraphReader&,
    const MapMatcher&,
    std::vector<MatchResult>::iterator,
    std::vector<MatchResult>::iterator);

template std::vector<EdgeSegment>
ConstructRoute<std::vector<MatchResult>::const_iterator>(
    baldr::GraphReader&,
    const MapMatcher&,
    std::vector<MatchResult>::const_iterator,
    std::vector<MatchResult>::const_iterator);

}
}
