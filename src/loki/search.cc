#include "loki/search.h"
#include "baldr/tilehierarchy.h"
#include "loki/reach.h"
#include "midgard/distanceapproximator.h"
#include "midgard/linesegment2.h"
#include "midgard/util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <list>
#include <unordered_set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace {

float PassThroughEdgeFilter(const valhalla::baldr::DirectedEdge* edge) {
  return !(edge->is_shortcut() || edge->IsTransitLine());
}

bool PassThroughNodeFilter(const valhalla::baldr::NodeInfo* node) {
  return false;
}

template <typename T> inline T square(T v) {
  return v * v;
}

bool side_filter(const PathLocation::PathEdge& edge, const Location& location, GraphReader& reader) {
  // nothing to filter if you dont want to filter or if there is no side of street
  if (edge.sos == PathLocation::SideOfStreet::NONE ||
      location.preferred_side_ == Location::PreferredSide::EITHER)
    return false;

  // need the driving side for this edge
  const GraphTile* tile;
  auto* opp = reader.GetOpposingEdge(edge.id, tile);
  if (!opp)
    return false;
  auto* node = reader.GetEndNode(opp, tile);
  if (!node)
    return false;
  // if its on the right side and you drive on the rigth OR if its not on the right and you dont drive
  // on the right THEN its the same side that you drive on
  bool same = node->drive_on_right() == (edge.sos == PathLocation::SideOfStreet::RIGHT);
  // and then if you were asking for the same and it was the same OR if you were asking for opposite
  // and it was opposite THEN we dont filter
  return same != (location.preferred_side_ == Location::PreferredSide::SAME);
}

bool heading_filter(const DirectedEdge* edge,
                    const EdgeInfo& info,
                    const Location& location,
                    const PointLL& point,
                    size_t index) {
  // no heading means we filter nothing
  if (!location.heading_) {
    return false;
  }

  // get the angle of the shape from this point
  auto angle =
      tangent_angle(index, point, info.shape(),
                    GetOffsetForHeading(edge->classification(), edge->use()), edge->forward());
  // we want the closest distance between two angles which can be had
  // across 0 or between the two so we just need to know which is bigger
  if (*location.heading_ > angle) {
    return std::min(*location.heading_ - angle, (360.f - *location.heading_) + angle) >
           location.heading_tolerance_;
  }
  return std::min(angle - *location.heading_, (360.f - angle) + *location.heading_) >
         location.heading_tolerance_;
}

PathLocation::SideOfStreet flip_side(const PathLocation::SideOfStreet side) {
  if (side != PathLocation::SideOfStreet::NONE) {
    return side == PathLocation::SideOfStreet::LEFT ? PathLocation::SideOfStreet::RIGHT
                                                    : PathLocation::SideOfStreet::LEFT;
  }
  return side;
}

std::function<std::tuple<int32_t, unsigned short, float>()> make_binner(const PointLL& p,
                                                                        const GraphReader& reader) {
  const auto& tiles = TileHierarchy::levels().rbegin()->second.tiles;
  return tiles.ClosestFirst(p);
}

// Model a segment (2 consecutive points in an edge in a bin).
struct candidate_t {
  double sq_distance;
  PointLL point;
  size_t index;

  GraphId edge_id;
  const DirectedEdge* edge;
  std::shared_ptr<const EdgeInfo> edge_info;

  const GraphTile* tile;

  bool operator<(const candidate_t& c) const {
    return sq_distance < c.sq_distance;
  }

  PathLocation::SideOfStreet
  get_side(const PointLL& original, double sq_distance, double sq_tolerance) const {
    // its so close to the edge that its basically on the edge
    if (sq_distance < sq_tolerance) {
      return PathLocation::SideOfStreet::NONE;
    }

    // get the side TODO: this can technically fail for longer segments..
    // to fix it we simply compute the plane formed by the triangle
    // through the center of the earth and the two shape points and test
    // whether the original point is above or below the plane (depending on winding)
    auto& shape = edge_info->shape();
    LineSegment2<PointLL> segment(shape[index], shape[index + 1]);
    return (segment.IsLeft(original) > 0) == edge->forward() ? PathLocation::SideOfStreet::LEFT
                                                             : PathLocation::SideOfStreet::RIGHT;
  }
};

// This structure contains the context of the projection of a
// Location.  At the creation, a bin is affected to the point.  The
// test() method should be called to each valid segment of the bin.
// When the bin is finished, next_bin() switch to the next possible
// interesting bin.  if has_bin() is false, then the best projection
// is found.
struct projector_wrapper {
  projector_wrapper(const Location& location, GraphReader& reader)
      : binner(make_binner(location.latlng_, reader)), location(location),
        sq_radius(square(double(location.radius_))), project(location.latlng_) {
    // TODO: something more empirical based on radius
    unreachable.reserve(64);
    reachable.reserve(64);
    // initialize
    next_bin(reader);
  }

  // non default constructible and move only type
  projector_wrapper() = delete;
  projector_wrapper(const projector_wrapper&) = delete;
  projector_wrapper& operator=(const projector_wrapper&) = delete;
  projector_wrapper(projector_wrapper&&) = default;
  projector_wrapper& operator=(projector_wrapper&&) = default;

  // the ones marked with null current tile are finished so put them on the end
  // otherwise we sort by bin so that ones with the same bin are next to each other
  bool operator<(const projector_wrapper& other) const {
    // the
    if (cur_tile != other.cur_tile) {
      return cur_tile > other.cur_tile;
    }
    return bin_index < other.bin_index;
  }

  bool has_same_bin(const projector_wrapper& other) const {
    return cur_tile == other.cur_tile && bin_index == other.bin_index;
  }

  bool has_bin() const {
    return cur_tile != nullptr;
  }

  // Advance to the next bin. Must not be called if has_bin() is false.
  void next_bin(GraphReader& reader) {
    do {
      // give up if the next bin is outside the overall cut off OR
      // we have something AND cant find more in the search radius AND
      // cant find anything better in general than what we have
      int32_t tile_index;
      float distance;
      std::tie(tile_index, bin_index, distance) = binner();
      if (distance > location.search_cutoff_ ||
          (reachable.size() && distance > location.radius_ &&
           distance > std::sqrt(reachable.back().sq_distance))) {
        cur_tile = nullptr;
        break;
      }

      // grab the tile the lat, lon is in
      auto tile_id = GraphId(tile_index, TileHierarchy::levels().rbegin()->first, 0);
      reader.GetGraphTile(tile_id, cur_tile);
    } while (!cur_tile);
  }

  std::function<std::tuple<int32_t, unsigned short, float>()> binner;
  const GraphTile* cur_tile = nullptr;
  Location location;
  unsigned short bin_index = 0;
  double sq_radius;
  std::vector<candidate_t> unreachable;
  std::vector<candidate_t> reachable;
  double closest_external_reachable = std::numeric_limits<double>::max();

  // critical data
  projector_t project;
};

struct bin_handler_t {
  std::vector<projector_wrapper> pps;
  valhalla::baldr::GraphReader& reader;
  EdgeFilter edge_filter;
  NodeFilter node_filter;
  const DynamicCost* costing;
  unsigned int max_reach_limit;
  std::vector<candidate_t> bin_candidates;
  std::unordered_set<uint64_t> correlated_edges;

  // keep track of edges whose reachability we've already computed
  // TODO: dont use pointers as keys, its safe for now but fancy caching one day could be bad
  std::unordered_map<const DirectedEdge*, directed_reach> directed_reaches;

  bin_handler_t(const std::vector<valhalla::baldr::Location>& locations,
                valhalla::baldr::GraphReader& reader,
                const DynamicCost* costing)
      : reader(reader), costing(costing),
        edge_filter(costing ? costing->GetEdgeFilter() : PassThroughEdgeFilter),
        node_filter(costing ? costing->GetNodeFilter() : PassThroughNodeFilter) {
    // get the unique set of input locations and the max reachability of them all
    std::unordered_set<Location> uniq_locations(locations.begin(), locations.end());
    pps.reserve(uniq_locations.size());
    max_reach_limit = 0;
    for (const auto& loc : uniq_locations) {
      pps.emplace_back(loc, reader);
      max_reach_limit = std::max(max_reach_limit, loc.min_outbound_reach_);
      max_reach_limit = std::max(max_reach_limit, loc.min_inbound_reach_);
    }
    // very annoying but it saves a lot of time to preallocate this instead of doing it in the loop
    // in handle_bins
    bin_candidates.resize(pps.size());
    // TODO: make space for reach check in a more empirical way
    auto reservation = std::max(max_reach_limit, static_cast<decltype(max_reach_limit)>(1));
    directed_reaches.reserve(reservation * 1024);
  }

  void correlate_node(const Location& location,
                      const GraphId& found_node,
                      const candidate_t& candidate,
                      PathLocation& correlated,
                      std::vector<PathLocation::PathEdge>& filtered) {
    // the search cutoff is a hard filter so skip any outside of that
    if (candidate.point.Distance(location.latlng_) > location.search_cutoff_)
      return;
    // we need this because we might need to go to different levels
    float distance = std::numeric_limits<float>::lowest();
    std::function<void(const GraphId& node_id, bool transition)> crawl;
    crawl = [&](const GraphId& node_id, bool follow_transitions) {
      // now that we have a node we can pass back all the edges leaving and entering it
      const auto* tile = reader.GetGraphTile(node_id);
      if (!tile) {
        return;
      }
      const auto* node = tile->node(node_id);
      const auto* start_edge = tile->directededge(node->edge_index());
      const auto* end_edge = start_edge + node->edge_count();
      PointLL node_ll = node->latlng(tile->header()->base_ll());
      // cache the distance
      if (distance == std::numeric_limits<float>::lowest())
        distance = node_ll.Distance(location.latlng_);
      // add edges entering/leaving this node
      for (const auto* edge = start_edge; edge < end_edge; ++edge) {
        // get some info about this edge and the opposing
        GraphId id = tile->id();
        id.set_id(node->edge_index() + (edge - start_edge));
        auto info = tile->edgeinfo(edge->edgeinfo_offset());

        // do we want this edge
        if (edge_filter(edge) != 0.0f) {
          auto reach = get_reach(edge);
          PathLocation::PathEdge path_edge{std::move(id),
                                           0.f,
                                           node_ll,
                                           distance,
                                           PathLocation::NONE,
                                           reach.outbound,
                                           reach.inbound};
          auto index = edge->forward() ? 0 : info.shape().size() - 2;
          if (heading_filter(edge, info, location, candidate.point, index)) {
            filtered.emplace_back(std::move(path_edge));
          } else if (correlated_edges.insert(path_edge.id).second) {
            correlated.edges.push_back(std::move(path_edge));
          }
        }

        // do we want the evil twin
        const GraphTile* other_tile;
        const auto other_id = reader.GetOpposingEdgeId(id, other_tile);
        if (!other_tile) {
          continue;
        }
        const auto* other_edge = other_tile->directededge(other_id);
        if (edge_filter(other_edge) != 0.0f) {
          auto reach = get_reach(other_edge);
          PathLocation::PathEdge path_edge{std::move(other_id),
                                           1.f,
                                           node_ll,
                                           distance,
                                           PathLocation::NONE,
                                           reach.outbound,
                                           reach.inbound};
          // index is opposite the logic above
          auto index = other_edge->forward() ? info.shape().size() - 2 : 0;
          if (heading_filter(other_edge, tile->edgeinfo(edge->edgeinfo_offset()), location,
                             candidate.point, index)) {
            filtered.emplace_back(std::move(path_edge));
          } else if (correlated_edges.insert(path_edge.id).second) {
            correlated.edges.push_back(std::move(path_edge));
          }
        }
      }

      // Follow transition to other hierarchy levels
      if (follow_transitions && node->transition_count() > 0) {
        const NodeTransition* trans = tile->transition(node->transition_index());
        for (uint32_t i = 0; i < node->transition_count(); ++i, ++trans) {
          crawl(trans->endnode(), false);
        }
      }
    };

    // start where we are and crawl from there
    crawl(found_node, true);
  }

  void correlate_edge(const Location& location,
                      const candidate_t& candidate,
                      PathLocation& correlated,
                      std::vector<PathLocation::PathEdge>& filtered) {
    // get the distance between the result
    auto distance = candidate.point.Distance(location.latlng_);
    // the search cutoff is a hard filter so skip any outside of that
    if (distance > location.search_cutoff_)
      return;
    // now that we have an edge we can pass back all the info about it
    if (candidate.edge != nullptr) {
      // we need the ratio in the direction of the edge we are correlated to
      double partial_length = 0;
      for (size_t i = 0; i < candidate.index; ++i) {
        partial_length +=
            candidate.edge_info->shape()[i].Distance(candidate.edge_info->shape()[i + 1]);
      }
      partial_length += candidate.edge_info->shape()[candidate.index].Distance(candidate.point);
      partial_length = std::min(partial_length, static_cast<double>(candidate.edge->length()));
      float length_ratio =
          static_cast<float>(partial_length / static_cast<double>(candidate.edge->length()));
      if (!candidate.edge->forward()) {
        length_ratio = 1.f - length_ratio;
      }
      // side of street
      auto sq_tolerance = square(double(location.street_side_tolerance_));
      auto side = candidate.get_side(location.latlng_, candidate.sq_distance, sq_tolerance);
      auto reach = get_reach(candidate.edge);
      PathLocation::PathEdge path_edge{candidate.edge_id, length_ratio, candidate.point,
                                       distance,          side,         reach.outbound,
                                       reach.inbound};
      // correlate the edge we found
      if (side_filter(path_edge, location, reader) ||
          heading_filter(candidate.edge, *candidate.edge_info, location, candidate.point,
                         candidate.index)) {
        filtered.push_back(std::move(path_edge));
      } else if (correlated_edges.insert(candidate.edge_id).second) {
        correlated.edges.push_back(std::move(path_edge));
      }
      // correlate its evil twin
      const GraphTile* other_tile;
      auto opposing_edge_id = reader.GetOpposingEdgeId(candidate.edge_id, other_tile);
      const DirectedEdge* other_edge;
      if (opposing_edge_id.Is_Valid() && (other_edge = other_tile->directededge(opposing_edge_id)) &&
          edge_filter(other_edge) != 0.0f) {
        auto reach = get_reach(other_edge);
        PathLocation::PathEdge other_path_edge{opposing_edge_id, 1 - length_ratio, candidate.point,
                                               distance,         flip_side(side),  reach.outbound,
                                               reach.inbound};
        if (side_filter(other_path_edge, location, reader) ||
            heading_filter(other_edge, *candidate.edge_info, location, candidate.point,
                           candidate.index)) {
          filtered.push_back(std::move(other_path_edge));
        } else if (correlated_edges.insert(opposing_edge_id).second) {
          correlated.edges.push_back(std::move(other_path_edge));
        }
      }
    }
  }

  directed_reach get_reach(const DirectedEdge* edge) {
    // if its in cache return it
    auto itr = directed_reaches.find(edge);
    if (itr != directed_reaches.cend())
      return itr->second;

    // notice we do both directions here because in the end we use this reach for all input locations
    auto reach =
        SimpleReach(edge, max_reach_limit, reader, edge_filter, node_filter, kInbound | kOutbound);
    directed_reaches[edge] = reach;
    return reach;
  }

  // do a mini network expansion or maybe not
  directed_reach check_reachability(std::vector<projector_wrapper>::iterator begin,
                                    std::vector<projector_wrapper>::iterator end,
                                    const GraphTile* tile,
                                    const DirectedEdge* edge) {
    // no need when set to 0
    if (max_reach_limit == 0)
      return {};

    // do we already know about this one?
    auto found = directed_reaches.find(edge);
    if (found != directed_reaches.cend())
      return found->second;

    // we only want to waste time checking if this could become the best reachable option for a
    // given location
    bool check = false;
    auto c_itr = bin_candidates.begin();
    for (auto p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
      check = check || p_itr->reachable.empty() ||
              c_itr->sq_distance < p_itr->reachable.back().sq_distance;
    }

    // assume its reachable
    if (!check)
      return {max_reach_limit, max_reach_limit};

    // notice we do both directions here because in the end we use this reach for all input locations
    auto reach =
        SimpleReach(edge, max_reach_limit, reader, edge_filter, node_filter, kInbound | kOutbound);
    directed_reaches[edge] = reach;

    // if the inbound reach is not 0 and the outbound reach is not 0 and the opposing edge is not
    // filtered then the reaches of both edges are the same
    const DirectedEdge* opp_edge = nullptr;
    if (reach.outbound > 0 && reach.inbound > 0 && (opp_edge = reader.GetOpposingEdge(edge, tile)) &&
        edge_filter(opp_edge) > 0.f)
      directed_reaches[opp_edge] = reach;

    return reach;
  }

  // handle a bin for the range of candidates that share it
  void handle_bin(std::vector<projector_wrapper>::iterator begin,
                  std::vector<projector_wrapper>::iterator end) {
    // iterate over the edges in the bin
    auto tile = begin->cur_tile;
    auto edges = tile->GetBin(begin->bin_index);
    for (auto e : edges) {
      // get the tile and edge
      if (!reader.GetGraphTile(e, tile)) {
        continue;
      }

      // no thanks on this one or its evil twin
      const auto* edge = tile->directededge(e);
      if (edge_filter(edge) == 0.0f && (!(e = reader.GetOpposingEdgeId(e, tile)).Is_Valid() ||
                                        edge_filter(edge = tile->directededge(e)) == 0.0f)) {
        continue;
      }

      // reset these so we know the best point along the edge
      auto c_itr = bin_candidates.begin();
      decltype(begin) p_itr;
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
        c_itr->sq_distance = std::numeric_limits<float>::max();
      }

      // TODO: can we speed this up? the majority of edges will be short and far away enough
      // such that the closest point on the edge will be one of the edges end points, we can get
      // these coordinates them from the nodes in the graph. we can then find whichever end is
      // closest to the input point p, call it n. we can then define an half plane h intersecting n
      // so that its orthogonal to the ray from p to n. using h, we only need to test segments
      // of the shape which are on the same side of h that p is. to make this fast we would need a
      // a trivial half plane test as maybe a single dot product and comparison?

      // get some shape of the edge
      auto edge_info = std::make_shared<const EdgeInfo>(tile->edgeinfo(edge->edgeinfo_offset()));
      auto shape = edge_info->lazy_shape();
      PointLL v;
      if (!shape.empty()) {
        v = shape.pop();
      }

      // iterate along this edges segments projecting each of the points
      for (size_t i = 0; !shape.empty(); ++i) {
        auto u = v;
        v = shape.pop();
        // for each input point
        c_itr = bin_candidates.begin();
        for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
          // how close is the input to this segment
          auto point = p_itr->project(u, v);
          auto sq_distance = p_itr->project.approx.DistanceSquared(point);
          // do we want to keep it
          if (sq_distance < c_itr->sq_distance) {
            c_itr->sq_distance = sq_distance;
            c_itr->point = std::move(point);
            c_itr->index = i;
          }
        }
      }

      // if we already have a better reachable candidate we can just assume this one is reachable
      auto reach = check_reachability(begin, end, tile, edge);

      // keep the best point along this edge if it makes sense
      c_itr = bin_candidates.begin();
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
        // is this edge reachable in the right way
        bool reachable = reach.outbound >= p_itr->location.min_outbound_reach_ &&
                         reach.inbound >= p_itr->location.min_inbound_reach_;
        // it's possible that it isnt reachable but the opposing is, switch to that if so
        const GraphTile* opp_tile = tile;
        const DirectedEdge* opp_edge = nullptr;
        if (!reachable && (opp_edge = reader.GetOpposingEdge(edge, opp_tile)) &&
            edge_filter(opp_edge) > 0.f) {
          auto opp_reach = check_reachability(begin, end, opp_tile, opp_edge);
          if (opp_reach.outbound >= p_itr->location.min_outbound_reach_ &&
              opp_reach.inbound >= p_itr->location.min_inbound_reach_) {
            tile = opp_tile;
            edge = opp_edge;
            reach = opp_reach;
            reachable = true;
          }
        }

        // which batch of findings will this go into
        auto* batch = reachable ? &p_itr->reachable : &p_itr->unreachable;

        // if its empty append
        if (batch->empty()) {
          c_itr->edge = edge;
          c_itr->edge_id = e;
          c_itr->edge_info = edge_info;
          c_itr->tile = tile;
          batch->emplace_back(std::move(*c_itr));
          continue;
        }

        // get some info about possibilities
        bool in_radius = c_itr->sq_distance < p_itr->sq_radius;
        bool better = c_itr->sq_distance < batch->back().sq_distance;
        bool last_in_radius = batch->back().sq_distance < p_itr->sq_radius;
        // TODO: this is a bit blunt in that any reachable edges between the best or ones within
        // the radius will make unreachable edges that are even the tiniest bit further away unviable
        // it seems like we should have a slightly looser radius to allow for unreachable edges but
        // its unclear what that should be as in most cases we are working around not actually knowing
        // the accuracy or even input modality of the incoming location
        bool closer_external_reachable =
            reachable && c_itr->sq_distance < p_itr->closest_external_reachable;

        // it has to either be better or in the radius to move on
        if (in_radius || better) {
          c_itr->edge = edge;
          c_itr->edge_id = e;
          c_itr->edge_info = edge_info;
          c_itr->tile = tile;
          // the last one wasnt in the radius so replace it with this one because its better or is
          // in the radius
          if (!last_in_radius) {
            if (closer_external_reachable)
              p_itr->closest_external_reachable = batch->back().sq_distance;
            batch->back() = std::move(*c_itr);
            // last one is in the radius but this one is better so put it on the end
          } else if (better) {
            batch->emplace_back(std::move(*c_itr));
            // last one and this one are both in the radius but this one is not as good
          } else {
            batch->emplace_back(std::move(*c_itr));
            std::swap(*(batch->end() - 1), *(batch->end() - 2));
          }
        } // not in radius or better and reachable and closer than closest one outside of radius
        else if (closer_external_reachable)
          p_itr->closest_external_reachable = c_itr->sq_distance;
      }
    }

    // bin is finished, advance the candidates to their respective next bins
    for (auto p_itr = begin; p_itr != end; ++p_itr) {
      p_itr->next_bin(reader);
    }
  }

  // Find the best range to do.  The given vector should be sorted for
  // interesting grouping.  Returns the greatest range of non empty
  // equal bins.
  std::pair<std::vector<projector_wrapper>::iterator, std::vector<projector_wrapper>::iterator>
  find_best_range(std::vector<projector_wrapper>& pps) const {
    auto best = std::make_pair(pps.begin(), pps.begin());
    auto cur = best;
    while (cur.second != pps.end()) {
      cur.first = cur.second;
      cur.second = std::find_if_not(cur.first, pps.end(), [&cur](const projector_wrapper& pp) {
        return cur.first->has_same_bin(pp);
      });
      if (cur.first->has_bin() && cur.second - cur.first > best.second - best.first) {
        best = cur;
      }
    }
    return best;
  }

  // we keep the points sorted at each round such that unfinished ones
  // are at the front of the sorted list
  void search() {
    std::sort(pps.begin(), pps.end());
    while (pps.front().has_bin()) {
      auto range = find_best_range(pps);
      handle_bin(range.first, range.second);
      std::sort(pps.begin(), pps.end());
    }
  }

  // create the PathLocation corresponding to the best projection of the given candidate
  std::unordered_map<Location, PathLocation> finalize() {
    // at this point we have candidates for each location so now we
    // need to go get the actual correlated location with edge_id etc.
    std::unordered_map<Location, PathLocation> searched;
    for (auto& pp : pps) {
      // remove non-sensical island candidates
      auto new_end = std::remove_if(pp.unreachable.begin(), pp.unreachable.end(),
                                    [&pp](const candidate_t& c) -> bool {
                                      return c.sq_distance > pp.closest_external_reachable;
                                    });
      pp.unreachable.erase(new_end, pp.unreachable.end());
      // concatenate and sort
      pp.reachable.reserve(pp.reachable.size() + pp.unreachable.size());
      std::move(pp.unreachable.begin(), pp.unreachable.end(), std::back_inserter(pp.reachable));
      std::sort(pp.reachable.begin(), pp.reachable.end());
      // keep a look up around so we dont add duplicates with worse scores
      correlated_edges.reserve(pp.reachable.size());
      correlated_edges.clear();
      // go through getting all the results for this one
      PathLocation correlated(pp.location);
      // TODO: this is already in PathLocation, use it there
      std::vector<PathLocation::PathEdge> filtered;
      for (const auto& candidate : pp.reachable) {
        // this may be at a node, either because it was the closest thing or from snap tolerance
        bool front = candidate.point == candidate.edge_info->shape().front() ||
                     pp.location.latlng_.Distance(candidate.edge_info->shape().front()) <
                         pp.location.node_snap_tolerance_;
        bool back = candidate.point == candidate.edge_info->shape().back() ||
                    pp.location.latlng_.Distance(candidate.edge_info->shape().back()) <
                        pp.location.node_snap_tolerance_;
        // it was the begin node
        if ((front && candidate.edge->forward()) || (back && !candidate.edge->forward())) {
          const GraphTile* other_tile;
          auto opposing_edge = reader.GetOpposingEdge(candidate.edge_id, other_tile);
          if (!other_tile) {
            continue; // TODO: do an edge snap instead, but you'll only get one direction
          }
          correlate_node(pp.location, opposing_edge->endnode(), candidate, correlated, filtered);
        } // it was the end node
        else if ((back && candidate.edge->forward()) || (front && !candidate.edge->forward())) {
          correlate_node(pp.location, candidate.edge->endnode(), candidate, correlated, filtered);
        } // it was along the edge
        else {
          correlate_edge(pp.location, candidate, correlated, filtered);
        }
      }

      // if it was a through location with a heading its pretty confusing.
      // does the user want to come into and exit the location at the preferred
      // angle? for now we are just saying that they want it to exit at the
      // heading provided. this means that if it was node snapped we only
      // want the outbound edges
      if ((pp.location.stoptype_ == Location::StopType::THROUGH ||
           pp.location.stoptype_ == Location::StopType::BREAK_THROUGH) &&
          pp.location.heading_) {
        // partition the ones we want to move to the end
        auto new_end =
            std::stable_partition(correlated.edges.begin(), correlated.edges.end(),
                                  [](const PathLocation::PathEdge& e) { return !e.end_node(); });
        // move them to the end
        filtered.insert(filtered.end(), std::make_move_iterator(new_end),
                        std::make_move_iterator(correlated.edges.end()));
        // remove them from the original
        correlated.edges.erase(new_end, correlated.edges.end());
      }

      // if we have nothing because of filtering (heading/side) we'll just ignore it
      if (correlated.edges.size() == 0 && filtered.size()) {
        for (auto& path_edge : filtered) {
          if (correlated_edges.insert(path_edge.id).second) {
            correlated.edges.push_back(std::move(path_edge));
          }
        }
        filtered.clear();
      }

      // keep filtered edges for retry in case we cant find a route with non filtered edges
      // use the max score of the non filtered edges as a penalty increase on each of the
      // filtered edges so that when finding a route using non filtered edges fails the
      // use of filtered edges are always penalized higher than the non filtered ones
      auto max =
          std::max_element(correlated.edges.begin(), correlated.edges.end(),
                           [](const PathLocation::PathEdge& a, const PathLocation::PathEdge& b) {
                             return a.distance < b.distance;
                           });
      std::for_each(filtered.begin(), filtered.end(),
                    [&max](PathLocation::PathEdge& e) { e.distance += (3600.0f + max->distance); });
      correlated.filtered_edges.insert(correlated.filtered_edges.end(),
                                       std::make_move_iterator(filtered.begin()),
                                       std::make_move_iterator(filtered.end()));

      // if we found nothing that is no good but if its batch maybe throwing makes no sense?
      if (correlated.edges.size() != 0) {
        searched.insert({pp.location, correlated});
      }
    }
    // give back all the results
    return searched;
  }
};

} // namespace

namespace valhalla {
namespace loki {

std::unordered_map<valhalla::baldr::Location, PathLocation>
Search(const std::vector<valhalla::baldr::Location>& locations,
       GraphReader& reader,
       const DynamicCost* costing) {
  // trivially finished already
  if (locations.empty())
    return std::unordered_map<valhalla::baldr::Location, PathLocation>{};
  // setup the unique list of locations
  bin_handler_t handler(locations, reader, costing);
  // search over the bins doing multiple locations per bin
  handler.search();
  // turn each locations candidate set into path locations
  return handler.finalize();
}

} // namespace loki
} // namespace valhalla
