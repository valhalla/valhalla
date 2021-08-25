#include "loki/search.h"
#include "baldr/graphconstants.h"
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

template <typename T> inline T square(T v) {
  return v * v;
}

bool is_search_filter_triggered(const DirectedEdge* edge,
                                const DynamicCost& costing,
                                const graph_tile_ptr& tile,
                                const Location::SearchFilter& search_filter) {
  // check if this edge matches any of the exclusion filters
  uint32_t road_class = static_cast<uint32_t>(edge->classification());
  uint32_t min_road_class = static_cast<uint32_t>(search_filter.min_road_class_);
  uint32_t max_road_class = static_cast<uint32_t>(search_filter.max_road_class_);

  // Note that min_ and max_road_class are integers where, by default, max_road_class
  // is 0 and min_road_class is 7. This filter rejects roads where the functional
  // road class is outside of the min to max range.
  return (road_class > min_road_class || road_class < max_road_class) ||
         (search_filter.exclude_tunnel_ && edge->tunnel()) ||
         (search_filter.exclude_bridge_ && edge->bridge()) ||
         (search_filter.exclude_ramp_ && (edge->use() == Use::kRamp)) ||
         (search_filter.exclude_closures_ && (costing.flow_mask() & kCurrentFlowMask) &&
          tile->IsClosed(edge));
}

bool side_filter(const PathLocation::PathEdge& edge, const Location& location, GraphReader& reader) {
  // nothing to filter if you dont want to filter or if there is no side of street
  if (edge.sos == PathLocation::SideOfStreet::NONE ||
      location.preferred_side_ == Location::PreferredSide::EITHER)
    return false;

  // need the driving side for this edge
  graph_tile_ptr tile;
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

bool heading_filter(const Location& location, float angle) {
  // no heading means we filter nothing
  if (!location.heading_) {
    return false;
  }

  // we want the closest distance between two angles which can be had
  // across 0 or between the two so we just need to know which is bigger
  if (*location.heading_ > angle) {
    return std::min(*location.heading_ - angle, (360.f - *location.heading_) + angle) >
           location.heading_tolerance_;
  }
  return std::min(angle - *location.heading_, (360.f - angle) + *location.heading_) >
         location.heading_tolerance_;
}

bool layer_filter(const Location& location, int8_t layer) {
  // no layer - we do not filter
  if (!location.preferred_layer_) {
    return false;
  }

  return *location.preferred_layer_ != layer;
}

PathLocation::SideOfStreet flip_side(const PathLocation::SideOfStreet side) {
  if (side != PathLocation::SideOfStreet::NONE) {
    return side == PathLocation::SideOfStreet::LEFT ? PathLocation::SideOfStreet::RIGHT
                                                    : PathLocation::SideOfStreet::LEFT;
  }
  return side;
}

std::function<std::tuple<int32_t, unsigned short, double>()> make_binner(const PointLL& p) {
  const auto& tiles = TileHierarchy::levels().back().tiles;
  return tiles.ClosestFirst(p);
}

// Model a segment (2 consecutive points in an edge in a bin).
struct candidate_t {
  double sq_distance{};
  PointLL point;
  size_t index{};
  bool prefiltered{};

  GraphId edge_id;
  const DirectedEdge* edge{};
  std::shared_ptr<const EdgeInfo> edge_info;

  graph_tile_ptr tile;

  bool operator<(const candidate_t& c) const {
    return sq_distance < c.sq_distance;
  }

  PathLocation::SideOfStreet get_side(const PointLL& original,
                                      float tang_angle,
                                      double sq_distance,
                                      double sq_tolerance,
                                      double sq_max_distance) const {
    // point is so close to the edge that its basically on the edge,
    // or its too far from the edge that we shouldn't use it to determine side of street
    if (sq_distance < sq_tolerance || sq_distance > sq_max_distance) {
      return PathLocation::SideOfStreet::NONE;
    }

    // get the absolute angle between the snap point and the provided point
    auto angle_to_point = point.Heading(original);

    // add 360 degrees if angle becomes negative
    auto angle_diff = angle_to_point - tang_angle;
    if (angle_diff < 0) {
      angle_diff += 360.f;
    }

    // 10 degrees on either side is considered to be straight ahead
    constexpr float angle_tolerance = 10.f;

    /* check which side the point falls in:
       If angle_diff is between 10 and 170 it's on the right side,
       if angle_diff is between 190 and 350 it's on the left side,
       otherwise it's practically straight ahead or behind.

             \    L    /
              \       /
        - - - - - x - - - - -
              /       \
             /    R    \
    */
    if (angle_diff > angle_tolerance && angle_diff < (180.f - angle_tolerance)) {
      return PathLocation::SideOfStreet::RIGHT;
    } else if (angle_diff > (180.f + angle_tolerance) && angle_diff < (360.f - angle_tolerance)) {
      return PathLocation::SideOfStreet::LEFT;
    } else {
      return PathLocation::SideOfStreet::NONE;
    }
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
      : binner(make_binner(location.latlng_)), location(location),
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
      return cur_tile.get() > other.cur_tile.get();
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
      double distance;
      std::tie(tile_index, bin_index, distance) = binner();
      if (distance > location.search_cutoff_ ||
          (reachable.size() && distance > location.radius_ &&
           distance > std::sqrt(reachable.back().sq_distance))) {
        cur_tile = nullptr;
        break;
      }

      // grab the tile the lat, lon is in
      auto tile_id = GraphId(tile_index, TileHierarchy::levels().back().level, 0);
      reader.GetGraphTile(tile_id, cur_tile);
    } while (!cur_tile);
  }

  std::function<std::tuple<int32_t, unsigned short, double>()> binner;
  graph_tile_ptr cur_tile;
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
  std::shared_ptr<DynamicCost> costing;
  unsigned int max_reach_limit;
  std::vector<candidate_t> bin_candidates;
  std::unordered_set<uint64_t> correlated_edges;
  Reach reach_finder;

  // keep track of edges whose reachability we've already computed
  // TODO: dont use pointers as keys, its safe for now but fancy caching one day could be bad
  std::unordered_map<const DirectedEdge*, directed_reach> directed_reaches;

  bin_handler_t(const std::vector<valhalla::baldr::Location>& locations,
                valhalla::baldr::GraphReader& reader,
                const std::shared_ptr<DynamicCost>& costing)
      : reader(reader), costing(costing) {
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
    double distance = std::numeric_limits<double>::lowest();
    std::function<void(const GraphId& node_id, bool transition)> crawl;
    crawl = [&](const GraphId& node_id, bool follow_transitions) {
      // now that we have a node we can pass back all the edges leaving and entering it
      auto tile = reader.GetGraphTile(node_id);
      if (!tile) {
        return;
      }
      const auto* node = tile->node(node_id);
      const auto* start_edge = tile->directededge(node->edge_index());
      const auto* end_edge = start_edge + node->edge_count();
      PointLL node_ll = node->latlng(tile->header()->base_ll());
      // cache the distance
      if (distance == std::numeric_limits<double>::lowest())
        distance = node_ll.Distance(location.latlng_);
      // add edges entering/leaving this node
      for (const auto* edge = start_edge; edge < end_edge; ++edge) {
        // get some info about this edge and the opposing
        GraphId id = tile->id();
        id.set_id(node->edge_index() + (edge - start_edge));
        auto info = tile->edgeinfo(edge);

        // calculate the heading of the snapped point to the shape for use in heading filter
        size_t index = edge->forward() ? 0 : info.shape().size() - 2;
        float angle =
            tangent_angle(index, candidate.point, info.shape(),
                          GetOffsetForHeading(edge->classification(), edge->use()), edge->forward());
        auto layer = info.layer();
        // do we want this edge
        if (costing->Allowed(edge, tile, kDisallowShortcut)) {
          auto reach = get_reach(id, edge);
          PathLocation::PathEdge
              path_edge{id, 0, node_ll, distance, PathLocation::NONE, reach.outbound, reach.inbound};
          if (heading_filter(location, angle) || layer_filter(location, layer)) {
            filtered.emplace_back(std::move(path_edge));
          } else if (correlated_edges.insert(path_edge.id).second) {
            correlated.edges.push_back(std::move(path_edge));
          }
        }

        // do we want the evil twin
        const DirectedEdge* other_edge = nullptr;
        graph_tile_ptr other_tile;
        const auto other_id = reader.GetOpposingEdgeId(id, other_edge, other_tile);
        if (!other_edge)
          continue;

        if (costing->Allowed(other_edge, other_tile, kDisallowShortcut)) {
          auto reach = get_reach(other_id, other_edge);
          PathLocation::PathEdge path_edge{other_id,
                                           1,
                                           node_ll,
                                           distance,
                                           PathLocation::NONE,
                                           reach.outbound,
                                           reach.inbound};
          // angle is 180 degrees opposite direction of the one above
          if (heading_filter(location, std::fmod(angle + 180.f, 360.f)) ||
              layer_filter(location, layer)) {
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
      // TODO: length of the edge only has meters resolution, either store more precision or
      // measure the rest of the shapes length
      partial_length = std::min(partial_length, static_cast<double>(candidate.edge->length()));
      double length_ratio = partial_length / candidate.edge->length();
      if (!candidate.edge->forward()) {
        length_ratio = 1.f - length_ratio;
      }
      // calculate the heading of the snapped point to the shape for use in heading
      // filter and side of street calculation
      float angle =
          tangent_angle(candidate.index, candidate.point, candidate.edge_info->shape(),
                        GetOffsetForHeading(candidate.edge->classification(), candidate.edge->use()),
                        candidate.edge->forward());
      auto layer = candidate.edge_info->layer();
      auto sq_tolerance = square(double(location.street_side_tolerance_));
      auto sq_max_distance = square(double(location.street_side_max_distance_));
      auto side =
          candidate.get_side(location.display_latlng_ ? *location.display_latlng_ : location.latlng_,
                             angle,
                             location.display_latlng_
                                 ? location.display_latlng_->DistanceSquared(candidate.point)
                                 : candidate.sq_distance,
                             sq_tolerance, sq_max_distance);
      auto reach = get_reach(candidate.edge_id, candidate.edge);
      PathLocation::PathEdge path_edge{candidate.edge_id, length_ratio, candidate.point,
                                       distance,          side,         reach.outbound,
                                       reach.inbound};
      // correlate the edge we found
      if (side_filter(path_edge, location, reader) || heading_filter(location, angle) ||
          layer_filter(location, layer)) {
        filtered.push_back(std::move(path_edge));
      } else if (correlated_edges.insert(candidate.edge_id).second) {
        correlated.edges.push_back(std::move(path_edge));
      }
      // correlate its evil twin
      const DirectedEdge* other_edge = nullptr;
      graph_tile_ptr other_tile;
      auto opposing_edge_id = reader.GetOpposingEdgeId(candidate.edge_id, other_edge, other_tile);

      if (other_edge && costing->Allowed(other_edge, other_tile, kDisallowShortcut)) {
        reach = get_reach(opposing_edge_id, other_edge);
        PathLocation::PathEdge other_path_edge{opposing_edge_id, 1 - length_ratio, candidate.point,
                                               distance,         flip_side(side),  reach.outbound,
                                               reach.inbound};
        // angle is 180 degrees opposite of the one above
        if (side_filter(other_path_edge, location, reader) ||
            heading_filter(location, std::fmod(angle + 180.f, 360.f)) ||
            layer_filter(location, layer)) {
          filtered.push_back(std::move(other_path_edge));
        } else if (correlated_edges.insert(opposing_edge_id).second) {
          correlated.edges.push_back(std::move(other_path_edge));
        }
      }
    }
  }

  directed_reach get_reach(const GraphId edge_id, const DirectedEdge* edge) {
    // if its in cache return it
    auto itr = directed_reaches.find(edge);
    if (itr != directed_reaches.cend())
      return itr->second;

    // notice we do both directions here because in the end we use this reach for all input locations
    auto reach = reach_finder(edge, edge_id, max_reach_limit, reader, costing, kInbound | kOutbound);
    directed_reaches[edge] = reach;
    return reach;
  }

  // do a mini network expansion or maybe not
  directed_reach check_reachability(std::vector<projector_wrapper>::iterator begin,
                                    std::vector<projector_wrapper>::iterator end,
                                    graph_tile_ptr tile,
                                    const DirectedEdge* edge,
                                    const GraphId edge_id) {
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
    auto reach = reach_finder(edge, edge_id, max_reach_limit, reader, costing, kInbound | kOutbound);
    directed_reaches[edge] = reach;

    // if the inbound reach is not 0 and the outbound reach is not 0 and the opposing edge is not
    // filtered then the reaches of both edges are the same

    const DirectedEdge* opp_edge = nullptr;
    if (reach.outbound > 0 && reach.inbound > 0 && (opp_edge = reader.GetOpposingEdge(edge, tile)) &&
        costing->Allowed(opp_edge, tile, kDisallowShortcut)) {
      directed_reaches[opp_edge] = reach;
    }
    return reach;
  }

  // handle a bin for the range of candidates that share it
  void handle_bin(std::vector<projector_wrapper>::iterator begin,
                  std::vector<projector_wrapper>::iterator end) {
    // iterate over the edges in the bin
    auto tile = begin->cur_tile;
    auto edges = tile->GetBin(begin->bin_index);
    for (auto edge_id : edges) {
      // get the tile and edge
      if (!reader.GetGraphTile(edge_id, tile)) {
        continue;
      }

      // if this edge is filtered
      const auto* edge = tile->directededge(edge_id);
      if (!costing->Allowed(edge, tile, kDisallowShortcut)) {
        // then we try its opposing edge
        edge_id = reader.GetOpposingEdgeId(edge_id, edge, tile);
        // but if we couldnt get it or its filtered too then we move on
        if (!edge_id.Is_Valid() || !costing->Allowed(edge, tile, kDisallowShortcut))
          continue;
      }

      // initialize candidates vector:
      // - reset sq_distance to max so we know the best point along the edge
      // - apply prefilters based on user's SearchFilter request options
      auto c_itr = bin_candidates.begin();
      decltype(begin) p_itr;
      bool all_prefiltered = true;
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
        c_itr->sq_distance = std::numeric_limits<double>::max();
        c_itr->prefiltered =
            is_search_filter_triggered(edge, *costing, tile, p_itr->location.search_filter_);
        // set to false if even one candidate was not filtered
        all_prefiltered = all_prefiltered && c_itr->prefiltered;
      }

      // short-circuit if all candidates were prefiltered
      if (all_prefiltered) {
        continue;
      }

      // TODO: can we speed this up? the majority of edges will be short and far away enough
      // such that the closest point on the edge will be one of the edges end points, we can get
      // these coordinates them from the nodes in the graph. we can then find whichever end is
      // closest to the input point p, call it n. we can then define an half plane h intersecting n
      // so that its orthogonal to the ray from p to n. using h, we only need to test segments
      // of the shape which are on the same side of h that p is. to make this fast we would need a
      // a trivial half plane test as maybe a single dot product and comparison?

      // get some shape of the edge
      auto edge_info = std::make_shared<const EdgeInfo>(tile->edgeinfo(edge));
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
          // skip updating this candidate because it was prefiltered
          if (c_itr->prefiltered) {
            continue;
          }
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
      auto reach = check_reachability(begin, end, tile, edge, edge_id);

      // keep the best point along this edge if it makes sense
      c_itr = bin_candidates.begin();
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
        // skip updating this candidate because it was prefiltered
        if (c_itr->prefiltered) {
          continue;
        }
        // is this edge reachable in the right way
        bool reachable = reach.outbound >= p_itr->location.min_outbound_reach_ &&
                         reach.inbound >= p_itr->location.min_inbound_reach_;
        const DirectedEdge* opp_edge = nullptr;
        graph_tile_ptr opp_tile = tile;
        GraphId opp_edgeid;
        // it's possible that it isnt reachable but the opposing is, switch to that if so
        if (!reachable && (opp_edgeid = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)) &&
            costing->Allowed(opp_edge, opp_tile, kDisallowShortcut)) {
          auto opp_reach = check_reachability(begin, end, opp_tile, opp_edge, opp_edgeid);
          if (opp_reach.outbound >= p_itr->location.min_outbound_reach_ &&
              opp_reach.inbound >= p_itr->location.min_inbound_reach_) {
            tile = opp_tile;
            edge = opp_edge;
            edge_id = opp_edgeid;
            reach = opp_reach;
            reachable = true;
          }
        }

        // which batch of findings will this go into
        auto* batch = reachable ? &p_itr->reachable : &p_itr->unreachable;

        // if its empty append
        if (batch->empty()) {
          c_itr->edge = edge;
          c_itr->edge_id = edge_id;
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
          c_itr->edge_id = edge_id;
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
          graph_tile_ptr other_tile;
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
        for (auto&& path_edge : filtered) {
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
       const std::shared_ptr<DynamicCost>& costing) {
  // we cannot continue without costing
  if (!costing)
    throw std::runtime_error("No costing was provided for edge candidate search");

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
