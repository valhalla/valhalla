#include "loki/search.h"
#include "midgard/linesegment2.h"
#include "midgard/distanceapproximator.h"
#include "baldr/tilehierarchy.h"

#include <unordered_set>
#include <list>
#include <math.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace {
//the cutoff at which we will assume the input is too far away from civilisation to be
//worth correlating to the nearest graph elements
constexpr float SEARCH_CUTOFF = 35000.f;
//during edge correlation, if you end up < 5 meters from the beginning or end of the
//edge we just assume you were at that node and not actually along the edge
//we keep it small because point and click interfaces are more accurate than gps input
constexpr float NODE_SNAP = 5.f;
//during side of street computations we figured you're on the street if you are less than
//5 meters (16) feet from the centerline. this is actually pretty large (with accurate shape
//data for the roads it might want half that) but its better to assume on street than not
constexpr float SIDE_OF_STREET_SNAP = 5.f;
//if you are this far away from the edge we are considering and you set a heading we will
//ignore it because its not really useful at this distance from the geometry
constexpr float NO_HEADING = 30.f;
//how much of the shape should be sampled to get heading
constexpr float HEADING_SAMPLE = 30.f;
//cone width to use for cosine similarity comparisons for favoring heading
constexpr float DEFAULT_ANGLE_WIDTH = 60.f;

std::function<std::tuple<int32_t, unsigned short, float>()>
make_binner(const PointLL& p, const GraphReader& reader) {
  const auto& tiles = TileHierarchy::levels().rbegin()->second.tiles;
  return tiles.ClosestFirst(p);
}

// Model a segment (2 consecutive points in an edge in a bin).
struct candidate_t {
  float sq_distance;
  PointLL point;
  size_t index;

  GraphId edge_id;
  const DirectedEdge* edge;
  std::shared_ptr<const EdgeInfo> edge_info;

  const GraphTile* tile;
};

// This structure contains the context of the projection of a
// Location.  At the creation, a bin is affected to the point.  The
// test() method should be called to each valid segment of the bin.
// When the bin is finished, next_bin() switch to the next possible
// interesting bin.  if has_bin() is false, then the best projection
// is found.
struct projector_t {
  projector_t(const Location& location, GraphReader& reader):
    binner(make_binner(location.latlng_, reader)),
    location(location), sq_radius(location.radius_ * location.radius_),
    lon_scale(cosf(location.latlng_.lat() * kRadPerDeg)),
    lat(location.latlng_.lat()),
    lng(location.latlng_.lng()),
    approx(location.latlng_) {
    //TODO: something more empirical based on radius
    isolated.reserve(64);
    unisolated.reserve(64);
    //initialize
    next_bin(reader);
  }

  // non default constructible and move only type
  projector_t() = delete;
  projector_t(const projector_t&) = delete;
  projector_t& operator=(const projector_t&) = delete;
  projector_t(projector_t&&) = default;
  projector_t& operator=(projector_t&&) = default;

  //the ones marked with null current tile are finished so put them on the end
  //otherwise we sort by bin so that ones with the same bin are next to each other
  bool operator<(const projector_t& other) const {
    //the
    if (cur_tile != other.cur_tile)
      return cur_tile > other.cur_tile;
    return bin_index < other.bin_index;
  }

  bool has_same_bin(const projector_t& other) const {
    return cur_tile == other.cur_tile && bin_index == other.bin_index;
  }

  bool has_bin() const {
    return cur_tile != nullptr;
  }

  // Advance to the next bin. Must not be called if has_bin() is false.
  void next_bin(GraphReader& reader) {
    do {
      //TODO: make configurable the radius at which we give up searching
      //the closest thing in this bin is further than what we have already
      int32_t tile_index; float distance;
      std::tie(tile_index, bin_index, distance) = binner();
      if(distance > SEARCH_CUTOFF || (unisolated.size() && distance > sqrt(unisolated.back().sq_distance))) {
        cur_tile = nullptr;
        break;
      }

      //grab the tile the lat, lon is in
      auto tile_id = GraphId(tile_index, TileHierarchy::levels().rbegin()->first, 0);
      reader.GetGraphTile(tile_id, cur_tile);
    } while (!cur_tile);
  }

  // Test if a segment is a candidate to the projection.  This method
  // is performance critical.  Copy, function call, cache locality and
  // useless computation must be handled with care.
  void project(const PointLL& u, const PointLL& v, PointLL& p) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    auto bx = v.first - u.first;
    auto by = v.second - u.second;

    // Scale longitude when finding the projection. Avoid divided-by-zero
    // which gives a NaN scale, otherwise comparisons below will fail
    auto bx2 = bx * lon_scale;
    auto sq = bx2*bx2 + by*by;
    auto scale = sq > 0 ? ((lng - u.lng())*lon_scale*bx2 + (lat - u.lat())*by) / sq : 0.f;

    //projects along the ray before u
    if(scale <= 0.f) {
      p.first = u.first;
      p.second = u.second;
    }//projects along the ray after v
    else if(scale >= 1.f){
      p.first = v.first;
      p.second = v.second;
    }//projects along the ray between u and v
    else {
      p.first = u.first + bx * scale;
      p.second = u.second + by * scale;
    }
  }

  std::function<std::tuple<int32_t, unsigned short, float>()> binner;
  const GraphTile* cur_tile = nullptr;
  Location location;
  unsigned short bin_index = 0;
  float sq_radius;
  std::vector<candidate_t> isolated;
  std::vector<candidate_t> unisolated;

  // critical data
  float lon_scale;
  float lat;
  float lng;
  DistanceApproximator approx;
};

//TODO: move this to midgard and test the crap out of it
//we are essentially estimating the angle of the tangent
//at a point along a discretised curve. we attempt to mostly
//use the shape coming into the point on the curve but if there
//isnt enough there we will use the shape coming out of the it
float tangent_angle(size_t index, const PointLL& point, const std::vector<PointLL>& shape, bool forward) {
  //depending on if we are going forward or backward we choose a different increment
  auto increment = forward ? -1 : 1;
  auto first_end = forward ? shape.cbegin() : shape.cend() - 1 ;
  auto second_end = forward ? shape.cend() - 1 : shape.cbegin();

  //u and v will be points we move along the shape until we have enough distance between them or run out of points

  //move backwards until we have enough or run out
  float remaining = HEADING_SAMPLE;
  auto u = point;
  auto i = shape.cbegin() + index + forward;
  while(remaining > 0 && i != first_end) {
    //move along and see how much distance that added
    i += increment;
    auto d = u.Distance(*i);
    //are we done yet?
    if(remaining <= d) {
      auto coef = remaining / d;
      u = u.AffineCombination(1 - coef, coef, *i);
      return u.Heading(point);
    }
    //next one
    u = *i;
    remaining -= d;
  }

  //move forwards until we have enough or run out
  auto v = point;
  i = shape.cbegin() + index + !forward;
  while(remaining > 0 && i != second_end) {
    //move along and see how much distance that added
    i -= increment;
    auto d = v.Distance(*i);
    //are we done yet?
    if(remaining <= d) {
      auto coef = remaining / d;
      v = v.AffineCombination(1 - coef, coef, *i);
      return u.Heading(v);
    }
    //next one
    v = *i;
    remaining -= d;
  }

  return u.Heading(v);
}

bool heading_filter(const DirectedEdge* edge, const EdgeInfo& info,
  const Location& location, const PointLL& point, float distance, size_t index) {
  //if its far enough away from the edge, the heading is pretty useless
  if(!location.heading_ || distance > NO_HEADING)
    return false;

  //get the angle of the shape from this point
  auto angle = tangent_angle(index, point, info.shape(), edge->forward());
  //we want the closest distance between two angles which can be had
  //across 0 or between the two so we just need to know which is bigger
  if(*location.heading_ > angle)
    return std::min(*location.heading_ - angle, (360.f - *location.heading_) + angle) > location.heading_tolerance_.get_value_or(DEFAULT_ANGLE_WIDTH);
  return std::min(angle - *location.heading_, (360.f - angle) + *location.heading_) > location.heading_tolerance_.get_value_or(DEFAULT_ANGLE_WIDTH);
}

PathLocation::SideOfStreet flip_side(const PathLocation::SideOfStreet side) {
  if(side != PathLocation::SideOfStreet::NONE)
    return side == PathLocation::SideOfStreet::LEFT ? PathLocation::SideOfStreet::RIGHT : PathLocation::SideOfStreet::LEFT;
  return side;
}

PathLocation::SideOfStreet get_side(const candidate_t& candidate, const PointLL& original, float distance){
  //its so close to the edge that its basically on the edge
  if(distance < SIDE_OF_STREET_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //if the projected point is way too close to the begin or end of the shape
  //TODO: if the original point is really far away side of street may also not make much sense..
  auto& shape = candidate.edge_info->shape();
  if(candidate.point.Distance(shape.front()) < SIDE_OF_STREET_SNAP ||
     candidate.point.Distance(shape.back())  < SIDE_OF_STREET_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //get the side TODO: this can technically fail for longer segments..
  //to fix it we simply compute the plane formed by the triangle
  //through the center of the earth and the two shape points and test
  //whether the original point is above or below the plane (depending on winding)
  LineSegment2<PointLL> segment(shape[candidate.index], shape[candidate.index + 1]);
  return (segment.IsLeft(original) > 0) == candidate.edge->forward()  ? PathLocation::SideOfStreet::LEFT : PathLocation::SideOfStreet::RIGHT;
}

PathLocation correlate_node(GraphReader& reader, const Location& location, const EdgeFilter& edge_filter, const GraphId& found_node, const candidate_t& candidate){
  PathLocation correlated(location);
  auto distance = location.latlng_.Distance(candidate.point);
  std::list<PathLocation::PathEdge> heading_filtered;

  //we need this because we might need to go to different levels
  std::function<void (const GraphId& node_id, bool transition)> crawl;
  crawl = [&](const GraphId& node_id, bool follow_transitions) {
    //now that we have a node we can pass back all the edges leaving and entering it
    const auto* tile = reader.GetGraphTile(node_id);
    if(!tile)
      return;
    const auto* node = tile->node(node_id);
    const auto* start_edge = tile->directededge(node->edge_index());
    const auto* end_edge = start_edge + node->edge_count();
    for(const auto* edge = start_edge; edge < end_edge; ++edge) {
      //if this is an edge leaving this level then we should go do that level awhile
      if(follow_transitions && (edge->trans_down() || edge->trans_up())) {
        crawl(edge->endnode(), false);
        continue;
      }

      //get some info about this edge and the opposing
      GraphId id = tile->id();
      id.fields.id = node->edge_index() + (edge - start_edge);
      auto info = tile->edgeinfo(edge->edgeinfo_offset());

      //do we want this edge
      if(edge_filter(edge) != 0.0f) {
        PathLocation::PathEdge path_edge{std::move(id), 0.f, node->latlng(), distance, PathLocation::NONE};
        auto index = edge->forward() ? 0 : info.shape().size() - 2;
        if(!heading_filter(edge, info, location, candidate.point, distance, index))
          correlated.edges.push_back(std::move(path_edge));
        else
          heading_filtered.emplace_back(std::move(path_edge));
      }

      //do we want the evil twin
      const GraphTile* other_tile;
      const auto other_id = reader.GetOpposingEdgeId(id, other_tile);
      if(!other_tile)
        continue;
      const auto* other_edge = other_tile->directededge(other_id);
      if(edge_filter(other_edge) != 0.0f) {
        PathLocation::PathEdge path_edge{std::move(other_id), 1.f, node->latlng(), distance, PathLocation::NONE};
        auto index = other_edge->forward() ? 0 : info.shape().size() - 2;
        if(!heading_filter(other_edge, tile->edgeinfo(edge->edgeinfo_offset()), location, candidate.point, distance, index))
          correlated.edges.push_back(std::move(path_edge));
        else
          heading_filtered.emplace_back(std::move(path_edge));
      }
    }

    //if we have nothing because of heading we'll just ignore it
    if(correlated.edges.size() == 0 && heading_filtered.size())
      for(auto& path_edge : heading_filtered)
        correlated.edges.push_back(std::move(path_edge));

    //if we still found nothing that is no good..
    if(correlated.edges.size() == 0)
      throw std::runtime_error("No suitable edges near location");
  };

  //start where we are and crawl from there
  crawl(found_node, true);

  //if it was a through location with a heading its pretty confusing.
  //does the user want to come into and exit the location at the preferred
  //angle? for now we are just saying that they want it to exit at the
  //heading provided. this means that if it was node snapped we only
  //want the outbound edges
  if(location.stoptype_ == Location::StopType::THROUGH && location.heading_) {
    auto new_end = std::remove_if(correlated.edges.begin(), correlated.edges.end(),
      [](const PathLocation::PathEdge& e) { return e.end_node(); });
    correlated.edges.erase(new_end, correlated.edges.end());
  }

  if(correlated.edges.size() == 0)
    throw std::runtime_error("No suitable edges near location");

  //give it back
  return correlated;
}

PathLocation correlate_edge(GraphReader& reader, const Location& location, const EdgeFilter& edge_filter, const candidate_t& candidate) {
  //now that we have an edge we can pass back all the info about it
  PathLocation correlated(location);
  auto distance = location.latlng_.Distance(candidate.point);
  if(candidate.edge != nullptr){
    //we need the ratio in the direction of the edge we are correlated to
    double partial_length = 0;
    for(size_t i = 0; i < candidate.index; ++i)
      partial_length += candidate.edge_info->shape()[i].Distance(candidate.edge_info->shape()[i + 1]);
    partial_length += candidate.edge_info->shape()[candidate.index].Distance(candidate.point);
    partial_length = std::min(partial_length, static_cast<double>(candidate.edge->length()));
    float length_ratio = static_cast<float>(partial_length / static_cast<double>(candidate.edge->length()));
    if(!candidate.edge->forward())
      length_ratio = 1.f - length_ratio;
    //side of street
    auto side = get_side(candidate, location.latlng_, distance);
    //correlate the edge we found
    std::list<PathLocation::PathEdge> heading_filtered;
    if(heading_filter(candidate.edge, *candidate.edge_info, location, candidate.point, distance, candidate.index))
      heading_filtered.emplace_back(candidate.edge_id, length_ratio, candidate.point, side);
    else
      correlated.edges.push_back(PathLocation::PathEdge{candidate.edge_id, length_ratio, candidate.point, distance, side});
    //correlate its evil twin
    const GraphTile* other_tile;
    auto opposing_edge_id = reader.GetOpposingEdgeId(candidate.edge_id, other_tile);
    const DirectedEdge* other_edge;
    if(opposing_edge_id.Is_Valid() && (other_edge = other_tile->directededge(opposing_edge_id)) && edge_filter(other_edge) != 0.0f) {
      if(heading_filter(other_edge, *candidate.edge_info, location, candidate.point, distance, candidate.index))
        heading_filtered.emplace_back(opposing_edge_id, 1 - length_ratio, candidate.point, distance, flip_side(side));
      else
        correlated.edges.push_back(PathLocation::PathEdge{opposing_edge_id, 1 - length_ratio, candidate.point, distance, flip_side(side)});
    }

    //if we have nothing because of heading we'll just ignore it
    if(correlated.edges.size() == 0 && heading_filtered.size())
      for(auto& path_edge : heading_filtered)
        correlated.edges.push_back(std::move(path_edge));
  }

  //if we found nothing that is no good..
  if(correlated.edges.size() == 0)
    throw std::runtime_error("No suitable edges near location");

  //give it back
  return correlated;
}

struct bin_handler_t {
  std::vector<projector_t> pps;
  valhalla::baldr::GraphReader& reader;
  const EdgeFilter& edge_filter;
  const NodeFilter& node_filter;
  std::unordered_map<uint64_t, unsigned int**> visited;
  unsigned int max_isolated;
  std::vector<candidate_t> bin_candidates;

  bin_handler_t(const std::vector<valhalla::baldr::Location>& locations, valhalla::baldr::GraphReader& reader,
    const EdgeFilter& edge_filter, const NodeFilter& node_filter):
    reader(reader), edge_filter(edge_filter), node_filter(node_filter) {
    //get the unique set of input locations and the max isolation level of them all
    std::unordered_set<Location> uniq_locations(locations.begin(), locations.end());
    pps.reserve(uniq_locations.size());
    max_isolated = 0;
    for (const auto& loc: uniq_locations) {
      pps.emplace_back(loc, reader);
      max_isolated = std::max(max_isolated, loc.isolated_);
    }
    //very annoying but it saves a lot of time to preallocate this instead of doing it in the loop in handle_bins
    bin_candidates.resize(pps.size());
  }

  ~bin_handler_t(){
    //this is faster than using any of the managed pointers
    for(auto& kv : visited) {
      if(*kv.second) {
        delete *kv.second;
        *kv.second = nullptr;
      }
    }
  }

  // Find the best range to do.  The given vector should be sorted for
  // interesting grouping.  Returns the greatest range of non empty
  // equal bins.
  std::pair<std::vector<projector_t>::iterator, std::vector<projector_t>::iterator>
  find_best_range(std::vector<projector_t>& pps) const {
    auto best = std::make_pair(pps.begin(), pps.begin());
    auto cur = best;
    while (cur.second != pps.end()) {
      cur.first = cur.second;
      cur.second = std::find_if_not(cur.first, pps.end(), [&cur](const projector_t& pp) {
        return cur.first->has_same_bin(pp);
      });
      if (cur.first->has_bin() && cur.second - cur.first > best.second - best.first) {
        best = cur;
      }
    }
    return best;
  }

  //we keep the points sorted at each round such that unfinished ones
  //are at the front of the sorted list
  void search() {
    std::sort(pps.begin(), pps.end());
    while(pps.front().has_bin()) {
      auto range = find_best_range(pps);
      handle_bin(range.first, range.second);
      std::sort(pps.begin(), pps.end());
    }
  }

  //recursive depth first search for expanding edges
  //TODO: test whether writing this iteratively would be faster
  //ie use a vector instead of the call stack to keep state
  void depth_first(const GraphTile*& tile, const DirectedEdge* edge, unsigned int** cardinality) {
    //grab the end node tile
    if(!reader.GetGraphTile(edge->endnode(), tile))
      return;
    //can we go through its node
    const auto* node = tile->node(edge->endnode());
    if(node_filter(node))
      return;
    //for each edge recurse on the usable ones
    auto id = tile->id(); id.fields.id = node->edge_index();
    auto* e = tile->directededge(node->edge_index());
    for(uint32_t i = 0; **cardinality < max_isolated && i < node->edge_count(); ++i, ++e, ++id) {
      //check if this edge is usable
      if(edge_filter(e) != 0.f) {
        //try to mark it, TODO: dont count transition edges and opposing
        auto inserted = visited.emplace(id, cardinality);
        //we saw this one already
        if(!inserted.second) {
          //we saw this edge in a previous run so we are done
          if(*cardinality != *inserted.first->second) {
            delete *cardinality;
            *cardinality = *inserted.first->second;
            return;
          }
          //we saw this edge in this run so just skip it
          continue;
        }
        //recurse
        ++(**cardinality);
        auto* previous = *cardinality;
        depth_first(tile, e, cardinality);

        //if we saw the edge in a previous run we want to be done completely
        if(*cardinality != previous)
          return;
      }
    }
  }

  unsigned int check_isolation(std::vector<projector_t>::iterator begin, std::vector<projector_t>::iterator end,
    const GraphTile* tile, const DirectedEdge* edge, const GraphId& id) {
    //no need when set to 0
    if(max_isolated == 0)
      return 1;

    //do we already know about this one?
    auto found = visited.find(id);
    if(found != visited.cend())
      return **found->second;

    //see if we even need to do it
    bool deisolate = false;
    for (auto it = begin; it != end && !deisolate; ++it)
      deisolate = deisolate || it->unisolated.empty();

    //we do have to go check then
    if (deisolate) {
      unsigned int** cardinality = new unsigned int*[1];
      *cardinality = new unsigned int{0};
      depth_first(tile, edge, cardinality);
      return **cardinality;
    }

    //we didnt have to do any work so we just assume its not isolated
    return max_isolated + 1;
  }

  //handle a bin for the range of candidates that share it
  void handle_bin(std::vector<projector_t>::iterator begin,
                  std::vector<projector_t>::iterator end) {
    //iterate over the edges in the bin
    auto tile = begin->cur_tile;
    auto edges = tile->GetBin(begin->bin_index);
    for(auto e : edges) {
      //get the tile and edge
      if(!reader.GetGraphTile(e, tile))
        continue;

      //no thanks on this one or its evil twin
      const auto* edge = tile->directededge(e);
      if(edge_filter(edge) == 0.0f &&
        (!(e = reader.GetOpposingEdgeId(e, tile)).Is_Valid() ||
        edge_filter(edge = tile->directededge(e)) == 0.0f)) {
        continue;
      }

      //check for island or dont
      auto isolation = check_isolation(begin, end, tile, edge, e);

      //get some shape of the edge
      auto edge_info = std::make_shared<const EdgeInfo>(tile->edgeinfo(edge->edgeinfo_offset()));
      auto shape = edge_info->lazy_shape();
      PointLL v;
      if (!shape.empty())
        v = shape.pop();

      //reset these so we know the best point along the edge
      auto c_itr = bin_candidates.begin();
      decltype(begin) p_itr;
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr)
        c_itr->sq_distance = std::numeric_limits<float>::max();

      //iterate along this edges segments projecting each of the points
      for(size_t i = 0; !shape.empty(); ++i) {
        auto u = v;
        v = shape.pop();
        //for each input point
        c_itr = bin_candidates.begin();
        for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
          //how close is the input to this segment
          PointLL point;
          p_itr->project(u, v, point);
          auto sq_distance = p_itr->approx.DistanceSquared(point);
          //do we want to keep it
          if(sq_distance < c_itr->sq_distance) {
            c_itr->sq_distance = sq_distance;
            c_itr->point = std::move(point);
            c_itr->index = i;
          }
        }
      }

      //keep the best point along this edge if it makes sense
      c_itr = bin_candidates.begin();
      for (p_itr = begin; p_itr != end; ++p_itr, ++c_itr) {
        //which batch of findings
        auto* batch = isolation > p_itr->location.isolated_ ? &p_itr->unisolated : &p_itr->isolated;

        //if its empty append
        if(batch->empty()) {
          c_itr->edge = edge; c_itr->edge_id = e; c_itr->edge_info = edge_info; c_itr->tile = tile;
          batch->emplace_back(std::move(*c_itr));
          continue;
        }

        //get some info about possibilities
        bool in_radius = c_itr->sq_distance < p_itr->sq_radius;
        bool better = c_itr->sq_distance < batch->back().sq_distance;
        bool last_in_radius = batch->back().sq_distance < p_itr->sq_radius;

        //it has to either be better or in the radius to move on
        if(in_radius || better) {
          c_itr->edge = edge; c_itr->edge_id = e; c_itr->edge_info = edge_info; c_itr->tile = tile;
          //the last one wasnt in the radius so replace it with this one because its better or is in the radius
          if(!last_in_radius)
            batch->back() = std::move(*c_itr);
          //last one is in the radius but this one is better so put it on the end
          else if(better)
            batch->emplace_back(std::move(*c_itr));
          //last one is in the radius and this one is not as good so put it on before it
          else {
            batch->emplace_back(std::move(*c_itr));
            std::swap(*(batch->end() - 1), *(batch->end() - 2));
          }
        }
      }
    }

    //bin is finished, advance the candidates to their respective next bins
    for (auto p_itr = begin; p_itr != end; ++p_itr)
      p_itr->next_bin(reader);
  }

  //create the PathLocation corresponding to the best projection of the given candidate
  std::unordered_map<Location, PathLocation> finalize() {
    //at this point we have candidates for each location so now we
    //need to go get the actual correlated location with edge_id etc.
    std::unordered_map<Location, PathLocation> searched;
    for (const auto& pp: pps) {
      if (pp.unisolated.size()) {
        //this may be at a node, either because it was the closest thing or from snap tolerance
        const auto& candidate = pp.unisolated.back();
        bool front = candidate.point == candidate.edge_info->shape().front() ||
          pp.location.latlng_.Distance(candidate.edge_info->shape().front()) < NODE_SNAP;
        bool back = candidate.point == candidate.edge_info->shape().back() ||
          pp.location.latlng_.Distance(candidate.edge_info->shape().back()) < NODE_SNAP;
        //it was the begin node
        if((front && candidate.edge->forward()) || (back && !candidate.edge->forward())) {
          const GraphTile* other_tile;
          auto opposing_edge = reader.GetOpposingEdge(candidate.edge_id, other_tile);
          if(!other_tile)
            throw std::runtime_error("No suitable edges near location");
          searched.insert({pp.location, correlate_node(reader, pp.location, edge_filter, opposing_edge->endnode(), candidate)});
        }//it was the end node
        else if((back && candidate.edge->forward()) || (front && !candidate.edge->forward())) {
          searched.insert({pp.location, correlate_node(reader, pp.location, edge_filter, candidate.edge->endnode(), candidate)});
        }//it was along the edge
        else {
          searched.insert({pp.location, correlate_edge(reader, pp.location, edge_filter, candidate)});
        }
      }
    }
    return searched;
  }

};

}

namespace valhalla {
namespace loki {

std::unordered_map<Location, PathLocation>
Search(const std::vector<Location>& locations, GraphReader& reader, const EdgeFilter& edge_filter, const NodeFilter& node_filter) {
  //trivially finished already
  if(locations.empty())
    return {};
  //setup the unique list of locations
  bin_handler_t handler(locations, reader, edge_filter, node_filter);
  //search over the bins doing multiple locations per bin
  handler.search();
  //turn each locations candidate set into path locations
  return handler.finalize();
}

}
}
