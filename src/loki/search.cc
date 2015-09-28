#include "loki/search.h"
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>

#include <unordered_set>
#include <list>
#include <math.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace {

//during side of street computations we figured you're on the street if you are less than
//5 meters (16) feet from the centerline. this is actually pretty large (with accurate shape
//data for the roads it might want half that) but its better to assume on street than not
constexpr float NONE_SNAP = 5.f * 5.f;
//during edge searching we snap to vertices if you are closer than
//12.5 meters (40 feet) to a given vertex.
constexpr float NODE_SNAP = 12.5f * 12.5f;
//if you arent in this vicinity from an edge then you arent very close to it, helps to not
//look at the shape of edges (expensive) for edges that are probably very far from your location
constexpr float EDGE_RADIUS = 250.f * 250.f;
//if you are this far away from the edge we are considering and you set a heading we will
//ignore it because its not really useful at this distance from the geometry
constexpr float NO_HEADING = 30.f * 30.f;
//how much of the shape should be sampled to get heading
constexpr float HEADING_SAMPLE = 30.f;
//cone width to use for cosine similarity comparisons for favoring heading
constexpr float ANGLE_WIDTH = 88.f;

//TODO: move this to midgard and test the crap out of it
//we are essentially estimating the angle of the tangent
//at a point along a discretised curve. we attempt to mostly
//use the shape coming into the point on the curve but if there
//isnt enough there we will use the shape coming out of the it
float Angle(size_t index, const PointLL& point, const std::vector<PointLL>& shape, bool forward) {
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

bool HeadingFilter(const DirectedEdge* edge, const std::unique_ptr<const EdgeInfo>& info,
  const std::tuple<PointLL, float, int>& point, boost::optional<int> heading) {
  //if its far enough away from the edge, the heading is pretty useless
  if(!heading || std::get<1>(point) > NO_HEADING)
    return false;

  //get the angle of the shape from this point
  auto angle = Angle(std::get<2>(point), std::get<0>(point), info->shape(), edge->forward());
  //we want the closest distance between two angles which can be had
  //across 0 or between the two so we just need to know which is bigger
  if(*heading > angle)
    return std::min(*heading - angle, (360.f - *heading) + angle) > ANGLE_WIDTH;
  return std::min(angle - *heading, (360.f - angle) + *heading) > ANGLE_WIDTH;
}

PathLocation::SideOfStreet FlipSide(const PathLocation::SideOfStreet side) {
  if(side != PathLocation::SideOfStreet::NONE)
    return side == PathLocation::SideOfStreet::LEFT ? PathLocation::SideOfStreet::RIGHT : PathLocation::SideOfStreet::LEFT;
  return side;
}

PathLocation::SideOfStreet GetSide(const DirectedEdge* edge, const std::unique_ptr<const EdgeInfo>& info,
  const std::tuple<PointLL, float, int>& point, const PointLL& original){

  //its so close to the edge that its basically on the edge
  if(std::get<1>(point) < NONE_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //if the projected point is way too close to the begin or end of the shape
  //TODO: if the original point is really far away side of street may also not make much sense..
  if(std::get<0>(point).DistanceSquared(info->shape().front()) < NODE_SNAP ||
     std::get<0>(point).DistanceSquared(info->shape().back()) < NODE_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //what side
  auto index = std::get<2>(point);
  LineSegment2<PointLL> segment(info->shape()[index], info->shape()[index + 1]);
  return (segment.IsLeft(original) > 0) == edge->forward()  ? PathLocation::SideOfStreet::LEFT : PathLocation::SideOfStreet::RIGHT;
}

const NodeInfo* GetEndNode(GraphReader& reader, const DirectedEdge* edge) {
  //the node could be in another tile so we grab that
  const auto tile = reader.GetGraphTile(edge->endnode());
  //grab the nth edge leaving the node
  return tile->node(edge->endnode());
}

bool FilterNode(const GraphTile* tile, const NodeInfo* node, const EdgeFilter filter) {
  //for each edge leaving this node
  const auto start_edge = tile->directededge(node->edge_index());
  const auto end_edge = start_edge + node->edge_count();
  for(auto edge = start_edge; edge < end_edge; ++edge) {
    //if this edge is good we'll take
    if(!filter(edge)){
      return false;
    }
  }
  return true;
}

PathLocation CorrelateNode(GraphReader& reader, const NodeInfo* node, const Location& location, const GraphTile* tile, EdgeFilter filter, const float sqdist){
  PathLocation correlated(location);
  correlated.CorrelateVertex(node->latlng());
  std::list<PathLocation::PathEdge> heading_filtered;
  //now that we have a node we can pass back all the edges leaving and entering it
  const auto* start_edge = tile->directededge(node->edge_index());
  const auto* end_edge = start_edge + node->edge_count();
  auto closest_point = std::make_tuple(node->latlng(), sqdist, 0);
  for(const auto* edge = start_edge; edge < end_edge; ++edge) {
    //get some info about this edge and the opposing
    GraphId id = tile->id();
    id.fields.id = node->edge_index() + (edge - start_edge);
    const GraphTile* other_tile;
    const auto other_id = reader.GetOpposingEdgeId(id, other_tile);
    const auto* other_edge = other_tile->directededge(other_id);
    auto info = tile->edgeinfo(edge->edgeinfo_offset());

    //do we want this edge
    if(!filter(edge)) {
      PathLocation::PathEdge path_edge{std::move(id), 0.f, PathLocation::NONE};
      std::get<2>(closest_point) = edge->forward() ? 0 : info->shape().size() - 2;
      if(!HeadingFilter(edge, info, closest_point, location.heading_))
        correlated.CorrelateEdge(std::move(path_edge));
      else
        heading_filtered.emplace_back(std::move(path_edge));
    }

    //do we want the evil twin
    if(!filter(other_edge)) {
      PathLocation::PathEdge path_edge{std::move(other_id), 1.f, PathLocation::NONE};
      std::get<2>(closest_point) = other_edge->forward() ? 0 : info->shape().size() - 2;
      if(!HeadingFilter(other_edge, tile->edgeinfo(edge->edgeinfo_offset()), closest_point, location.heading_))
        correlated.CorrelateEdge(std::move(path_edge));
      else
        heading_filtered.emplace_back(std::move(path_edge));
    }
  }

  //if we have nothing because of heading we'll just ignore it
  if(correlated.edges().size() == 0 && heading_filtered.size())
    for(auto& path_edge : heading_filtered)
      correlated.CorrelateEdge(std::move(path_edge));

  //if we still found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("No suitable edges near location");

  //give it back
  return correlated;
}

const NodeInfo* FindClosestNode(const Location& location, const GraphTile* tile, EdgeFilter filter, float& sqdist){
  //a place to keep track of which node is closest to our location
  const NodeInfo* closest = nullptr;
  sqdist = std::numeric_limits<float>::max();
  DistanceApproximator approximator(location.latlng_);

  //for each node
  const auto start_node = tile->node(0);
  const auto end_node = start_node + tile->header()->nodecount();
  for(auto node = start_node; node < end_node; ++node) {
    //if this is closer then its better, unless nothing interesting leaves it..
    float node_sqdist = approximator.DistanceSquared(node->latlng());
    if(node_sqdist < sqdist && !FilterNode(tile, node, filter)) {
      sqdist = node_sqdist;
      closest = node;
    }
  }

  return closest;
}

PathLocation NodeSearch(const Location& location, GraphReader& reader, EdgeFilter filter) {
  //grab the tile the lat, lon is in
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);

  //we couldn't find any data for this region
  //TODO: be smarter about this either in loki or in baldr cache
  if(!tile)
    throw std::runtime_error("No data found for location");

  //grab the absolute closest node to this location
  float square_distance = 0.f;
  const auto* node = FindClosestNode(location, tile, filter, square_distance);
  //TODO: look in other tiles
  if(node == nullptr)
    throw std::runtime_error("No data found for location");

  //get some information about it that we can use to make a path from it
  return CorrelateNode(reader, node, location, tile, filter, square_distance);
}

std::tuple<PointLL, float, size_t> Project(const PointLL& p, const std::vector<PointLL>& shape, const DistanceApproximator& approximator) {
  size_t closest_segment = 0;
  float closest_distance = std::numeric_limits<float>::max();
  PointLL closest_point{};

  //for each segment
  for(size_t i = 0; i < shape.size() - 1; ++i) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto bx = v.first - u.first;
    auto by = v.second - u.second;
    const auto scale = ((p.first - u.first)*bx + (p.second - u.second)*by) / (bx*bx + by*by);
    //projects along the ray before u
    if(scale <= 0.f) {
      bx = u.first;
      by = u.second;
    }//projects along the ray after v
    else if(scale >= 1.f) {
      bx = v.first;
      by = v.second;
    }//projects along the ray between u and v
    else {
      bx = bx*scale + u.first;
      by = by*scale + u.second;
    }
    //check if this point is better
    PointLL point(bx, by);
    const auto distance = approximator.DistanceSquared(point);
    if(distance < closest_distance) {
      closest_segment = i;
      closest_distance = distance;
      closest_point = std::move(point);
    }
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment);
}

PathLocation EdgeSearch(const Location& location, GraphReader& reader, EdgeFilter filter, float sq_search_radius = EDGE_RADIUS, float sq_node_snap = NODE_SNAP) {
  //grab the tile the lat, lon is in
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);

  //we couldn't find any data for this region
  //TODO: be smarter about this either in loki or in baldr cache
  if(!tile || tile->header()->directededgecount() == 0)
    throw std::runtime_error("No data found for location");

  //a place to keep track of the edges we should look at by their match quality (just distance for now)
  std::unordered_set<uint32_t> visited(tile->header()->directededgecount());
  std::list<const DirectedEdge*> close, far;
  DistanceApproximator approximator(location.latlng_);

  //for each node
  const auto first_node = tile->node(0);
  const auto last_node  = first_node + tile->header()->nodecount();
  for (auto start_node = first_node; start_node < last_node; start_node++) {

    //for each edge at this node
    const auto first_edge = tile->directededge(start_node->edge_index());
    const auto last_edge  = first_edge + start_node->edge_count();
    for (auto edge = first_edge; edge < last_edge; edge++) {

      //we haven't looked at this edge yet and its not junk
      if(!filter(edge) && visited.insert(edge->edgeinfo_offset()).second) {
        auto end_node = GetEndNode(reader, edge);

        //is it basically right on the start of the edge
        float sqdist;
        if((sqdist = approximator.DistanceSquared(start_node->latlng())) < sq_node_snap) {
          return CorrelateNode(reader, start_node, location, tile, filter, sqdist);
        }//is it basically right on the end of the edge
        else if((sqdist = approximator.DistanceSquared(end_node->latlng())) < sq_node_snap) {
          return CorrelateNode(reader, end_node, location, reader.GetGraphTile(edge->endnode()), filter, sqdist);
        }

        //we take the mid point of the edges end points and make a radius of half the length of the edge around it
        //this circle is guaranteed to enclose all of the shape. we then store how far away from this circle the
        //the input location is
        auto sq_radius = static_cast<float>(edge->length()) * .5f;
        sq_radius *= sq_radius;
        if(approximator.DistanceSquared(start_node->latlng().MidPoint(end_node->latlng())) - sq_radius < sq_search_radius)
          close.push_back(edge);
        else
          far.push_back(edge);
      }
    }
  }

  //TODO: if the tile is pretty sparse anyway just search the whole thing
  //we only want to look at edges that are a decently close to our input, however if none
  //of them are close we just assume its a pretty empty tile and search the whole thing
  if(close.size() == 0)
    close.splice(close.end(), far);

  //for each edge
  const DirectedEdge* closest_edge = nullptr;
  GraphId closest_edge_id = tile->header()->graphid();
  std::unique_ptr<const EdgeInfo> closest_edge_info;
  std::tuple<PointLL, float, int> closest_point{{}, std::numeric_limits<float>::max(), 0};
  for(const auto edge : close) {
    //get some info about the edge
    auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
    auto candidate = Project(location.latlng_, edge_info->shape(), approximator);

    //does this look better than the current edge
    if(std::get<1>(candidate) < std::get<1>(closest_point)) {
      closest_edge = edge;
      closest_edge_id.fields.id = edge - tile->directededge(0);
      closest_edge_info.swap(edge_info);
      closest_point = std::move(candidate);
    }
  }

  //now that we have an edge we can pass back all the info about it
  PathLocation correlated(location);
  if(closest_edge != nullptr){
    //correlate the spot
    correlated.CorrelateVertex(std::get<0>(closest_point));
    //compute partial distance along the shape
    double partial_length = 0;
    for(size_t i = 0; i < std::get<2>(closest_point); ++i)
        partial_length += closest_edge_info->shape()[i].Distance(closest_edge_info->shape()[i + 1]);
    partial_length += closest_edge_info->shape()[std::get<2>(closest_point)].Distance(std::get<0>(closest_point));
    float length_ratio = static_cast<float>(partial_length / static_cast<double>(closest_edge->length()));
    // The length ratio at this point could be slightly greater than 1.0
    // because of floating point imprecisions during the partial_length
    // computation, so we clip it to 1.0.
    length_ratio = std::min(1.0f, length_ratio);
    if(!closest_edge->forward())
      length_ratio = 1.f - length_ratio;
    //side of street
    auto side = GetSide(closest_edge, closest_edge_info, closest_point, location.latlng_);
    //correlate the edge we found
    std::list<PathLocation::PathEdge> heading_filtered;
    if(HeadingFilter(closest_edge, closest_edge_info, closest_point, location.heading_))
      heading_filtered.emplace_back(closest_edge_id, length_ratio, side);
    else
      correlated.CorrelateEdge(PathLocation::PathEdge{closest_edge_id, length_ratio, side});
    //correlate its evil twin
    const GraphTile* other_tile;
    auto opposing_edge_id = reader.GetOpposingEdgeId(closest_edge_id, other_tile);
    const auto* other_edge = other_tile->directededge(opposing_edge_id);
    if(!filter(other_edge)) {
      if(HeadingFilter(other_edge, closest_edge_info, closest_point, location.heading_))
        heading_filtered.emplace_back(opposing_edge_id, 1 - length_ratio, FlipSide(side));
      else
        correlated.CorrelateEdge(PathLocation::PathEdge{opposing_edge_id, 1 - length_ratio, FlipSide(side)});
    }

    //if we have nothing because of heading we'll just ignore it
    if(correlated.edges().size() == 0 && heading_filtered.size())
      for(auto& path_edge : heading_filtered)
        correlated.CorrelateEdge(std::move(path_edge));
  }

  //if we found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("No suitable edges near location");

  //give it back
  return correlated;
}

}

namespace valhalla {
namespace loki {

PathLocation Search(const Location& location, GraphReader& reader, const EdgeFilter filter, const SearchStrategy strategy) {
  //TODO: if a name is supplied try to find edges with similar name
  //TODO: if a direction is provided, try to find edges whose angles are similar to the input

  if(strategy == SearchStrategy::EDGE)
    return EdgeSearch(location, reader, filter);
  return NodeSearch(location, reader, filter);
}

}
}
