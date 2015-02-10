#include "loki/search.h"
#include <valhalla/midgard/distanceapproximator.h>

#include <unordered_set>
#include <math.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

//during edge searching we snap to vertices if you are closer than
//15 meters to a given vertex.
constexpr float NODE_SNAP = 15 * 15;
constexpr float EDGE_RADIUS = 500 * 500;

const DirectedEdge* GetOpposingEdge(GraphReader& reader, const DirectedEdge* edge) {
  //get the node at the end of this edge
  const auto node_id = edge->endnode();
  //we are looking for the nth edge exiting this node
  const auto opposing_index = edge->opp_index();
  //the node could be in another tile so we grab that
  const auto tile = reader.GetGraphTile(node_id);
  //grab the nth edge leaving the node
  return tile->directededge(tile->node(node_id)->edge_index() + opposing_index);
}

const NodeInfo* GetBeginNode(GraphReader& reader, const DirectedEdge* edge) {
  auto opposing_edge = GetOpposingEdge(reader, edge);
  auto tile = reader.GetGraphTile(opposing_edge->endnode());
  return tile->node(opposing_edge->endnode());
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

PathLocation CorrelateNode(const NodeInfo* node, const Location& location, const GraphTile* tile, EdgeFilter filter){
  //now that we have a node we can pass back all the edges leaving it
  PathLocation correlated(location);
  correlated.CorrelateVertex(node->latlng());
  const auto start_edge = tile->directededge(node->edge_index());
  const auto end_edge = start_edge + node->edge_count();
  for(auto edge = start_edge; edge < end_edge; ++edge) {
    if(!filter(edge)) {
      GraphId id(tile->id());
      id.fields.id = node->edge_index() + (edge - start_edge);
      correlated.CorrelateEdge(std::move(id), 0);
    }
  }

  //if we found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("Unable to find any paths leaving this location");

  //give it back
  return correlated;
}

const NodeInfo* FindClosestNode(const Location& location, const GraphTile* tile, EdgeFilter filter){
  //a place to keep track of which node is closest to our location
  const NodeInfo* closest = nullptr;
  float sqdist = std::numeric_limits<float>::max();
  DistanceApproximator approximator(location.latlng_);

  //for each node
  const auto start_node = tile->node(0);
  const auto end_node = start_node + tile->header()->nodecount();
  for(auto node = start_node; node < end_node; ++node) {
    //if this is closer then its better, unless nothing interesting leaves it..
    float node_sqdist = approximator.DistanceSquared(location.latlng_);
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
  const auto node = FindClosestNode(location, tile, filter);
  //TODO: look in other tiles
  if(node == nullptr)
    throw std::runtime_error("No data found for location");

  //get some information about it that we can use to make a path from it
  return CorrelateNode(node, location, tile, filter);
}

std::tuple<PointLL, float, int> Project(const PointLL& p, const std::vector<PointLL>& shape, const DistanceApproximator& approximator) {
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

  return std::make_tuple(std::move(closest_point), std::move(closest_distance), std::move(closest_segment));
}

PathLocation EdgeSearch(const Location& location, GraphReader& reader, EdgeFilter filter, const float sq_radius = EDGE_RADIUS) {
  //grab the tile the lat, lon is in
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);

  //we couldn't find any data for this region
  //TODO: be smarter about this either in loki or in baldr cache
  if(!tile)
    throw std::runtime_error("No data found for location");

  //a place to keep the closest information so far
  const DirectedEdge* closest_edge = nullptr;
  GraphId closest_edge_id = tile->header()->graphid();
  std::unique_ptr<const EdgeInfo> closest_edge_info;
  std::tuple<PointLL, float, int> closest_point{{}, std::numeric_limits<float>::max(), 0};
  DistanceApproximator approximator(location.latlng_);

  //a place to keep track of the edgeinfos we've already inspected
  std::unordered_set<uint32_t> searched(tile->header()->directededgecount());

  //for each edge
  const auto start_edge = tile->directededge(0);
  const auto end_edge = start_edge + tile->header()->directededgecount();
  for(auto edge = start_edge; edge < end_edge; ++edge) {

    //we haven't looked at this edge yet and its not junk
    auto inserted = searched.insert(edge->edgeinfo_offset());
    if(inserted.second && !filter(edge)) {

      //we take the mid point of the edges end points and make a radius of half the length of the edge around it
      //this circle is guaranteed to enclose all of the shape. we then say if the input location is less than
      //the input sq_radius away from the circle around the shape, then we consider checking the shape
      auto sq_length = static_cast<float>(edge->length()) * .5f;
      sq_length *= sq_length;
      auto end_node = GetEndNode(reader, edge)->latlng();
      auto start_node = GetBeginNode(reader, edge)->latlng();
      if(sq_length + sq_radius < approximator.DistanceSquared(start_node.MidPoint(end_node)))
        continue;

      //get some info about the edge
      auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
      auto candidate = Project(location.latlng_, edge_info->shape(), approximator);

      //does this look better than the current edge
      if(std::get<1>(candidate) < std::get<1>(closest_point)) {
        closest_edge = edge;
        closest_edge_id.fields.id = edge - start_edge;
        closest_edge_info.swap(edge_info);
        closest_point = std::move(candidate);
      }
    }
  }

  //now that we have an edge we can pass back all the info about it
  PathLocation correlated(location);
  if(closest_edge != nullptr){
    //is it basically right on the start of the line
    if(std::get<2>(closest_point) == 0 &&
      DistanceApproximator::DistanceSquared(std::get<0>(closest_point), closest_edge_info->shape().front()) < NODE_SNAP) {
      //get the opposing edge of this one
      const auto node = GetBeginNode(reader, closest_edge);
      return CorrelateNode(node, location, tile, filter);
    }//is it basically right on the end of the line
    else if(std::get<2>(closest_point) == closest_edge_info->shape().size() - 2 &&
      DistanceApproximator::DistanceSquared(std::get<0>(closest_point), closest_edge_info->shape().back()) < NODE_SNAP) {
      //the end node could be in another tile
      const auto other_tile = reader.GetGraphTile(closest_edge->endnode());
      const auto node = other_tile->node(closest_edge->endnode());
      return CorrelateNode(node, location, other_tile, filter);
    }

    //correlate the spot
    correlated.CorrelateVertex(std::get<0>(closest_point));
    //compute partial distance along the shape
    double partial_length = 0;
    for(size_t i = 1; i < std::get<2>(closest_point); ++i)
        partial_length += closest_edge_info->shape()[i - 1].Distance(closest_edge_info->shape()[i]);
    partial_length += closest_edge_info->shape()[std::get<2>(closest_point)].Distance(std::get<0>(closest_point));
    float length_ratio = static_cast<float>(partial_length / static_cast<double>(closest_edge->length()));
    //correlate the edge we found
    correlated.CorrelateEdge(closest_edge_id, length_ratio);
    //correlate its evil twin
    const auto other_tile = reader.GetGraphTile(closest_edge->endnode());
    const auto end_node = other_tile->node(closest_edge->endnode());
    auto opposing_edge_id = other_tile->header()->graphid();
    opposing_edge_id.fields.id = end_node->edge_index() + closest_edge->opp_index();
    if(!filter(other_tile->directededge(opposing_edge_id)))
      correlated.CorrelateEdge(opposing_edge_id, 1 - length_ratio);
  }

  //if we found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("Unable to find any paths leaving this location");

  //give it back
  return correlated;
}

}

namespace valhalla {
namespace loki {

PathLocation Search(const Location& location, GraphReader& reader, const EdgeFilter filter, const SearchStrategy strategy) {
  if(strategy == SearchStrategy::EDGE)
    return EdgeSearch(location, reader, filter);
  return NodeSearch(location, reader, filter);
}

}
}
