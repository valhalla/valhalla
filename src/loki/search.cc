#include "loki/search.h"

#include <unordered_set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

//during edge searching we snap to vertices if you are closer than
//10 meters to a given vertex.
constexpr float NODE_SNAP = 15 * 15;

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

bool FilterNode(const GraphTile* tile, const NodeInfo* node, const EdgeFilter filter) {
  //for each edge leaving this node
  for(uint32_t edge_index = 0; edge_index < node->edge_count(); ++edge_index) {
    const auto edge = tile->directededge(node->edge_index() + edge_index);
    //if this edge is good we'll take
    if(!filter(edge)){
      return false;
    }
  }
  return true;
}

PathLocation CorrelateNode(const NodeInfo* closest, const Location& location, const GraphTile* tile, EdgeFilter filter){
  //now that we have a node we can pass back all the edges leaving it
  PathLocation correlated(location);
  correlated.CorrelateVertex(closest->latlng());
  for(size_t edge_index = 0; edge_index < closest->edge_count(); ++edge_index) {
    const DirectedEdge* edge = tile->directededge(edge_index);
    if(edge->trans_down() || edge->trans_up())
      continue;
    GraphId id(tile->id());
    id.Set(id.tileid(), id.level(), closest->edge_index() + edge_index);
    if(!filter(edge))
      correlated.CorrelateEdge(id, 0);
  }

  //if we found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("Unable to find any paths leaving this location");

  //give it back
  return correlated;
}

PathLocation NodeSearch(const Location& location, GraphReader& reader, EdgeFilter filter) {

  //grab the tile the lat, lon is in
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);

  //we couldn't find any data for this region
  //TODO: be smarter about this either in loki or in baldr cache
  if(!tile)
    throw std::runtime_error("No data found for location");

  //a place to keep track of which node is closest to our location
  const NodeInfo* closest = tile->node(0);
  float sqdist =  closest->latlng().DistanceSquared(location.latlng_);

  //for each node
  for(size_t node_index = 1; node_index < tile->header()->nodecount(); ++node_index) {
    //if this is closer then its better, unless nothing interesting leaves it..
    const NodeInfo* node = tile->node(node_index);
    float node_sqdist = node->latlng().DistanceSquared(location.latlng_);
    if(node_sqdist < sqdist && !FilterNode(tile, node, filter)) {
      sqdist = node_sqdist;
      closest = node;
    }
  }

  return CorrelateNode(closest, location, tile, filter);
}

PathLocation EdgeSearch(const Location& location, GraphReader& reader, EdgeFilter filter) {
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

  //a place to keep track of the edgeinfos we've already inspected
  std::unordered_set<uint32_t> searched(tile->header()->directededgecount());

  //for each edge
  for(uint32_t edge_index = 0; edge_index < tile->header()->directededgecount(); ++edge_index) {
    //get the edge
    const DirectedEdge* edge = tile->directededge(static_cast<size_t>(edge_index));
    //if its junk, skip it
    if(filter(edge))
      continue;

    //we haven't looked at this edge yet
    auto inserted = searched.insert(edge->edgeinfo_offset());
    if(inserted.second) {
      //get some info about the edge
      auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
      auto candidate = location.latlng_.ClosestPoint(edge_info->shape());

      //is this really close to the geometry
      if(std::get<1>(candidate) < NODE_SNAP)
      {
        //is it basically right on the start of the line
        if(std::get<2>(candidate) == 0 &&
          std::get<0>(candidate).DistanceSquared(edge_info->shape().front()) < NODE_SNAP) {

          //get the opposing edge of this one
          const auto opposing_edge = GetOpposingEdge(reader, edge);
          const auto node = tile->node(opposing_edge->endnode());
          if(!FilterNode(tile, node, filter))
            return CorrelateNode(node, location, tile, filter);
        }//is it basically right on the end of the line
        else if(std::get<2>(candidate) == edge_info->shape().size() - 2 &&
          std::get<0>(candidate).DistanceSquared(edge_info->shape().back()) < NODE_SNAP) {

          //the end node could be in another tile
          const auto other_tile = reader.GetGraphTile(edge->endnode());
          const auto node = other_tile->node(edge->endnode());
          if(!FilterNode(other_tile, node, filter))
            return CorrelateNode(node, location, other_tile, filter);
        }
      }

      //does this look better than the current edge
      if(std::get<1>(candidate) < std::get<1>(closest_point)) {
        closest_edge = edge;
        closest_edge_id.fields.id = edge_index;
        closest_edge_info.swap(edge_info);
        std::swap(closest_point, candidate);
      }
    }
  }

  //now that we have a node we can pass back all the edges leaving it
  PathLocation correlated(location);
  if(closest_edge != nullptr){
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
