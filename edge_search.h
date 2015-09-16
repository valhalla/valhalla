// -*- mode: c++ -*-

#include <cmath>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/sif/dynamiccost.h>

#include "candidate.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


// Copy from valhalla/loki/search.cc
std::tuple<PointLL, float, size_t>
Project(const PointLL& p, const std::vector<PointLL>& shape, const DistanceApproximator& approximator)
{
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


// Some helpers
inline const DirectedEdge*
get_directededge(const GraphTile& tile, const GraphId edgeid)
{
  return tile.directededge(edgeid);
}


inline std::unique_ptr<const EdgeInfo>
get_edgeinfo_ptr(const GraphTile& tile, const DirectedEdge* directededge)
{
  auto offset = directededge->edgeinfo_offset();
  return tile.edgeinfo(offset);
}


inline std::unique_ptr<const EdgeInfo>
get_edgeinfo_ptr(const GraphTile& tile, const GraphId edgeid)
{
  auto edge = get_directededge(tile, edgeid);
  return get_edgeinfo_ptr(tile, edge);
}


inline GraphId
get_edgeid(const GraphTile& tile, const DirectedEdge* directededge)
{
  auto edgeid = tile.header()->graphid();
  edgeid.fields.id = directededge - tile.directededge(0);
  return edgeid;
}


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


std::vector<Candidate>
EdgeSearch(const Location& location,
           GraphReader& reader,
           EdgeFilter filter,
           float sq_search_radius)
{
  std::vector<Candidate> candidates;

  const GraphTile* tile = reader.GetGraphTile(location.latlng_);
  if(!tile || tile->header()->directededgecount() == 0) {
    return candidates;
  }

  std::unordered_set<uint32_t> visited(tile->header()->directededgecount());
  DistanceApproximator approximator(location.latlng_);

  //for each edge
  const auto first_node = tile->node(0);
  const auto last_node  = first_node + tile->header()->nodecount();
  for (auto start_node = first_node; start_node < last_node; start_node++) {

    //for each edge at this node
    const auto first_edge = tile->directededge(start_node->edge_index());
    const auto last_edge  = first_edge + start_node->edge_count();
    for (auto edge = first_edge; edge < last_edge; edge++) {

      //we haven't looked at this edge yet and its not junk
      if(!filter(edge) && visited.insert(edge->edgeinfo_offset()).second) {
        auto edgeinfo_ptr = get_edgeinfo_ptr(*tile, edge);
        auto tuple = Project(location.latlng_, edgeinfo_ptr->shape(), approximator);
        auto sq_distance = std::get<1>(tuple);
        if (sq_distance <= sq_search_radius) {
          auto edgeid = get_edgeid(*tile, edge);
          auto segment = std::get<2>(tuple);

          // Get length ratio of the closet point
          auto point = std::get<0>(tuple);
          float partial_length = 0.f;
          for(size_t i = 0; i < segment; ++i) {
            partial_length += edgeinfo_ptr->shape()[i].Distance(edgeinfo_ptr->shape()[i + 1]);
          }
          partial_length += edgeinfo_ptr->shape()[segment].Distance(point);
          float length_ratio = static_cast<float>(partial_length / static_cast<double>(edge->length()));

          length_ratio = std::max(0.f, std::min(length_ratio, 1.f));
          assert(0.f <= length_ratio && length_ratio <= 1.f);
          auto distance = std::sqrt(sq_distance);

          PathLocation correlated(location);
          correlated.CorrelateVertex(point);
          correlated.CorrelateEdge({edgeid, length_ratio});
          if (!filter(get_directededge(*tile, edgeid))) {
            auto opp_edgeid = reader.GetOpposingEdgeId(edgeid);
            correlated.CorrelateEdge({opp_edgeid, 1 - length_ratio});
          }

          candidates.emplace_back(correlated, distance);
        }
      }
    }
  }

  return candidates;
}
