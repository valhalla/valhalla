#include "loki/search.h"
#include <valhalla/midgard/linesegment2.h>

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
constexpr float SEARCH_CUTTOFF = 35000.f;
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
constexpr float ANGLE_WIDTH = 88.f;

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

bool heading_filter(const DirectedEdge* edge, const std::unique_ptr<const EdgeInfo>& info,
  const std::tuple<PointLL, float, int>& point, boost::optional<int> heading) {
  //if its far enough away from the edge, the heading is pretty useless
  if(!heading || std::get<1>(point) > NO_HEADING)
    return false;

  //get the angle of the shape from this point
  auto angle = tangent_angle(std::get<2>(point), std::get<0>(point), info->shape(), edge->forward());
  //we want the closest distance between two angles which can be had
  //across 0 or between the two so we just need to know which is bigger
  if(*heading > angle)
    return std::min(*heading - angle, (360.f - *heading) + angle) > ANGLE_WIDTH;
  return std::min(angle - *heading, (360.f - angle) + *heading) > ANGLE_WIDTH;
}

PathLocation::SideOfStreet flip_side(const PathLocation::SideOfStreet side) {
  if(side != PathLocation::SideOfStreet::NONE)
    return side == PathLocation::SideOfStreet::LEFT ? PathLocation::SideOfStreet::RIGHT : PathLocation::SideOfStreet::LEFT;
  return side;
}

PathLocation::SideOfStreet get_side(const DirectedEdge* edge, const std::unique_ptr<const EdgeInfo>& info,
  const std::tuple<PointLL, float, int>& point, const PointLL& original){

  //its so close to the edge that its basically on the edge
  if(std::get<1>(point) < SIDE_OF_STREET_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //if the projected point is way too close to the begin or end of the shape
  //TODO: if the original point is really far away side of street may also not make much sense..
  if(std::get<0>(point).Distance(info->shape().front()) < SIDE_OF_STREET_SNAP ||
     std::get<0>(point).Distance(info->shape().back()) < SIDE_OF_STREET_SNAP)
    return PathLocation::SideOfStreet::NONE;

  //get the side TODO: this can technically fail for longer segments..
  //to fix it we simply compute the plane formed by the triangle
  //through the center of the earth and the two shape points and test
  //whether the original point is above or below the plane (depending on winding)
  auto index = std::get<2>(point);
  LineSegment2<PointLL> segment(info->shape()[index], info->shape()[index + 1]);
  return (segment.IsLeft(original) > 0) == edge->forward()  ? PathLocation::SideOfStreet::LEFT : PathLocation::SideOfStreet::RIGHT;
}

const NodeInfo* get_end_node(GraphReader& reader, const DirectedEdge* edge) {
  //the node could be in another tile so we grab that
  const auto tile = reader.GetGraphTile(edge->endnode());
  return tile->node(edge->endnode());
}

PathLocation correlate_node(GraphReader& reader, const Location& location, const EdgeFilter& edge_filter, const GraphTile* tile, const NodeInfo* node, const float sqdist){
  PathLocation correlated(location);
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
    if(edge_filter(edge) != 0.0f) {
      PathLocation::PathEdge path_edge{std::move(id), 0.f, node->latlng(), std::get<1>(closest_point), PathLocation::NONE};
      std::get<2>(closest_point) = edge->forward() ? 0 : info->shape().size() - 2;
      if(!heading_filter(edge, info, closest_point, location.heading_))
        correlated.edges.push_back(std::move(path_edge));
      else
        heading_filtered.emplace_back(std::move(path_edge));
    }

    //do we want the evil twin
    if(edge_filter(other_edge) != 0.0f) {
      PathLocation::PathEdge path_edge{std::move(other_id), 1.f, node->latlng(), std::get<1>(closest_point), PathLocation::NONE};
      std::get<2>(closest_point) = other_edge->forward() ? 0 : info->shape().size() - 2;
      if(!heading_filter(other_edge, tile->edgeinfo(edge->edgeinfo_offset()), closest_point, location.heading_))
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

  //give it back
  return correlated;
}

PathLocation correlate_edge(GraphReader& reader, const Location& location, const EdgeFilter& edge_filter, const std::tuple<PointLL, float, size_t>& closest_point,
    const DirectedEdge* closest_edge, const GraphId& closest_edge_id, const std::unique_ptr<const EdgeInfo>&closest_edge_info) {
  //now that we have an edge we can pass back all the info about it
  PathLocation correlated(location);
  if(closest_edge != nullptr){
    //we need the ratio in the direction of the edge we are correlated to
    double partial_length = 0;
    for(size_t i = 0; i < std::get<2>(closest_point); ++i)
      partial_length += closest_edge_info->shape()[i].Distance(closest_edge_info->shape()[i + 1]);
    partial_length += closest_edge_info->shape()[std::get<2>(closest_point)].Distance(std::get<0>(closest_point));
    partial_length = std::min(partial_length, static_cast<double>(closest_edge->length()));
    float length_ratio = static_cast<float>(partial_length / static_cast<double>(closest_edge->length()));
    if(!closest_edge->forward())
      length_ratio = 1.f - length_ratio;
    //side of street
    auto side = get_side(closest_edge, closest_edge_info, closest_point, location.latlng_);
    //correlate the edge we found
    std::list<PathLocation::PathEdge> heading_filtered;
    if(heading_filter(closest_edge, closest_edge_info, closest_point, location.heading_))
      heading_filtered.emplace_back(closest_edge_id, length_ratio, std::get<0>(closest_point), side);
    else
      correlated.edges.push_back(PathLocation::PathEdge{closest_edge_id, length_ratio, std::get<0>(closest_point), std::get<1>(closest_point), side});
    //correlate its evil twin
    const GraphTile* other_tile;
    auto opposing_edge_id = reader.GetOpposingEdgeId(closest_edge_id, other_tile);
    const DirectedEdge* other_edge;
    if(opposing_edge_id.Is_Valid() && (other_edge = other_tile->directededge(opposing_edge_id)) && edge_filter(other_edge) != 0.0f) {
      if(heading_filter(other_edge, closest_edge_info, closest_point, location.heading_))
        heading_filtered.emplace_back(opposing_edge_id, 1 - length_ratio, std::get<0>(closest_point), std::get<1>(closest_point), flip_side(side));
      else
        correlated.edges.push_back(PathLocation::PathEdge{opposing_edge_id, 1 - length_ratio, std::get<0>(closest_point), std::get<1>(closest_point), flip_side(side)});
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

std::tuple<PointLL, float, size_t> project(const PointLL& p, const std::vector<PointLL>& shape) {
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
    auto sq = bx*bx + by*by;
    //avoid divided-by-zero which gives a NaN scale, otherwise comparisons below will fail
    const auto scale = sq > 0? (((p.first - u.first)*bx + (p.second - u.second)*by) / sq) : 0.f;
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
    const auto distance = p.Distance(point);
    if(distance < closest_distance) {
      closest_segment = i;
      closest_distance = distance;
      closest_point = std::move(point);
    }
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment);
}

//TODO: this is frought with peril. to properly to this we need to know
//where in the world we are and use lower casing rules that are appropriate
//so we can maximize the similarity measure.
//are the names similar enough to consider them matching
/*bool name_filter() {

}*/

// Test if this location is an isolated "island" without connectivity to the
// larger routing graph. Does a breadth first search - if possible paths are
// exhausted within some threshold this returns a set of edges within the
// island.
std::unordered_set<GraphId> island(const PathLocation& location,
             GraphReader& reader, const NodeFilter& node_filter,
             const EdgeFilter& edge_filter, const uint32_t edge_threshold,
             const uint32_t length_threshold, const uint32_t node_threshold) {
  std::unordered_set<GraphId> todo(edge_threshold);
  std::unordered_set<GraphId> done(edge_threshold);

  // Seed the list of edges to expand
  for (const auto& edge : location.edges) {
    todo.insert(edge.id);
  }

  // We are done if we hit a threshold meaning it isn't an island or we ran
  // out of edges and we determine it is an island
  uint32_t total_edge_length = 0;
  uint32_t nodes_expanded = 0;
  while ((done.size() < edge_threshold || total_edge_length < length_threshold ||
          nodes_expanded < node_threshold) && todo.size()) {
    // Get the next edge
    const GraphId edge = *todo.cbegin();
    done.emplace(edge);

    // Get the directed edge - filter it out if not accessible
    const DirectedEdge* directededge = reader.GetGraphTile(edge)->directededge(edge);
    if (edge_filter(directededge) == 0.0f) {
      continue;
    }
    total_edge_length += directededge->length();

    // Get the end node - filter it out if not accessible
    const GraphId node = directededge->endnode();
    const GraphTile* tile = reader.GetGraphTile(node);
    const NodeInfo* nodeinfo = tile->node(node);
    if (node_filter(nodeinfo)) {
      continue;
    }

    // Expand edges from the node
    bool expanded = false;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Skip transition edges, transit connection edges, and edges that are not allowed
      if (directededge->trans_up() || directededge->trans_down() ||
          directededge->use() == Use::kTransitConnection ||
          edge_filter(directededge) == 0.0f) {
        continue;
      }

      // Add to the todo list
      todo.emplace(edgeid);
      expanded = true;
    }
    nodes_expanded += expanded;
  }

  // If there are still edges to do then we broke out of the loop above due to
  // meeting thresholds and this is not a disconnected island. If there are no
  // more edges then this is a disconnected island and we want to know what
  // edges constitute the island so a second pass can avoid them
  return (todo.size() == 0) ? done : std::unordered_set<GraphId>{};
}

PathLocation search(const Location& location, GraphReader& reader, const EdgeFilter& edge_filter) {
  //iterate over bins in a closest first manner
  const auto& tiles = reader.GetTileHierarchy().levels().rbegin()->second.tiles;
  const auto level = reader.GetTileHierarchy().levels().rbegin()->first;
  auto binner = tiles.ClosestFirst(location.latlng_);

  //TODO: if a name is supplied try to find edges with similar name
  //TODO: change filtering from boolean to floating point 0-1

  //a place to keep track of the edges we should look at by their match quality (just distance for now)
  const GraphTile* closest_tile = nullptr;
  const DirectedEdge* closest_edge = nullptr;
  GraphId closest_edge_id;
  std::unique_ptr<const EdgeInfo> closest_edge_info;
  std::tuple<PointLL, float, int> closest_point{{}, std::numeric_limits<float>::max(), 0};

  //give up if we find nothing after a while
  size_t bins = 0;
  while(true) {
    try {
      //TODO: make configurable the radius at which we give up searching
      auto bin = binner();
      if(std::get<2>(bin) > SEARCH_CUTTOFF)
        throw std::exception{};

      //the closest thing in this bin is further than what we have already
      if(std::get<2>(bin) > std::get<1>(closest_point))
        break;

      //grab the tile the lat, lon is in
      const auto* tile = reader.GetGraphTile(GraphId(std::get<0>(bin), level, 0));
      if(!tile)
        continue;

      //iterate over the edges in the bin
      auto edges = tile->GetBin(std::get<1>(bin));
      bins += static_cast<size_t>(edges.size() > 0);
      for(auto e : edges) {
        //get the tile and edge
        if(!tile || e.tileid() != tile->id().tileid()) {
          tile = reader.GetGraphTile(e);
          if(!tile)
            continue;
        }
        const auto* edge = tile->directededge(e);
        //no thanks on this one or its evil twin
        if(edge_filter(edge) == 0.0f && (!(e = reader.GetOpposingEdgeId(e, tile)).Is_Valid() ||
          edge_filter(edge = tile->directededge(e)) == 0.0f)) {
          continue;
        }
        //get some info about the edge
        auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
        auto candidate = project(location.latlng_, edge_info->shape());

        //does this look better than the current edge
        if(std::get<1>(candidate) < std::get<1>(closest_point)) {
          closest_edge = edge;
          closest_edge_id = e;
          closest_edge_info.swap(edge_info);
          closest_point = std::move(candidate);
          closest_tile = tile;
        }
      }
    }
    catch(...) {
      throw std::runtime_error("No data found for location");
    }
  }

  //keep track of bins we looked in but only the ones that had something
  //would rather log this in the service only, so lets figure a way to pass it back
  //midgard::logging::Log("valhalla_loki_bins_searched::" + std::to_string(bins), " [ANALYTICS] ");

  //this may be at a node, either because it was the closest thing or from snap tolerance
  bool front = std::get<0>(closest_point) == closest_edge_info->shape().front() ||
               location.latlng_.Distance(closest_edge_info->shape().front()) < NODE_SNAP;
  bool back = std::get<0>(closest_point) == closest_edge_info->shape().back() ||
              location.latlng_.Distance(closest_edge_info->shape().back()) < NODE_SNAP;
  //it was the begin node
  if((front && closest_edge->forward()) || (back && !closest_edge->forward())) {
    const GraphTile* other_tile;
    auto opposing_edge = reader.GetOpposingEdge(closest_edge_id, other_tile);
    if(!other_tile)
      throw std::runtime_error("No suitable edges near location");
    return correlate_node(reader, location, edge_filter, closest_tile, closest_tile->node(opposing_edge->endnode()), std::get<1>(closest_point));
  }
  //it was the end node
  if((back && closest_edge->forward()) || (front && !closest_edge->forward())) {
    const GraphTile* other_tile = reader.GetGraphTile(closest_edge->endnode());
    if(!other_tile)
      throw std::runtime_error("No suitable edges near location");
    return correlate_node(reader, location, edge_filter, other_tile, other_tile->node(closest_edge->endnode()), std::get<1>(closest_point));
  }
  //it was along the edge
  return correlate_edge(reader, location, edge_filter, closest_point, closest_edge, closest_edge_id, closest_edge_info);
}

}

namespace valhalla {
namespace loki {

PathLocation Search(const Location& location, GraphReader& reader, const EdgeFilter& edge_filter, const NodeFilter& node_filter) {
  //TODO: check if its an island and then search again
  return search(location, reader, edge_filter);
}

}
}
