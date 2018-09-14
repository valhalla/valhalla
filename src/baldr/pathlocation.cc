#include "baldr/pathlocation.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

PathLocation::PathEdge::PathEdge(const GraphId& id,
                                 const float dist,
                                 const midgard::PointLL& projected,
                                 const float score,
                                 const SideOfStreet sos,
                                 const unsigned int minimum_reachability)
    : id(id), percent_along(dist), projected(projected), sos(sos), distance(score),
      minimum_reachability(minimum_reachability) {
}
bool PathLocation::PathEdge::begin_node() const {
  return percent_along == 0.f;
}
bool PathLocation::PathEdge::end_node() const {
  return percent_along == 1.f;
}

PathLocation::PathLocation(const Location& location) : Location(location) {
  edges.reserve(16);
}

bool PathLocation::operator==(const PathLocation& other) const {
  // Check all of the scalar properties
  if (other.minimum_reachability_ != minimum_reachability_ || other.radius_ != radius_ ||
      other.stoptype_ != stoptype_ || other.latlng_ != latlng_ || other.heading_ != heading_ ||
      other.heading_tolerance_ != heading_tolerance_ ||
      other.node_snap_tolerance_ != node_snap_tolerance_ || other.way_id_ != way_id_ ||
      other.city_ != city_ || other.country_ != country_ || other.date_time_ != date_time_ ||
      other.name_ != name_ || other.state_ != state_ || other.street_ != street_ ||
      other.zip_ != zip_ || other.edges.size() != edges.size()) {
    return false;
  }

  return shares_edges(other);
}

bool PathLocation::shares_edges(const PathLocation& other) const {
  // Check that the other PathLocation has all the edges we do
  for (const auto& edge : edges) {
    bool found = false;
    for (const auto& other_edge : other.edges) {
      if (edge.id == other_edge.id && edge.sos == other_edge.sos &&
          midgard::equal<float>(edge.percent_along, other_edge.percent_along) &&
          midgard::similar<float>(edge.distance + 1, other_edge.distance + 1) &&
          edge.projected.ApproximatelyEqual(other_edge.projected)) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

rapidjson::Value PathLocation::ToRapidJson(size_t index,
                                           rapidjson::Document::AllocatorType& allocator) const {
  rapidjson::Value value{rapidjson::kObjectType};
  rapidjson::Value array{rapidjson::kArrayType};
  rapidjson::Value filtered_edges_array{rapidjson::kArrayType};

  array.Reserve(edges.size(), allocator);
  filtered_edges_array.Reserve(filtered_edges.size(), allocator);

  for (const auto& edge : edges) {
    rapidjson::Value e = PathEdgeToRapidJson(edge, allocator);
    array.PushBack(e.Move(), allocator);
  }
  for (const auto& edge : filtered_edges) {
    rapidjson::Value e = PathEdgeToRapidJson(edge, allocator);
    filtered_edges_array.PushBack(e.Move(), allocator);
  }

  value.AddMember("edges", array.Move(), allocator)
      .AddMember("location_index", static_cast<int>(index), allocator)
      .AddMember("filtered_edges", filtered_edges_array.Move(), allocator);
  return value;
}

PathLocation PathLocation::FromRapidJson(const std::vector<Location>& locations,
                                         const rapidjson::Value& path_location) {
  auto index = rapidjson::get<uint64_t>(path_location, "/location_index");
  PathLocation p(locations[index]);
  for (const auto& edge : rapidjson::get<rapidjson::Value::ConstArray>(path_location, "/edges")) {
    p.edges.emplace_back(GraphId(rapidjson::get<uint64_t>(edge, "/id")),
                         rapidjson::get<float>(edge, "/dist"),
                         midgard::PointLL(rapidjson::get<double>(edge, "/projected/lon"),
                                          rapidjson::get<double>(edge, "/projected/lat")),
                         rapidjson::get<float>(edge, "/score"),
                         static_cast<SideOfStreet>(rapidjson::get<int>(edge, "/sos")),
                         rapidjson::get<unsigned int>(edge, "/minimum_reachability"));
  }
  for (const auto& edge :
       rapidjson::get<rapidjson::Value::ConstArray>(path_location, "/filtered_edges")) {
    p.filtered_edges.emplace_back(GraphId(rapidjson::get<uint64_t>(edge, "/id")),
                                  rapidjson::get<float>(edge, "/dist"),
                                  midgard::PointLL(rapidjson::get<double>(edge, "/projected/lon"),
                                                   rapidjson::get<double>(edge, "/projected/lat")),
                                  rapidjson::get<float>(edge, "/score"),
                                  static_cast<SideOfStreet>(rapidjson::get<int>(edge, "/sos")),
                                  rapidjson::get<unsigned int>(edge, "/minimum_reachability"));
  }
  return p;
}

rapidjson::Value
PathLocation::PathEdgeToRapidJson(const PathEdge& edge,
                                  rapidjson::Document::AllocatorType& allocator) const {
  rapidjson::Value e{rapidjson::kObjectType};

  e.AddMember("id", edge.id.value, allocator)
      .AddMember("dist", edge.percent_along, allocator)
      .AddMember("sos", static_cast<int>(edge.sos), allocator)
      .AddMember("score", edge.distance, allocator)
      .AddMember("minimum_reachability", edge.minimum_reachability, allocator);

  // Serialize projected lat,lng as double (otherwise leads to shape
  // artifacts at begin/end of routes as the float values are rounded
  rapidjson::Value vtx{rapidjson::kObjectType};
  vtx.AddMember("lon", static_cast<double>(edge.projected.first), allocator)
      .AddMember("lat", static_cast<double>(edge.projected.second), allocator);
  e.AddMember("projected", vtx.Move(), allocator);

  return e;
}

} // namespace baldr
} // namespace valhalla
