#include "baldr/pathlocation.h"
#include "midgard/util.h"

namespace valhalla{
namespace baldr{

  PathLocation::PathEdge::PathEdge(const GraphId& id, const float dist,
    const midgard::PointLL& projected, const float score, const SideOfStreet sos, const int minimum_reachability):
    id(id), dist(dist), projected(projected), sos(sos), score(score), minimum_reachability(minimum_reachability) {
  }
  bool PathLocation::PathEdge::begin_node() const {
    return dist == 0.f;
  }
  bool PathLocation::PathEdge::end_node() const {
    return dist == 1.f;
  }

  PathLocation::PathLocation(const Location& location):Location(location) {
    edges.reserve(16);
  }

  bool PathLocation::operator==(const PathLocation& other) const {
    for(const auto& edge : edges) {
      bool found = false;
      for(const auto& other_edge : other.edges) {
        if(edge.id == other_edge.id && edge.sos == other_edge.sos && midgard::equal<float>(edge.dist, other_edge.dist) &&
            midgard::similar<float>(edge.score + 1, other_edge.score + 1) && edge.projected.ApproximatelyEqual(other_edge.projected)){
          found = true;
          break;
        }
      }
      if(!found)
        return false;
    }
    return true;
  }

  rapidjson::Value PathLocation::ToRapidJson(size_t index, rapidjson::Document::AllocatorType& allocator) const {
    rapidjson::Value value{rapidjson::kObjectType};
    rapidjson::Value array{rapidjson::kArrayType};
    rapidjson::Value filtered_edges_array{rapidjson::kArrayType};

    array.Reserve(edges.size(), allocator);
    filtered_edges_array.Reserve(filtered_edges.size(), allocator);

    for(const auto& edge : edges) {
      rapidjson::Value e = PathEdgeToRapidJson(edge, allocator);
      array.PushBack(e.Move(), allocator);
    }
    for(const auto& edge : filtered_edges) {
      rapidjson::Value e = PathEdgeToRapidJson(edge, allocator);
      filtered_edges_array.PushBack(e.Move(), allocator);
    }

    value.AddMember("edges", array.Move(), allocator)
         .AddMember("location_index", static_cast<int>(index), allocator)
         .AddMember("filtered_edges", filtered_edges_array.Move(), allocator);
    return value;
  }


  PathLocation PathLocation::FromPtree(const std::vector<Location>& locations, const boost::property_tree::ptree& path_location){
    auto index = path_location.get<size_t>("location_index");
    PathLocation p(locations[index]);
    for(const auto& edge : path_location.get_child("edges")) {
      p.edges.emplace_back(GraphId(edge.second.get<uint64_t>("id")), edge.second.get<float>("dist"),
        midgard::PointLL(edge.second.get<double>("projected.lon"), edge.second.get<double>("projected.lat")),
        edge.second.get<float>("score"), static_cast<SideOfStreet>(edge.second.get<int>("sos")), edge.second.get<int>("minimum_reachability"));
    }
    for (const auto& edge : path_location.get_child("filtered_edges")) {
      p.filtered_edges.emplace_back(GraphId(edge.second.get<uint64_t>("id")), edge.second.get<float>("dist"),
        midgard::PointLL(edge.second.get<double>("projected.lon"), edge.second.get<double>("projected.lat")),
        edge.second.get<float>("score"), static_cast<SideOfStreet>(edge.second.get<int>("sos")), edge.second.get<int>("minimum_reachability"));
    }
    return p;
  }

  rapidjson::Value PathLocation::PathEdgeToRapidJson(const PathEdge &edge, rapidjson::Document::AllocatorType& allocator) const {
    rapidjson::Value e{rapidjson::kObjectType};

    e.AddMember("id", edge.id.value, allocator)
     .AddMember("dist", edge.dist, allocator)
     .AddMember("sos", static_cast<int>(edge.sos), allocator)
     .AddMember("score", edge.score, allocator)
     .AddMember("minimum_reachability", edge.minimum_reachability, allocator);

    // Serialize projected lat,lng as double (otherwise leads to shape
    // artifacts at begin/end of routes as the float values are rounded
    rapidjson::Value vtx{rapidjson::kObjectType};
    vtx.AddMember("lon", static_cast<double>(edge.projected.first), allocator)
       .AddMember("lat", static_cast<double>(edge.projected.second), allocator);
    e.AddMember("projected", vtx.Move(), allocator);

    return e;
  }
}
}
