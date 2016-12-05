#include "baldr/pathlocation.h"
#include <valhalla/midgard/util.h>

namespace valhalla{
namespace baldr{

  PathLocation::PathEdge::PathEdge(const GraphId& id, const float dist,
    const midgard::PointLL& projected, const float score, const SideOfStreet sos):
    id(id), dist(dist), projected(projected), sos(sos), score(score) {
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
            midgard::equal<float>(edge.score, other_edge.score, .1f) && edge.projected.ApproximatelyEqual(other_edge.projected)){
          found = true;
          break;
        }
      }
      if(!found)
        return false;
    }
    return true;
  }

  boost::property_tree::ptree PathLocation::ToPtree(size_t index) const {
    boost::property_tree::ptree correlated;
    auto& array = correlated.put_child("edges", boost::property_tree::ptree());
    for(const auto& edge : edges) {
      boost::property_tree::ptree e;
      e.put("id", edge.id.value);
      e.put("dist", edge.dist);
      e.put("sos", static_cast<int>(edge.sos));
      e.put("score", edge.score);

      // Serialize projected lat,lng as double (otherwise leads to shape
      // artifacts at begin/end of routes as the float values are rounded
      auto& vtx = e.put_child("projected", boost::property_tree::ptree());
      vtx.put("lon", static_cast<double>(edge.projected.first));
      vtx.put("lat", static_cast<double>(edge.projected.second));
      array.push_back(std::make_pair("", e));
    }
    correlated.put("location_index", index);
    return correlated;
  }

  PathLocation PathLocation::FromPtree(const std::vector<Location>& locations, const boost::property_tree::ptree& path_location){
    auto index = path_location.get<size_t>("location_index");
    PathLocation p(locations[index]);
    for(const auto& edge : path_location.get_child("edges")) {
      p.edges.emplace_back(GraphId(edge.second.get<uint64_t>("id")), edge.second.get<float>("dist"),
        midgard::PointLL(edge.second.get<double>("projected.lon"), edge.second.get<double>("projected.lat")),
        edge.second.get<float>("score"), static_cast<SideOfStreet>(edge.second.get<int>("sos")));
    }
    return p;
  }

}
}
