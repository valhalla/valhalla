#include "baldr/pathlocation.h"
#include <valhalla/midgard/util.h>

namespace valhalla{
namespace baldr{

  PathLocation::PathEdge::PathEdge(const GraphId& id, const float dist, const SideOfStreet sos): id(id), dist(dist), sos(sos) {
  }

  PathLocation::PathLocation(const Location& location):Location(location) {
    node_ = false;
    edges_.reserve(16);
  }

  bool PathLocation::IsNode() const {
    return node_;
  }

  void PathLocation::CorrelateEdge(const GraphId& id, const float dist) {
    //whether or not we are only correlated to nodes in the graph
    node_ = (node_ || edges_.size() == 0) && (1 == dist || dist == 0);
    //add the edge
    edges_.emplace_back(id, dist);
  }

  const std::vector<PathLocation::PathEdge>& PathLocation::edges() const {
    return edges_;
  }

  const midgard::PointLL& PathLocation::vertex() const{
    return vertex_;
  }

  bool PathLocation::IsCorrelated() const {
    return vertex_.IsValid() && edges_.size() /*&& correlation_quality_ <= 1.f*/;
  }

  void PathLocation::CorrelateVertex(const midgard::PointLL& vertex) {
    vertex_ = vertex;
  }

  void PathLocation::ClearEdges() {
    edges_.clear();
  }

  bool PathLocation::operator==(const PathLocation& other) const {
    if(node_ != other.node_ || !vertex_.ApproximatelyEqual(other.vertex_))
      return false;
    for(const auto& edge : edges_) {
      bool found = false;
      for(const auto& other_edge : other.edges_) {
        if(edge.id == other_edge.id && midgard::equal<float>(edge.dist, other_edge.dist)){
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
    for(const auto& edge : edges_) {
      boost::property_tree::ptree e;
      e.put("id", edge.id.value);
      e.put("dist", edge.dist);
      e.put("sos", edge.sos);
      array.push_back(std::make_pair("", e));
    }
    correlated.put("is_node", IsNode());
    auto& vtx = correlated.put_child("vertex", boost::property_tree::ptree());
    vtx.put("lon", vertex_.lng());
    vtx.put("lat", vertex_.lat());
    correlated.put("location_index", index);
    return correlated;
  }

  PathLocation PathLocation::FromPtree(const std::vector<Location>& locations, const boost::property_tree::ptree& path_location){
    auto index = path_location.get<size_t>("location_index");
    PathLocation p(locations[index]);
    p.node_ = path_location.get<bool>("is_node");
    p.vertex_.set_x(path_location.get<float>("vertex.lon"));
    p.vertex_.set_y(path_location.get<float>("vertex.lat"));
    for(const auto& edge : path_location.get_child("edges"))
      p.edges_.emplace_back(GraphId(edge.second.get<uint64_t>("id")), edge.second.get<float>("dist"), static_cast<SideOfStreet>(edge.second.get<int>("sos")));
    return p;
  }

}
}
