#include "baldr/pathlocation.h"

namespace valhalla{
namespace baldr{

  PathLocation::PathEdge::PathEdge(const GraphId& id, const float dist): id(id), dist(dist) {
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

}
}
