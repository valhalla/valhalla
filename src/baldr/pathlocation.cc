#include "baldr/pathlocation.h"

namespace valhalla{
namespace baldr{

  PathLocation::PathEdge::PathEdge(const GraphId& id, const float dist): id(id), dist(dist) {
  }

  PathLocation::PathLocation(const Location& location):location_(location), node_(false) {
  }

  bool PathLocation::IsNode() const {
    return node_;
  }

  void PathLocation::Correlate(const GraphId& id, const float dist) {
    //whether or not we are only correlated to nodes in the graph
    node_ = (node_ || edges_.size() == 0) && (1 == dist || dist == 0);
    //add the edge
    edges_.emplace_back(id, dist);
  }

  const std::vector<PathLocation::PathEdge>& PathLocation::edges() const {
    return edges_;
  }

  const Location& PathLocation::location() const {
    return location_;
  }


}
}
