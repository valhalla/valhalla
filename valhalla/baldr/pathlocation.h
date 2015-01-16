#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <vector>
#include <utility>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla{
namespace baldr{

/**
 * The graph correlated location object providing the path finding
 * algorithm the information to actually compute a path
 *
 * @author  Kevin Kreiser
 */
class PathLocation : public Location {
 public:
  using Location::Location;

  /**
   * Structure to store information about a given location correlated edge
   */
  struct PathEdge {
    PathEdge(const GraphId& id, const float dist);
    //the directed edge it appears on
    GraphId id;
    //how far along the edge it is (as a percentage  from 0 - 1)
    float dist;
  };

  /**
   * Get the edges associated with this location
   * @return edges
   */
  const std::vector<PathEdge>& edges() const;

  /*
   * Get the vertex associated with this location
   * @return vertex
   */
  const midgard::PointLL& vertex() const;

  /**
   * Whether or not this location is on a vertex in the graph (intersection)
   * note: this implies all distances on edges are either 0's or 1's
   *
   * @return bool True if its on an intersection false if not
   */
  bool IsNode() const;

  /**
   * Adds a correlated edge to the path location
   *
   * @param id      the graphid of the edge
   * @param dist    the distance along the provided edge where the location was correlated
   */
  void CorrelateEdge(const GraphId& id, const float dist);

  /**
   * @return true if the point has been correlated to the route network, false otherwise
   */
  bool IsCorrelated() const;

  /**
   * Set the route network correlation point for this location
   * @param point the correlation point
   */
  void CorrelateVertex(const midgard::PointLL& correlated);

 protected:
  //whether or not this location is on a vertex in the graph (intersection)
  //note: this implies all distances on edges are either 0's or 1's
  bool node_;

  //list of edges this location appears on within the graph
  std::vector<PathEdge> edges_;

  //correlated point in the graph (along an edge or at a vertex)
  midgard::PointLL vertex_;

  //a confidence interval of how good the correlation is
  //proportional to the distance between the input point and the correlated one
  //float correlation_quality_;
};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
