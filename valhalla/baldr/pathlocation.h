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
class PathLocation {
 public:
  /**
   * Constructor.
   * @param  latlng  the polar coordinates of the location
   */
  PathLocation(const Location& location);

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

  /**
   * Get the location associated with path location
   * @return location
   */
  const Location& location() const;

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
  void Correlate(const GraphId& id, const float dist);

 private:
  //whether or not this location is on a vertex in the graph (intersection)
  //note: this implies all distances on edges are either 0's or 1's
  bool node_;

  //list of edges this location appears on within the graph
  std::vector<PathEdge> edges_;

  //the location object used to correlate to the graph network
  Location location_;

};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
