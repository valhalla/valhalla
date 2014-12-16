#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <string>
#include "baldr/location.h"
#include "baldr/graphid.h"

namespace valhalla{
namespace baldr{

/**
 * The graph correlated location object providing the path finding
 * algorithm the information to actually compute a path
 *
 * @author  Kevin Kreiser
 */
struct PathLocation {
 public:
  /**
   * Constructor.
   * @param  latlng  the polar coordinates of the location
   */
  PathLocation(const Location& location);

  /**
   * Structure to store information about a given location correlated edge
   */
  struct Edge {
    //the directed edge it appears on
    GraphId id_;
    //how far along the edge it is (as a percentage  from 0 - 1)
    float dist_;
  };

  //list of edges this location appears on within the graph
  std::vector<Edge> edges_;

  //the location object used to correlate to the graph network
  Location location_;

  /**
   * Whether or not this location is on a vertex in the graph (intersection)
   * note: this implies all distances on edges are either 0's or 1's
   *
   * @return bool True if its on an interesction false if not
   */
  bool IsVertex() const;

 private:
  //whether or not this location is on a vertex in the graph (intersection)
  //note: this implies all distances on edges are either 0's or 1's
  mutable bool vertex_;

};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
