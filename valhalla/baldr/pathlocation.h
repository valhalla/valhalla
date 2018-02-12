#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <cstdint>
#include <vector>
#include <utility>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>

namespace valhalla{
namespace baldr{

/**
 * The graph correlated location object providing the path finding
 * algorithm the information to actually compute a path
 */
struct PathLocation : public Location {
 public:
  using Location::Location;
  PathLocation(const Location& location);

  /**
   * Structure to store information about a given location correlated edge
   */
  enum SideOfStreet { NONE = 0, LEFT, RIGHT };
  struct PathEdge {
    PathEdge(const GraphId& id, const float dist, const midgard::PointLL& projected, const float score,
      const SideOfStreet sos = NONE, const int minimum_reachability = -1);
    //the directed edge it appears on
    GraphId id;
    //how far along the edge it is (as a percentage  from 0 - 1)
    float dist;
    //the projected point along the edge where the original location correlates
    midgard::PointLL projected;
    //what side of the edge is it on
    SideOfStreet sos;
    //whether or not this correlation point is the begin node of this edge
    bool begin_node() const;
    //whether or not this correlation point is the end node of this edge
    bool end_node() const;

    //a measure of how close the result is to the original input where the
    //lower the score the better the match, maybe there's a better word for this?
    float score;
    //minimum number of edges reachable from this edge, this is a lower limit
    //it could be reachable from many many more edges than are reported here
    int minimum_reachability;
  };

  //list of edges this location appears on within the graph
  std::vector<PathEdge> edges;

  //list of edges this location appears on within the graph but are filtered because of heading or
  //something else, used for get_path retry when thor_worker_t::get_path with PathLocation.edges failed
  std::vector<PathEdge> filtered_edges;

  /**
   * Equality check
   * @return true if they are equal
   */
  bool operator==(const PathLocation& other) const;

  /**
   * Serializes this object to rapidjson
   * @return rapidjson::Value
   */
  rapidjson::Value ToRapidJson(size_t index, rapidjson::Document::AllocatorType& allocator) const;

  // Serialize this edge to rapidjson
  rapidjson::Value PathEdgeToRapidJson(const PathEdge &edge, rapidjson::Document::AllocatorType& allocator) const;

  /**
   * Serializes one of these objects from a ptree and a list of locations
   * @return PathLocation
   */
  static PathLocation FromRapidJson(const std::vector<Location>& locations, const rapidjson::Value& path_location);
};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
