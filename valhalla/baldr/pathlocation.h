#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <vector>
#include <utility>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphid.h>

#include <boost/property_tree/ptree.hpp>
#include <rapidjson/document.h>

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
    PathEdge(const GraphId& id, const float dist, const midgard::PointLL& projected, const float score, const SideOfStreet sos = NONE);
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
  };

  //list of edges this location appears on within the graph
  std::vector<PathEdge> edges;

  /**
   * Equality check
   * @return true if they are equal
   */
  bool operator==(const PathLocation& other) const;

  /**
   * Serializes this object to ptree
   * @return ptree
   */
  boost::property_tree::ptree ToPtree(size_t index) const;

  /**
   * Serializes this object to rapidjson
   * @return rapidjson::Value
   */
  rapidjson::Value ToRapidJson(size_t index, rapidjson::Document::AllocatorType& allocator) const;

  /**
   * Serializes one of these objects from a ptree and a list of locations
   * @return PathLocation
   */
  static PathLocation FromPtree(const std::vector<Location>& locations, const boost::property_tree::ptree& path_location);
};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
