#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/location.h>

namespace valhalla {
namespace baldr {
class GraphReader;

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
    PathEdge(const GraphId& id,
             const double percent_along,
             const midgard::PointLL& projected,
             const double score,
             const SideOfStreet sos = NONE,
             const unsigned int outbound_reach = 0,
             const unsigned int inbound_reach = 0,
             const float projected_heading = -1);
    // the directed edge it appears on
    GraphId id;
    // how far along the edge it is (as a percentage  from 0 - 1)
    double percent_along;
    // the projected point along the edge where the original location correlates
    midgard::PointLL projected;
    // what side of the edge is it on
    SideOfStreet sos;
    // whether or not this correlation point is the begin node of this edge
    bool begin_node() const;
    // whether or not this correlation point is the end node of this edge
    bool end_node() const;

    // a measure of how close the result is to the original input where the
    // lower the score the better the match, maybe there's a better word for this?
    double distance;
    // minimum number of nodes reachable from this edge
    unsigned int outbound_reach;
    // minimum number of nodes that can reach this edge
    unsigned int inbound_reach;
    // the heading of the projected point
    float projected_heading;
  };

  // list of edges this location appears on within the graph
  std::vector<PathEdge> edges;

  // list of edges this location appears on within the graph but are filtered because of heading or
  // something else, used for get_path retry when thor_worker_t::get_path with PathLocation.edges
  // failed
  std::vector<PathEdge> filtered_edges;

  /**
   * Equality check
   * @return true if they are equal
   */
  bool operator==(const PathLocation& other) const;

  /**
   * Check whether another PathLocation contains all the edges we do.
   * NOTE: This method does not care if additional edges exist.
   */
  bool shares_edges(const PathLocation& other) const;

  static void toPBF(const PathLocation& pl, valhalla::Location* l, baldr::GraphReader& reader);

  static baldr::Location::StopType fromPBF(valhalla::Location::Type type);

  static Location fromPBF(const valhalla::Location& loc);

  /**
   * Converts a list of pbf locations into a vector of custom class because loki hasnt moved to pbf
   * yet
   * @param locations
   * @param route_reach   whether or not we should treat reach differently for the first and last
   * locations
   * @return
   */
  static std::vector<Location>
  fromPBF(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
          bool route_reach = false) {
    std::vector<Location> pls;
    for (const auto& l : locations) {
      pls.emplace_back(fromPBF(l));
    }
    // for regular routing we dont really care about inbound reach for the origin or outbound reach
    // for the destination so we remove that requirement
    if (route_reach && pls.size() > 1) {
      // TODO Why only set min_reach for front/back? For routing, `pls` is only ever just size 2?
      pls.front().min_inbound_reach_ = 0;
      pls.back().min_outbound_reach_ = 0;
    }
    return pls;
  }
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PATHLOCATION_H_
