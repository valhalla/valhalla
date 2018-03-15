#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <cstdint>
#include <vector>
#include <utility>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/baldr/graphreader.h>

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
      const SideOfStreet sos = NONE, const unsigned int minimum_reachability = 0);
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
    unsigned int minimum_reachability;
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

  static void toPBF(const PathLocation& pl, odin::Location* l, baldr::GraphReader& reader) {
    l->mutable_ll()->set_lng(pl.latlng_.first);
    l->mutable_ll()->set_lat(pl.latlng_.second);
    l->set_type(pl.stoptype_ == Location::StopType::BREAK ? odin::Location::kBreak : odin::Location::kThrough);
    if(!pl.name_.empty()) l->set_name(pl.name_);
    if(!pl.street_.empty()) l->set_street(pl.street_);
    if(!pl.city_.empty()) l->set_city(pl.city_);
    if(!pl.state_.empty()) l->set_state(pl.state_);
    if(!pl.zip_.empty()) l->set_postal_code(pl.zip_);
    if(!pl.country_.empty()) l->set_country(pl.country_);
    if(pl.date_time_) l->set_date_time(*pl.date_time_);
    if(pl.heading_) l->set_heading(*pl.heading_);
    if(pl.heading_tolerance_) l->set_heading_tolerance(*pl.heading_tolerance_);
    if(pl.node_snap_tolerance_) l->set_node_snap_tolerance(*pl.node_snap_tolerance_);
    if(pl.way_id_) l->set_way_id(*pl.way_id_);
    l->set_minimum_reachability(pl.minimum_reachability_);
    l->set_radius(pl.radius_);

    auto* path_edges = l->mutable_path_edges();
    for(const auto& e : pl.edges) {
      auto* edge = path_edges->Add();
      edge->set_graph_id(e.id);
      edge->set_dist(e.dist);
      edge->mutable_ll()->set_lng(e.projected.first);
      edge->mutable_ll()->set_lat(e.projected.second);
      edge->set_side_of_street(e.sos == PathLocation::LEFT ? odin::Location::kLeft :
          (e.sos == PathLocation::RIGHT ? odin::Location::kRight : odin::Location::kNone));
      edge->set_score(e.score);
      edge->set_minimum_reachability(e.minimum_reachability);
      for(const auto& n : reader.edgeinfo(e.id).GetNames())
        edge->mutable_names()->Add()->assign(n);
    }

    auto* filtered_edges = l->mutable_filtered_edges();
    for(const auto& e : pl.edges) {
      auto* edge = filtered_edges->Add();
      edge->set_graph_id(e.id);
      edge->set_dist(e.dist);
      edge->mutable_ll()->set_lng(e.projected.first);
      edge->mutable_ll()->set_lat(e.projected.second);
      edge->set_side_of_street(e.sos == PathLocation::LEFT ? odin::Location::kLeft :
          (e.sos == PathLocation::RIGHT ? odin::Location::kRight : odin::Location::kNone));
      edge->set_score(e.score);
      edge->set_minimum_reachability(e.minimum_reachability);
      for(const auto& n : reader.edgeinfo(e.id).GetNames())
        edge->mutable_names()->Add()->assign(n);
    }
  }
};

}
}

#endif // VALHALLA_BALDR_PATHLOCATION_H_
