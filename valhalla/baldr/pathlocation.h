#ifndef VALHALLA_BALDR_PATHLOCATION_H_
#define VALHALLA_BALDR_PATHLOCATION_H_

#include <cstdint>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/rapidjson_utils.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace baldr {

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
             const float dist,
             const midgard::PointLL& projected,
             const float score,
             const SideOfStreet sos = NONE,
             const unsigned int outbound_reach = 0,
             const unsigned int inbound_reach = 0);
    // the directed edge it appears on
    GraphId id;
    // how far along the edge it is (as a percentage  from 0 - 1)
    float percent_along;
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
    float distance;
    // minimum number of nodes reachable from this edge
    unsigned int outbound_reach;
    // minimum number of nodes that can reach this edge
    unsigned int inbound_reach;
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

  static void toPBF(const PathLocation& pl, valhalla::Location* l, baldr::GraphReader& reader) {
    l->mutable_ll()->set_lng(pl.latlng_.first);
    l->mutable_ll()->set_lat(pl.latlng_.second);
    l->set_type(valhalla::Location::kBreak);
    if (pl.stoptype_ == Location::StopType::THROUGH)
      l->set_type(valhalla::Location::kThrough);
    else if (pl.stoptype_ == Location::StopType::VIA)
      l->set_type(valhalla::Location::kVia);
    else if (pl.stoptype_ == Location::StopType::BREAK_THROUGH)
      l->set_type(valhalla::Location::kBreakThrough);

    l->set_preferred_side(valhalla::Location::either);
    if (pl.preferred_side_ == Location::PreferredSide::SAME)
      l->set_preferred_side(valhalla::Location::same);
    else if (pl.preferred_side_ == Location::PreferredSide::OPPOSITE)
      l->set_preferred_side(valhalla::Location::opposite);

    if (!pl.name_.empty()) {
      l->set_name(pl.name_);
    }
    if (!pl.street_.empty()) {
      l->set_street(pl.street_);
    }
    if (!pl.city_.empty()) {
      l->set_city(pl.city_);
    }
    if (!pl.state_.empty()) {
      l->set_state(pl.state_);
    }
    if (!pl.zip_.empty()) {
      l->set_postal_code(pl.zip_);
    }
    if (!pl.country_.empty()) {
      l->set_country(pl.country_);
    }
    if (pl.date_time_) {
      l->set_date_time(*pl.date_time_);
    }
    if (pl.heading_) {
      l->set_heading(*pl.heading_);
    }
    l->set_heading_tolerance(pl.heading_tolerance_);
    l->set_node_snap_tolerance(pl.node_snap_tolerance_);
    if (pl.way_id_) {
      l->set_way_id(*pl.way_id_);
    }
    l->set_minimum_reachability(std::max(pl.min_outbound_reach_, pl.min_inbound_reach_));
    l->set_radius(pl.radius_);
    l->set_search_cutoff(pl.radius_ > pl.search_cutoff_ ? pl.radius_ : pl.search_cutoff_);
    l->set_street_side_tolerance(pl.street_side_tolerance_);

    auto* path_edges = l->mutable_path_edges();
    for (const auto& e : pl.edges) {
      auto* edge = path_edges->Add();
      edge->set_graph_id(e.id);
      edge->set_percent_along(e.percent_along);
      edge->set_begin_node(e.percent_along == 0.0f);
      edge->set_end_node(e.percent_along == 1.0f);
      edge->mutable_ll()->set_lng(e.projected.first);
      edge->mutable_ll()->set_lat(e.projected.second);
      edge->set_side_of_street(e.sos == PathLocation::LEFT
                                   ? valhalla::Location::kLeft
                                   : (e.sos == PathLocation::RIGHT ? valhalla::Location::kRight
                                                                   : valhalla::Location::kNone));
      edge->set_distance(e.distance);
      edge->set_outbound_reach(e.outbound_reach);
      edge->set_inbound_reach(e.inbound_reach);
      for (const auto& n : reader.edgeinfo(e.id).GetNames()) {
        edge->mutable_names()->Add()->assign(n);
      }
    }

    auto* filtered_edges = l->mutable_filtered_edges();
    for (const auto& e : pl.edges) {
      auto* edge = filtered_edges->Add();
      edge->set_graph_id(e.id);
      edge->set_percent_along(e.percent_along);
      edge->mutable_ll()->set_lng(e.projected.first);
      edge->mutable_ll()->set_lat(e.projected.second);
      edge->set_side_of_street(e.sos == PathLocation::LEFT
                                   ? valhalla::Location::kLeft
                                   : (e.sos == PathLocation::RIGHT ? valhalla::Location::kRight
                                                                   : valhalla::Location::kNone));
      edge->set_distance(e.distance);
      edge->set_outbound_reach(e.outbound_reach);
      edge->set_inbound_reach(e.inbound_reach);
      for (const auto& n : reader.edgeinfo(e.id).GetNames()) {
        edge->mutable_names()->Add()->assign(n);
      }
    }
  }

  static Location fromPBF(const valhalla::Location& loc) {
    auto stop_type = Location::StopType::BREAK;
    if (loc.type() == valhalla::Location::kThrough)
      stop_type = Location::StopType::THROUGH;
    else if (loc.type() == valhalla::Location::kVia)
      stop_type = Location::StopType::VIA;
    else if (loc.type() == valhalla::Location::kBreakThrough)
      stop_type = Location::StopType::BREAK_THROUGH;
    auto side = PreferredSide::EITHER;
    if (loc.preferred_side() == valhalla::Location::same)
      side = PreferredSide::SAME;
    else if (loc.preferred_side() == valhalla::Location::opposite)
      side = PreferredSide::OPPOSITE;
    Location l({loc.ll().lng(), loc.ll().lat()}, stop_type, loc.minimum_reachability(),
               loc.minimum_reachability(), loc.radius(), side);

    if (loc.has_name()) {
      l.name_ = loc.name();
    }
    if (loc.has_street()) {
      l.street_ = loc.street();
    }
    if (loc.has_city()) {
      l.city_ = loc.city();
    }
    if (loc.has_state()) {
      l.state_ = loc.state();
    }
    if (loc.has_postal_code()) {
      l.zip_ = loc.postal_code();
    }
    if (loc.has_country()) {
      l.country_ = loc.country();
    }
    if (loc.has_date_time()) {
      l.date_time_ = loc.date_time();
    }
    if (loc.has_heading()) {
      l.heading_ = loc.heading();
    }
    if (loc.has_heading_tolerance()) {
      l.heading_tolerance_ = loc.heading_tolerance();
    }
    if (loc.has_node_snap_tolerance()) {
      l.node_snap_tolerance_ = loc.node_snap_tolerance();
    }
    if (loc.has_way_id()) {
      l.way_id_ = loc.way_id();
    }
    if (loc.has_search_cutoff()) {
      l.search_cutoff_ = loc.search_cutoff();
    }
    if (loc.has_street_side_tolerance()) {
      l.street_side_tolerance_ = loc.street_side_tolerance();
    }
    return l;
  }

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
      pls.front().min_inbound_reach_ = 0;
      pls.back().min_outbound_reach_ = 0;
    }
    return pls;
  }
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PATHLOCATION_H_
