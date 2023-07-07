#ifndef __VALHALLA_INCIDENTS_UTILS_H__
#define __VALHALLA_INCIDENTS_UTILS_H__

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/openlr.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/traffictile.h>
#include <valhalla/proto/api.pb.h>

namespace valhalla {
namespace incidents {

struct OpenLrEdge {
  baldr::GraphId edge_id;
  uint8_t first_node_offset; // offset from edge start in 1/255th steps
  uint8_t last_node_offset;  // offset from edge start in 1/255th steps
  bool is_last;              // if it's the last edge for an openlr, will be embedded in a vector

  OpenLrEdge() = delete;
  OpenLrEdge(const baldr::GraphId i)
      : edge_id(i), first_node_offset(0), last_node_offset(0), is_last(false) {
  }

  bool operator==(const OpenLrEdge& rhs) {
    return edge_id == rhs.edge_id;
  }
};

struct RankEdge {
  // keep the attributes from /locate
  baldr::GraphId graph_id;
  float length;
  float percent_along;
  double corr_lon;
  double corr_lat;

  // used for ranking
  uint8_t frc_diff;
  uint32_t dist_diff; // that's the distance between LRP & /locate correlated point
  uint16_t heading_diff;

  RankEdge(baldr::GraphId id, float l, float pct, double lng, double lat)
      : graph_id(id), length(l), percent_along(pct), corr_lon(lng), corr_lat(lat) {
  }
};

class GraphReaderIncidents : public baldr::GraphReader {
public:
  using baldr::GraphReader::tile_extract_;
  explicit GraphReaderIncidents(const boost::property_tree::ptree& pt)
      : baldr::GraphReader(pt, nullptr, true) {
  }
};

// return the route request
void get_locate_req(Api& request,
                    const baldr::OpenLR::LocationReferencePoint& lrp,
                    const bool flip_bearing);

/**
 * Constructs the route request
 *.
 * @param request the request to be mutated
 * @param a_pt    the start point & LRP bearing
 * @param b_pt    the destination point & LRP bearing
 * @param use_bearing whether we even want to use the bearing
 */
void get_route_req(Api& request,
                   const std::pair<midgard::PointLL, uint32_t>&& a_pt,
                   const std::pair<midgard::PointLL, uint32_t>&& b_pt,
                   const bool use_bearing);

void print_route(const valhalla::TripLeg& res);

} // namespace incidents
} // namespace valhalla

#endif // __VALHALLA_INCIDENTS_UTILS_H__
