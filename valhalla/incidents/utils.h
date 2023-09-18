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
  float length;            // the full edge length
  float poff_start_offset; // meter offset of poff in positiv direction of openlr, 0 is no offset
  float noff_start_offset; // meter offset of noff in negative direction of openlr, 0 is no offset
  bool is_last;            // if it's the last edge for an openlr, so we can plot them as GeoJSON

  OpenLrEdge() = delete;
  // only allow this constructor: is_last will be set after initially collecting all edges
  OpenLrEdge(const baldr::GraphId i, const float le, const float po, const float no)
      : edge_id(i), length(le), poff_start_offset(po), noff_start_offset(no), is_last(false) {
  }

  // for this constructor, poff_start_offset is temporarilly used as "matched_length"
  OpenLrEdge(const baldr::GraphId i, const float le, const float po)
      : edge_id(i), length(le), poff_start_offset(po), noff_start_offset(0.f), is_last(false) {
  }

  // for sorting purposes
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
  std::unordered_map<valhalla::baldr::GraphId, valhalla::baldr::GraphId> edges_to_shorcuts;
  explicit GraphReaderIncidents(const boost::property_tree::ptree& pt)
      : baldr::GraphReader(pt, nullptr, false) {
    // find all superseded edges and build the cached map
    for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
      // we dont get shortcuts on level 2 and up
      if (level.level > 1)
        continue;
      // for each tile
      for (auto tile_id : GetTileSet(level.level)) {
        // cull cache if we are over allocated
        if (OverCommitted())
          Trim();
        // this shouldnt fail but garbled files could cause it
        auto tile = GetGraphTile(tile_id);
        assert(tile);
        // for each edge in the tile
        for (const auto& edge : tile->GetDirectedEdges()) {
          // skip non-shortcuts
          if (!edge.shortcut())
            continue;

          auto shortcut_id = tile->header()->graphid();
          shortcut_id.set_id(&edge - tile->directededge(0));

          for (const auto& r_id : RecoverShortcut(shortcut_id)) {
            edges_to_shorcuts[r_id] = shortcut_id;
          }
        }
      }
    }
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
