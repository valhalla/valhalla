#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/loki/polygon_search.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/worker.h>

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;

BOOST_GEOMETRY_REGISTER_POINT_2D(vm::PointLL, double, bg::cs::geographic<bg::degree>, first, second)
BOOST_GEOMETRY_REGISTER_RING(std::vector<vm::PointLL>)

namespace {
// register a few boost.geometry types
using line_bg_t = bg::model::linestring<vm::PointLL>;
using ring_bg_t = std::vector<vm::PointLL>;
using namespace vb::json;

typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::polygon<point_type> polygon_type;
typedef bg::model::linestring<point_type> linestring_t;

// map of tile for map of bin ids & their ring ids
// TODO: simplify the logic behind this a little
using bins_collector =
    std::unordered_map<uint32_t, std::unordered_map<unsigned short, std::vector<size_t>>>;

static const auto Haversine = [] {
  return bg::strategy::distance::haversine<float>(vm::kRadEarthMeters);
};

ring_bg_t PBFToRing(const valhalla::Options::Ring& ring_pbf) {
  ring_bg_t new_ring;
  for (const auto& coord : ring_pbf.coords()) {
    new_ring.push_back({coord.lng(), coord.lat()});
  }
  // fixes windedness & closes open rings
  bg::correct(new_ring);
  return new_ring;
}

#ifdef LOGGING_LEVEL_TRACE
// serializes an edge to geojson
std::string to_geojson(const std::unordered_set<vb::GraphId>& edge_ids, vb::GraphReader& reader) {
  auto features = array({});
  for (const auto& edge_id : edge_ids) {
    auto tile = reader.GetGraphTile(edge_id);
    auto edge = tile->directededge(edge_id);
    auto shape = tile->edgeinfo(edge).shape();
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }

    auto coords = array({});
    for (const auto& p : shape) {
      coords->emplace_back(array({fixed_t{p.lng(), 6}, fixed_t{p.lat(), 6}}));
    }
    features->emplace_back(
        map({{"type", std::string("Feature")},
             {"properties",
              map({{"shortcut", edge->is_shortcut() ? std::string("True") : std::string("False")},
                   {"edge_id", edge_id.value}})},
             {"geometry", map({{"type", std::string("LineString")}, {"coordinates", coords}})}}));
  }

  auto collection =
      vb::json::map({{"type", std::string("FeatureCollection")}, {"features", features}});

  std::stringstream ss;
  ss << *collection;

  return ss.str();
}
#endif // LOGGING_LEVEL_TRACE
} // namespace

namespace valhalla {
namespace loki {

// This custon within is created because using the boost::within for edge_info
// shape will give different result depending on the order of the point.
bool IsWithin(vb::EdgeInfo edge_info, polygon_type polygon) {
  // create temporary line from edge_info
  point_type p1(edge_info.shape()[0].first, edge_info.shape()[0].second);
  point_type p2(edge_info.shape()[1].first, edge_info.shape()[1].second);
  linestring_t edge_line{p1, p2};
  return bg::within(edge_line, polygon);
}

std::unordered_set<vb::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Options_Ring>& rings_pbf,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length) {
  // convert to bg object and check length restriction
  double rings_length = 0;
  std::vector<ring_bg_t> rings_bg;
  for (const auto& ring_pbf : rings_pbf) {
    rings_bg.push_back(PBFToRing(ring_pbf));
    const ring_bg_t ring_bg = rings_bg.back();
    rings_length += bg::perimeter(ring_bg, Haversine());
  }
  if (rings_length > max_length) {
    throw valhalla_exception_t(167, std::to_string(max_length));
  }

  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  // keep track which tile's bins intersect which rings
  bins_collector bins_intersected;
  std::unordered_set<vb::GraphId> avoid_edge_ids;

  // first pull out all *unique* bins which intersect the rings
  for (size_t ring_idx = 0; ring_idx < rings_bg.size(); ring_idx++) {
    auto ring = rings_bg[ring_idx];
    auto line_intersected = tiles.Intersect(ring);
    for (const auto& tb : line_intersected) {
      for (const auto& b : tb.second) {
        bins_intersected[static_cast<uint32_t>(tb.first)][b].push_back(ring_idx);
      }
    }
  }
  for (const auto& intersection : bins_intersected) {
    auto tile = reader.GetGraphTile({intersection.first, bin_level, 0});
    if (!tile) {
      continue;
    }
    for (const auto& bin : intersection.second) {
      // tile will be mutated most likely in the loop
      reader.GetGraphTile({intersection.first, bin_level, 0}, tile);
      for (const auto& edge_id : tile->GetBin(bin.first)) {
        if (avoid_edge_ids.count(edge_id) != 0) {
          continue;
        }
        // TODO: optimize the tile switching by enqueuing edges
        // from other levels & tiles and process them after this big loop
        if (edge_id.Tile_Base() != tile->header()->graphid().Tile_Base() &&
            !reader.GetGraphTile(edge_id, tile)) {
          continue;
        }
        const auto edge = tile->directededge(edge_id);
        auto opp_tile = tile;
        const baldr::DirectedEdge* opp_edge = nullptr;
        baldr::GraphId opp_id;

        // bail if we wouldnt be allowed on this edge anyway (or its opposing)
        if (!costing->Allowed(edge, tile) &&
            (!(opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)).Is_Valid() ||
             !costing->Allowed(opp_edge, opp_tile))) {
          continue;
        }

        // TODO: some logic to set percent_along for origin/destination edges
        // careful: polygon can intersect a single edge multiple times
        auto edge_info = tile->edgeinfo(edge);
        bool intersects = false;
        for (const auto& ring_loc : bin.second) {
          intersects = bg::intersects(rings_bg[ring_loc],
                                      line_bg_t(edge_info.shape().begin(), edge_info.shape().end()));
          if (intersects) {
            break;
          }
        }
        if (intersects) {
          avoid_edge_ids.emplace(edge_id);
          avoid_edge_ids.emplace(
              opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
        }
      }
    }
  }

// log the GeoJSON of avoided edges
#ifdef LOGGING_LEVEL_TRACE
  if (!avoid_edge_ids.empty()) {
    LOG_TRACE("Avoided edges GeoJSON: \n" + to_geojson(avoid_edge_ids, reader));
  }
#endif
  std::cout << "num avoid_edge_ids: " << avoid_edge_ids.size() << "\n";
  return avoid_edge_ids;
}

// TODO: Probable merged the logic with edges_in_rings above (if possible)
// TODO: Refactor the variable name (i.e. it's not specific to avoid edge)
std::unordered_set<vb::GraphId> edges_in_ring(const valhalla::Options_Ring& ring_pbf,
                                              baldr::GraphReader& reader,
                                              const std::shared_ptr<sif::DynamicCost>& costing,
                                              float max_length) {
  // convert to bg object and check length restriction
  const ring_bg_t ring_bg = PBFToRing(ring_pbf);
  if (bg::perimeter(ring_bg, Haversine()) > max_length) {
    throw valhalla_exception_t(167, std::to_string(max_length));
  }
  polygon_type polygon;

  std::vector<point_type> points;
  for (auto p : ring_bg) {
    // point_type new_point(p.first, p.second);
    // points.push_back(point_type(p.first, p.second));
    bg::append(polygon.outer(), point_type(p.first, p.second));
  }
  // polygon_type polygon{{points}};
  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  // keep track which tile's bins intersect which rings
  bins_collector bins_intersected;
  std::unordered_set<vb::GraphId> cp_edge_ids;

  // first pull out all *unique* bins which intersect the ring
  auto line_intersected = tiles.Intersect(ring_bg);
  for (const auto& tb : line_intersected) {
    for (const auto& b : tb.second) {
      bins_intersected[static_cast<uint32_t>(tb.first)][b].push_back(0);
    }
  }
  for (const auto& intersection : bins_intersected) {
    auto tile = reader.GetGraphTile({intersection.first, bin_level, 0});
    if (!tile) {
      continue;
    }
    for (const auto& bin : intersection.second) {
      // tile will be mutated most likely in the loop
      reader.GetGraphTile({intersection.first, bin_level, 0}, tile);
      for (const auto& edge_id : tile->GetBin(bin.first)) {
        if (cp_edge_ids.count(edge_id) != 0) {
          continue;
        }
        // TODO: optimize the tile switching by enqueuing edges
        // from other levels & tiles and process them after this big loop
        if (edge_id.Tile_Base() != tile->header()->graphid().Tile_Base() &&
            !reader.GetGraphTile(edge_id, tile)) {
          continue;
        }
        const auto edge = tile->directededge(edge_id);
        auto opp_tile = tile;
        const baldr::DirectedEdge* opp_edge = nullptr;
        baldr::GraphId opp_id;

        // bail if we wouldnt be allowed on this edge anyway (or its opposing)
        if (!costing->Allowed(edge, tile) &&
            (!(opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)).Is_Valid() ||
             !costing->Allowed(opp_edge, opp_tile))) {
          continue;
        }
        bool is_within = IsWithin(tile->edgeinfo(edge), polygon);
        if (is_within) {
          if (costing->IsAccessible(edge)) {
            cp_edge_ids.emplace(edge_id);
          }
          // Add the opposite edge if a two way.
          opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile);
          if (costing->IsAccessible(opp_edge)) {
            cp_edge_ids.emplace(opp_id);
          }
        }
      }
    }
  }
  std::cout << "num cp_edge_ids: " << cp_edge_ids.size() << "\n";
  return cp_edge_ids;
}

} // namespace loki
} // namespace valhalla
