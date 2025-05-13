#include <cstdarg>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <geos_c.h>

#include <valhalla/baldr/json.h>
#include <valhalla/loki/polygon_search.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
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

// map of tile for map of bin ids & their ring ids
// TODO: simplify the logic behind this a little
using bins_collector =
    std::unordered_map<uint32_t, std::unordered_map<unsigned short, std::vector<size_t>>>;

static const auto Haversine = [] {
  return bg::strategy::distance::haversine<float>(vm::kRadEarthMeters);
};

void correct_ring(ring_bg_t& ring) {
  // close open rings
  bool is_open =
      (ring.begin()->lat() != ring.rbegin()->lat() || ring.begin()->lng() != ring.rbegin()->lng());
  if (!ring.empty() && is_open) {
    ring.push_back(ring[0]);
  }

  // reverse ring if counter-clockwise
  if (vm::polygon_area(ring) > 0) {
    std::reverse(ring.begin(), ring.end());
  }
}

ring_bg_t PBFToRing(const valhalla::Ring& ring_pbf) {
  ring_bg_t new_ring;
  for (const auto& coord : ring_pbf.coords()) {
    new_ring.push_back({coord.lng(), coord.lat()});
  }
  correct_ring(new_ring);
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

geos_t::geos_t() : ctx(GEOS_init_r()) {
  LOG_WARN("GEOS Init");
}
geos_t::~geos_t() {
  finishGEOS_r(ctx);
}
GEOSGeometry* geos_t::linestring_from_shape(const std::vector<PointLL>& shape) const {
  GEOSCoordSequence* geos_coords = GEOSCoordSeq_create_r(ctx, shape.size(), 2);
  size_t i = 0;
  for (const auto& coord : shape) {
    GEOSCoordSeq_setX_r(ctx, geos_coords, i, coord.lng());
    GEOSCoordSeq_setY_r(ctx, geos_coords, i, coord.lat());
    ++i;
  }

  return GEOSGeom_createLineString_r(ctx, geos_coords);
}
GEOSGeometry* geos_t::polygon_from_pbf(const valhalla::Ring& ring) const {
  // possibly need to close this ring
  auto& first = ring.coords().at(0);
  auto& last = ring.coords().at(ring.coords_size() - 1);
  bool closed = first.lat() == last.lat() && first.lng() == last.lng();
  GEOSCoordSequence* geos_coords =
      GEOSCoordSeq_create_r(ctx, closed ? ring.coords_size() : ring.coords_size() + 1, 2);
  size_t i = 0;
  for (const auto& coord : ring.coords()) {
    GEOSCoordSeq_setX_r(ctx, geos_coords, i, coord.lng());
    GEOSCoordSeq_setY_r(ctx, geos_coords, i, coord.lat());
    ++i;
  }

  if (!closed) {
    GEOSCoordSeq_setX_r(ctx, geos_coords, i, ring.coords().at(0).lng());
    GEOSCoordSeq_setY_r(ctx, geos_coords, i, ring.coords().at(0).lat());
  }

  return GEOSGeom_createPolygon_r(ctx, GEOSGeom_createLinearRing_r(ctx, geos_coords), nullptr, 0);
}

std::unordered_set<vb::GraphId>
edges_in_polygons(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings_pbf,
                  baldr::GraphReader& reader,
                  const std::shared_ptr<sif::DynamicCost>& costing,
                  float max_length,
                  const loki::geos_t& geos_helper) {
  // protect for bogus input
  if (rings_pbf.empty() || rings_pbf.Get(0).coords().empty() ||
      !rings_pbf.Get(0).coords()[0].has_lat_case() || !rings_pbf.Get(0).coords()[0].has_lng_case()) {
    return {};
  }

  // convert to bg object and check length restriction
  const GEOSPreparedGeometry* multipolygon = nullptr;
  double rings_length = 0;
  {
    std::vector<GEOSGeometry*> polygons;
    polygons.reserve(rings_pbf.size());
    size_t perimeter = 0;
    for (const auto& ring_pbf : rings_pbf) {
      for (size_t i = 1; i < ring_pbf.coords_size(); ++i) {
        auto approx = DistanceApproximator(
            PointLL{ring_pbf.coords().at(i - 1).lng(), ring_pbf.coords().at(i - 1).lat()});
        auto this_ll = PointLL{ring_pbf.coords().at(i).lng(), ring_pbf.coords().at(i).lat()};
        perimeter += approx.DistanceSquared(this_ll);
        if (perimeter > max_length) {
          throw valhalla_exception_t(167,
                                     std::to_string(static_cast<size_t>(max_length)) + " meters");
        }
      }
      polygons.push_back(geos_helper.polygon_from_pbf(ring_pbf));
    }
    auto* collection = GEOSGeom_createCollection_r(geos_helper.ctx, GEOS_MULTIPOLYGON,
                                                   polygons.data(), polygons.size());
    if (collection == NULL)
      throw std::runtime_error("Bad polygon");
    multipolygon = GEOSPrepare_r(geos_helper.ctx, collection);

    if (multipolygon == NULL)
      throw std::runtime_error("Bad polygon");
  }

  // keep track which tile's bins intersect which rings
  bins_collector bins_intersected;
  std::unordered_set<vb::GraphId> avoid_edge_ids;
  // Get the lowest level and tiles
  const auto& tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  {
    std::vector<ring_bg_t> rings_bg;
    for (const auto& ring_pbf : rings_pbf) {
      rings_bg.push_back(PBFToRing(ring_pbf));
      const ring_bg_t ring_bg = rings_bg.back();
    }

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
  }
  size_t nbins = 0;
  for (const auto& intersection : bins_intersected) {
    auto tile = reader.GetGraphTile({intersection.first, bin_level, 0});
    if (!tile) {
      continue;
    }
    for (const auto& bin : intersection.second) {
      nbins++;
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
        auto& shape = edge_info.shape();
        GEOSGeometry* line = geos_helper.linestring_from_shape(shape);
        if (line == NULL)
          throw std::runtime_error("Bad line");

        bool intersects = GEOSPreparedIntersects_r(geos_helper.ctx, multipolygon, line);

        GEOSGeom_destroy_r(geos_helper.ctx, line);

        if (intersects) {
          avoid_edge_ids.emplace(edge_id);
          avoid_edge_ids.emplace(
              opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
        }
      }
    }
  }

  GEOSPreparedGeom_destroy_r(geos_helper.ctx, multipolygon);
  LOG_INFO("Bins: " + std::to_string(nbins));

  return avoid_edge_ids;
}
std::unordered_set<vb::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings_pbf,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length) {
  // protect for bogus input
  if (rings_pbf.empty() || rings_pbf.Get(0).coords().empty() ||
      !rings_pbf.Get(0).coords()[0].has_lat_case() || !rings_pbf.Get(0).coords()[0].has_lng_case()) {
    return {};
  }

  // convert to bg object and check length restriction
  double rings_length = 0;
  std::vector<ring_bg_t> rings_bg;
  for (const auto& ring_pbf : rings_pbf) {
    rings_bg.push_back(PBFToRing(ring_pbf));
    const ring_bg_t ring_bg = rings_bg.back();
    rings_length += bg::perimeter(ring_bg, Haversine());
  }
  if (rings_length > max_length) {
    throw valhalla_exception_t(167, std::to_string(static_cast<size_t>(max_length)) + " meters");
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

  return avoid_edge_ids;
}
} // namespace loki
} // namespace valhalla
