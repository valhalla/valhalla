#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <valhalla/loki/polygon_search.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/worker.h>

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;

BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::PointLL,
                                 double,
                                 bg::cs::geographic<bg::degree>,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_RING(std::vector<vm::PointLL>)

namespace {
// register a few boost.geometry types
using line_bg_t = bg::model::linestring<vm::PointLL>;
using ring_bg_t = std::vector<vm::PointLL>;

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
  return new_ring;
}

double GetRingLength(const ring_bg_t& ring) {
  // bg doesn't (yet) support length of ring geoms
  line_bg_t line{ring.begin(), ring.end()};
  auto length = bg::length(line, Haversine());
  return length;
}
} // namespace

namespace valhalla {
namespace loki {

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
    rings_length += GetRingLength(ring_bg);
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
          intersects = bg::intersects(rings_bg[ring_loc], edge_info.shape());
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

  return avoid_edge_ids;
}
} // namespace loki
} // namespace valhalla
