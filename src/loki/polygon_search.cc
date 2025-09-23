#include "baldr/json.h"
#include "loki/polygon_search.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "valhalla/worker.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <optional>

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;

BOOST_GEOMETRY_REGISTER_POINT_2D(vm::PointLL, double, bg::cs::geographic<bg::degree>, first, second)
BOOST_GEOMETRY_REGISTER_RING(std::vector<vm::PointLL>)

namespace {

uint64_t to_value(uint32_t tileid, unsigned short bin) {
  return static_cast<uint64_t>(tileid) | (static_cast<uint64_t>(bin) << 32);
}

// register a few boost.geometry types
using line_bg_t = bg::model::linestring<vm::PointLL>;
using ring_bg_t = std::vector<vm::PointLL>;
using namespace vb::json;

// map of tile for map of bin ids & their ring ids
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

void to_geojson(AABB2<PointLL>& bbox, rapidjson::writer_wrapper_t& writer, bool start_bin = false) {
  writer.start_object();
  writer("type", "Feature");
  writer.start_object("properties");
  writer("start", start_bin);
  writer.end_object();
  writer.start_object("geometry");
  writer("type", "Polygon");
  writer.start_array("coordinates");
  writer.start_array();

  // 1
  writer.start_array();
  writer(bbox.minx());
  writer(bbox.miny());
  writer.end_array();

  // 2
  writer.start_array();
  writer(bbox.minx());
  writer(bbox.maxy());
  writer.end_array();

  // 3
  writer.start_array();
  writer(bbox.maxx());
  writer(bbox.maxy());
  writer.end_array();

  // 4
  writer.start_array();
  writer(bbox.maxx());
  writer(bbox.miny());
  writer.end_array();

  // 5
  writer.start_array();
  writer(bbox.minx());
  writer(bbox.miny());
  writer.end_array();

  writer.end_array();
  writer.end_array();  // coordinates
  writer.end_object(); // geom
  writer.end_object(); // feat
}
#endif // LOGGING_LEVEL_TRACE
} // namespace

namespace valhalla {
namespace loki {

std::unordered_set<vb::GraphId> edges_in_rings(const Options& options,
                                               baldr::GraphReader& reader,
                                               const std::shared_ptr<sif::DynamicCost>& costing,
                                               float max_length) {
  // protect for bogus input
  const auto& rings_pbf = options.exclude_polygons();
  if (rings_pbf.empty() || rings_pbf.Get(0).coords().empty() ||
      !rings_pbf.Get(0).coords()[0].has_lat_case() || !rings_pbf.Get(0).coords()[0].has_lng_case()) {
    return {};
  }

  // keep a lookup for the excluded levels per ring/feature
  std::vector<std::unordered_set<float>> exclude_levels(options.exclude_polygons_size());
  // can't have more exclude levels than exclude polygons
  assert(options.exclude_levels_size() <= options.exclude_polygons_size());

  for (size_t i = 0; i < static_cast<size_t>(options.exclude_levels_size()); ++i) {
    for (size_t j = 0; j < static_cast<size_t>(options.exclude_levels().at(i).levels_size()); ++j) {
      exclude_levels[i].insert(options.exclude_levels().at(i).levels().at(j));
    }
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
  std::unordered_set<vb::GraphId> avoid_edge_ids;
  std::vector<std::pair<uint32_t, unsigned short>> contained_bins;
  contained_bins.reserve(200); // TODO: approximate based on polygon size?
  bins_collector bins_intersected;

  // first pull out all *unique* bins which intersect the boundaries
  for (size_t ring_idx = 0; ring_idx < rings_bg.size(); ring_idx++) {
    const auto& ring = rings_bg[ring_idx];
    auto line_intersected = tiles.Intersect(ring);
    for (const auto& tb : line_intersected) {
      for (const auto& b : tb.second) {
        bins_intersected[static_cast<uint32_t>(tb.first)][b].push_back(ring_idx);
      }
    }
    // do a flood fill to find all bins completely inside any of this ring:
    // first, find a bin that is fully contained in the ring (i.e. one of its border coordinates is
    // inside the ring and it's not in the set of line intersected bins)

    // note: a caveat of this is that this will only fill the
    // first area it finds, so very large and oddly concave
    // polygons might not work
    std::optional<std::pair<uint32_t, unsigned short>> start_bin;

    std::unordered_set<uint64_t> processed_bins;

    bool found = false;
    for (const auto& [tile_id, bin_map] : bins_intersected) {
      if (found)
        break;
      for (const auto& [bin_id, _] : bin_map) {
        if (found)
          break;
        // go through all of them, go through its neighbors
        // and if we find an intersecting neighbor, check if it's
        // already in the intersected bins

        auto bbox = tiles.BinBBox(tile_id, bin_id);
        std::array<PointLL, 4> corners = {
            PointLL(bbox.minx(), bbox.miny()),
            PointLL(bbox.minx(), bbox.maxy()),
            PointLL(bbox.maxx(), bbox.maxy()),
            PointLL(bbox.maxx(), bbox.miny()),
        };
        auto neighbors = tiles.GetNeighbors(tile_id, bin_id);

        for (uint8_t corner_idx = 0; corner_idx < 4; corner_idx++) {
          if (found)
            break;
          const auto& corner = corners[corner_idx];
          // corner is outside of the ring
          if (!bg::within(corner, ring))
            continue;

          // look for neighboring bins that share this corner
          for (const auto& neighbor : neighbors) {
            if (found)
              break;

            auto neighbor_bbox = tiles.BinBBox(neighbor.first, neighbor.second);
            std::array<PointLL, 4> neighbor_corners = {
                PointLL(neighbor_bbox.minx(), neighbor_bbox.miny()),
                PointLL(neighbor_bbox.minx(), neighbor_bbox.maxy()),
                PointLL(neighbor_bbox.maxx(), neighbor_bbox.maxy()),
                PointLL(neighbor_bbox.maxx(), neighbor_bbox.miny()),
            };

            for (const auto& neighbor_corner : neighbor_corners) {
              if (neighbor_corner == corner) {
                // we've found a neighbor that shares the corner
                if (auto it = bins_intersected.find(neighbor.first);
                    it != bins_intersected.end() &&
                    it->second.find(neighbor.second) != it->second.end()) {
                  // but this neighbor is not fully contained in the ring
                  continue;
                }

                start_bin = neighbor;
                found = true;
                break;
              }
            }
          }
        }
      }
    }

    if (start_bin) {
      std::queue<std::pair<uint32_t, unsigned short>> bin_queue;
      bin_queue.push(*start_bin);
      size_t n = 0;

      // don't run forever
      while (!bin_queue.empty() && n++ < 1e6) {
        // keep looking for contained bins until we run out of bins to look at
        auto bin = bin_queue.front();
        bin_queue.pop();
        // settle this bin and add its neighbors
        contained_bins.push_back(bin);

        for (const auto& neighbor : tiles.GetNeighbors(bin.first, bin.second)) {
          auto neighbor_bbox = tiles.BinBBox(neighbor.first, neighbor.second);
          if (processed_bins.count(to_value(neighbor.first, neighbor.second)) > 0) {
            continue; // we have already looked at this bin
          }

          if (auto it = bins_intersected.find(neighbor.first);
              it != bins_intersected.end() && it->second.find(neighbor.second) != it->second.end()) {
            continue; // this one intersects the boundary
          }
          bin_queue.push(neighbor);
          processed_bins.insert(to_value(neighbor.first, neighbor.second));
        }
      }
    }
  }

  for (const auto& bin : contained_bins) {
    auto tile = reader.GetGraphTile({bin.first, bin_level, 0});
    if (!tile) {
      continue;
    }
    for (const auto& edge_id : tile->GetBin(bin.second)) {
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

      avoid_edge_ids.emplace(edge_id);
      avoid_edge_ids.emplace(
          opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
    }
  }

  LOG_DEBUG("Marked " + std::to_string(contained_bins.size()) + " bins as fully contained by a ring");
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
        bool include = false;
        for (const auto& ring_loc : bin.second) {
          bool intersects = bg::intersects(rings_bg[ring_loc], line_bg_t(edge_info.shape().begin(),
                                                                         edge_info.shape().end()));
          if (intersects) {
            // if the edge shape intersects the ring, check if the user passed
            // levels
            if (exclude_levels[ring_loc].size() > 0) {
              // the user passed levels, so only exclude the edges if they run on
              // (not across) that level
              const auto& levels = edge_info.levels();
              // the edge runs on this level (i.e. does not merely traverse it)
              if (levels.first.size() == 1 && levels.first[0].first == levels.first[0].second) {
                if (exclude_levels[ring_loc].find(levels.first[0].first) !=
                    exclude_levels[ring_loc].end()) {
                  include = true;
                }
              }
            } else {
              include = true;
            }
            if (intersects && include)
              break;
          }
        }
        if (include) {
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
