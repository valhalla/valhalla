#include "baldr/json.h"
#include "loki/polygon_search.h"
#include "midgard/boost_geom_types.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "valhalla/worker.h"

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/perimeter.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/spherical/distance_haversine.hpp>

#include <optional>
#include <queue>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

uint64_t to_value(uint32_t tileid, unsigned short bin) {
  return static_cast<uint64_t>(tileid) | (static_cast<uint64_t>(bin) << 32);
}

// map of tile for map of bin ids & their ring ids
using bins_collector_t =
    std::unordered_map<uint32_t, std::unordered_map<unsigned short, std::vector<size_t>>>;
static const auto Haversine = [] {
  return boost::geometry::strategy::distance::haversine<float>(kRadEarthMeters);
};

void correct_ring(bg::ring_ll_t& ring) {
  // close open rings
  bool is_open =
      (ring.begin()->lat() != ring.rbegin()->lat() || ring.begin()->lng() != ring.rbegin()->lng());
  if (!ring.empty() && is_open) {
    ring.push_back(ring[0]);
  }

  // reverse ring if counter-clockwise
  if (polygon_area(ring) > 0) {
    std::reverse(ring.begin(), ring.end());
  }
}

std::pair<bg::ring_ll_t, AABB2<PointLL>> PBFToRing(const valhalla::Ring& ring_pbf) {
  bg::ring_ll_t new_ring;
  new_ring.reserve(ring_pbf.coords().size());
  for (const auto& coord : ring_pbf.coords()) {
    new_ring.push_back({coord.lng(), coord.lat()});
  }
  correct_ring(new_ring);
  return {new_ring, AABB2<PointLL>(new_ring)};
}

#ifdef LOGGING_LEVEL_TRACE
// serializes an edge to geojson
std::string to_geojson(const std::unordered_set<GraphId>& edge_ids, GraphReader& reader) {
  auto features = json::array({});
  for (const auto& edge_id : edge_ids) {
    auto tile = reader.GetGraphTile(edge_id);
    auto edge = tile->directededge(edge_id);
    auto shape = tile->edgeinfo(edge).shape();
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }

    auto coords = json::array({});
    for (const auto& p : shape) {
      coords->emplace_back(json::array({json::fixed_t{p.lng(), 6}, json::fixed_t{p.lat(), 6}}));
    }
    features->emplace_back(json::map(
        {{"type", std::string("Feature")},
         {"properties",
          json::map({{"shortcut", edge->is_shortcut() ? std::string("True") : std::string("False")},
                     {"edge_id", edge_id.value}})},
         {"geometry", json::map({{"type", std::string("LineString")}, {"coordinates", coords}})}}));
  }

  auto collection = json::map({{"type", std::string("FeatureCollection")}, {"features", features}});

  std::stringstream ss;
  ss << *collection;

  return ss.str();
}

#endif // LOGGING_LEVEL_TRACE
} // namespace

namespace valhalla {
namespace loki {

std::unordered_set<GraphId> edges_in_rings(const Options& options,
                                           baldr::GraphReader& reader,
                                           const sif::cost_ptr_t& costing,
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
  std::vector<std::pair<bg::ring_ll_t, AABB2<PointLL>>> rings_bg;
  rings_bg.reserve(rings_pbf.size());
  for (const auto& ring_pbf : rings_pbf) {
    rings_length +=
        boost::geometry::perimeter(rings_bg.emplace_back(PBFToRing(ring_pbf)).first, Haversine());
  }
  if (rings_length > max_length) {
    throw valhalla_exception_t(167, std::to_string(static_cast<size_t>(max_length)) + " meters");
  }

  // Get the lowest level and tiles
  const auto tiles = TileHierarchy::levels().back().tiles;
  const auto bin_level = TileHierarchy::levels().back().level;

  // keep track which tile's bins intersect which rings
  std::unordered_set<GraphId> avoid_edge_ids;
  bins_collector_t contained_bins;
  [[maybe_unused]] uint32_t contained_bin_count = 0;
  contained_bins.reserve(200); // TODO: approximate based on polygon size?
  bins_collector_t bins_intersected;

  // first pull out all *unique* bins which intersect the boundaries
  for (size_t ring_idx = 0; ring_idx < rings_bg.size(); ring_idx++) {
    const auto& ring = rings_bg[ring_idx];
    auto line_intersection = tiles.Intersect(ring.first);
    for (const auto& tb : line_intersection) {
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

    for (const auto& [tile_id, bin_map] : line_intersection) {
      if (start_bin)
        break;
      for (const auto& bin_id : bin_map) {
        if (start_bin)
          break;
        for (const auto& neighbor : tiles.GetNeighboringBins<false>(tile_id, bin_id)) {
          // skip neighbors that are themselves on the boundary
          if (auto it = line_intersection.find(neighbor.first);
              it != line_intersection.end() && it->second.find(neighbor.second) != it->second.end()) {
            continue;
          }
          auto bbox = tiles.BinBBox(neighbor.first, neighbor.second);
          PointLL center((bbox.minx() + bbox.maxx()) * 0.5, (bbox.miny() + bbox.maxy()) * 0.5);
          if (point_in_poly(center, ring.first)) {
            start_bin = neighbor;
            break;
          }
        }
      }
    }

    if (start_bin) {
      std::queue<std::pair<uint32_t, unsigned short>> bin_queue;
      bin_queue.push(*start_bin);
      processed_bins.insert(to_value(start_bin->first, start_bin->second));
      size_t n = 0;
      while (!bin_queue.empty() && n++ < 1'000'000) {
        // keep looking for contained bins until we run out of bins to look at
        auto bin = bin_queue.front();
        bin_queue.pop();
        // settle this bin and add its neighbors
        contained_bins[bin.first][bin.second].push_back(ring_idx);
        contained_bin_count++;
        // skip diagonal neighbors, since the Bresenham method used for finding the intersecting bins
        // does not move diagonally either
        for (const auto& neighbor : tiles.GetNeighboringBins<true>(bin.first, bin.second)) {
          if (processed_bins.count(to_value(neighbor.first, neighbor.second)) > 0) {
            continue; // we have already looked at this bin
          }

          processed_bins.insert(to_value(neighbor.first, neighbor.second));
          if (auto it = line_intersection.find(neighbor.first);
              it != line_intersection.end() && it->second.find(neighbor.second) != it->second.end()) {
            continue; // this one intersects the boundary
          }
          bin_queue.push(neighbor);
        }
      }
    }
  }

  auto check_bins = [&](uint32_t tileid, const std::pair<unsigned short, std::vector<size_t>>& bin,
                        bool intersect) {
    auto tile = reader.GetGraphTile({tileid, bin_level, 0});
    if (!tile) {
      return;
    }

    bool has_bounding_circles = tile->header()->has_bounding_circles();
    auto edges = tile->GetBin(bin.first);
    auto bounding_circles = tile->GetBoundingCircles(bin.first);
    auto bounding_circle = bounding_circles.begin();
    auto minx = tiles.TileBounds(tileid).minx();
    auto miny = tiles.TileBounds(tileid).miny();
    for (auto edge_it = edges.begin(); edge_it != edges.end(); ++edge_it, ++bounding_circle) {
      auto edge_id = *edge_it;
      if (avoid_edge_ids.count(edge_id) != 0) {
        continue;
      }
      double radius = 0, radius_deg, radius_sq;
      std::pair<PointLL, double> circle({0, 0}, 0.);
      // if we need to check each bin edge individually, prepare its bounding circle
      // if available
      if (intersect) {
        auto lat_offset =
            (bin.first / kBinsDim) * tiles.SubdivisionSize() + tiles.SubdivisionSize() / 2;
        auto lng_offset =
            (bin.first % kBinsDim) * tiles.SubdivisionSize() + tiles.SubdivisionSize() / 2;
        PointLL bin_center(minx + lng_offset, miny + lat_offset);
        auto bin_center_approximator = DistanceApproximator<PointLL>(bin_center);
        if (has_bounding_circles && bounding_circle->is_valid())
          circle = bounding_circle->get(bin_center_approximator, bin_center);
        radius = circle.second;
        radius_deg = radius / (kMetersPerDegreeLat * cosf(circle.first.lat() * kRadPerDeg));
        radius_sq = square(radius);
      }

      if (edge_id.tile_base() != tile->header()->graphid().tile_base() &&
          !reader.GetGraphTile(edge_id, tile)) {
        continue;
      }

      const DirectedEdge* edge;
      edge = tile->directededge(edge_id);
      auto opp_tile = tile;
      const baldr::DirectedEdge* opp_edge = nullptr;
      baldr::GraphId opp_id;

      // bail if we wouldnt be allowed on this edge anyway, nor its opposing
      if (!costing->Allowed(edge, tile) &&
          (!(opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)).is_valid() ||
           !costing->Allowed(opp_edge, opp_tile))) {
        continue;
      }

      bool exclude = false;
      std::optional<EdgeInfo> edgeinfo;
      for (const auto& ring_loc : bin.second) {
        // if intersect is false, the bin is entirely within the ring,
        // so the shape is known to intersect the ring
        bool intersects = !intersect;

        auto& ring = rings_bg[ring_loc];

        if (intersect &&
            (radius == 0 ||
             (radius != 0 && circle_intersects_bounds(circle.first, radius_deg, ring.second) !=
                                 CircleInBbox::OUTSIDE))) {

          // avoid boost::geometry::intersects(...) at all cost; if the circle center
          // is inside the polygon and the distance from the circle center
          // to each segment is larger than its radius, the shape must be fully inside
          // the polygon

          // if we have a bounding circle, and the center of that circle is inside the polygon
          if (radius != 0. && point_in_poly(circle.first, ring.first)) {
            bool segment_within_radius = false;
            // circle intersects the ring bbox, and the center is inside the polygon
            for (size_t i = 0; i < ring.first.size() - 1; ++i) {
              const auto& pt = ring.first[i];
              const auto& next_pt = ring.first[i + 1];
              auto projected = circle.first.Project(pt, next_pt);
              // if the circle collides with any ring segment,
              // we need to check the shape
              if (circle.first.DistanceSquared(projected) <= radius_sq) {
                // check_shape = true;
                segment_within_radius = true;
                break;
              }
            }

            if (!segment_within_radius) {
              intersects = true;
            }
          }

          if (!intersects) {
            // either we have no circle or we do but it's not completely outside of the ring bbox, so
            // we have to check the shape

            if (!edgeinfo)
              edgeinfo = tile->edgeinfo(edge);
            intersects =
                boost::geometry::intersects(ring.first, bg::linestring_ll_t(edgeinfo->shape().begin(),
                                                                            edgeinfo->shape().end()));
          }
        }

        // if the edge shape intersects the ring, check if the user passed
        // levels
        if (intersects) {
          if (exclude_levels[ring_loc].size() > 0) {
            // the user passed levels, so only exclude the edges if they run on
            // (not across) that level
            if (!edgeinfo)
              edgeinfo = tile->edgeinfo(edge);
            const auto& levels = edgeinfo->levels();
            // the edge runs on this level (i.e. does not merely traverse it)
            if (levels.first.size() == 1 && levels.first[0].first == levels.first[0].second &&
                exclude_levels[ring_loc].find(levels.first[0].first) !=
                    exclude_levels[ring_loc].end()) {
              exclude = true;
            }
          } else {
            exclude = true;
          }
          if (intersects && exclude)
            break;
        }
      }
      if (exclude) {
        avoid_edge_ids.emplace(edge_id);
        avoid_edge_ids.emplace(
            opp_id.is_valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
      }
    }
  };

  for (const auto& [tileid, bins] : contained_bins) {
    for (const auto& bin : bins) {
      check_bins(tileid, bin, false);
    }
  }

  LOG_DEBUG("Marked " + std::to_string(contained_bin_count) + " bins as fully contained by a ring");
  for (const auto& [tileid, bins] : bins_intersected) {
    for (const auto& bin : bins) {
      check_bins(tileid, bin, true);
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
