#include "baldr/json.h"
#include "loki/polygon_search.h"
#include "midgard/aabb2.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "valhalla/worker.h"

#include <flatbush.h>

#include <optional>
#include <queue>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

bool circle_outside_bounds(const PointLL& center,
                           float radius_sq,
                           const AABB2<valhalla::midgard::PointLL>& box) {

  PointLL closest{std::clamp(center.x(), box.minx(), box.maxx()),
                  std::clamp(center.y(), box.miny(), box.maxy())};

  return center.DistanceSquared(closest) > radius_sq;
}
/**
 * Tests whether a point lies inside a linear ring
 *
 * Uses a ray-casting algorithm supported by an rtree on the ring segments
 * to only consider segments that intersect with the test point's x axis.
 *
 */
bool point_in_ring(const PointLL& pt,
                   const std::vector<PointLL>& ring,
                   const flatbush::Flatbush<double>& rtree) {
  // cast a ray rightward from pt, count crossings
  int crossings = 0;

  auto candidates =
      rtree.search(flatbush::Box{pt.lng(), pt.lat(), std::numeric_limits<double>::max(), pt.lat()});

  for (auto idx : candidates) {
    const PointLL& a = ring[idx];
    const PointLL& b = ring[idx + 1];

    // skip horizontal segments
    if (a.lat() == b.y())
      continue;

    // check the ray at pt.y crosses the segment's y-span (half-open to avoid double counting at
    // vertices)
    if (!((a.lat() <= pt.lat() && b.lat() > pt.lat()) || (b.lat() <= pt.lat() && a.lat() > pt.lat())))
      continue;

    // x coordinate of the intersection of segment ab with the horizontal ray
    double x_intersect = a.lng() + (pt.lat() - a.lat()) * (b.lng() - a.lng()) / (b.lat() - a.lat());

    if (pt.lng() < x_intersect)
      ++crossings;
  }

  return crossings % 2 == 1;
}

flatbush::Box<double> box_from_shape(std::span<const PointLL> shape) {
  auto [minx, maxx] = std::ranges::minmax(shape, {}, &PointLL::x);
  auto [miny, maxy] = std::ranges::minmax(shape, {}, &PointLL::y);
  return {minx.x(), miny.y(), maxx.x(), maxy.y()};
}

CircleInBbox circle_intersects_ring(const PointLL& center,
                                    double radius,
                                    const std::vector<PointLL>& ring,
                                    const flatbush::Flatbush<double>& rtree) {

  auto candidates = rtree.neighbors(flatbush::Point<double>{center.x(), center.y()},
                                    std::numeric_limits<size_t>::max(), radius);

  for (const auto idx : candidates) {
    auto project = center.Project(ring[idx], ring[idx + 1]);
    if (center.Distance(project) <= radius) {
      return CircleInBbox::INTERSECTS;
    }
  }

  auto pip = point_in_ring(center, ring, rtree);
  return pip ? CircleInBbox::INSIDE : CircleInBbox::OUTSIDE;
}
/**
 * Test whether a line intersects a linear ring.
 */
bool line_intersects_ring(const std::vector<PointLL>& shape,
                          const std::vector<PointLL>& ring,
                          const flatbush::Flatbush<double>& rtree) {

  // go through the shape's segments
  for (size_t i = 0; i < shape.size() - 1; ++i) {
    LineSegment2<PointLL> seg(shape[i], shape[i + 1]);
    auto box = box_from_shape(std::span(shape).subspan(i, 2));

    // look for ring segments with intersecting bounding boxes
    auto candidates = rtree.search(box);
    for (const auto idx : candidates) {
      LineSegment2<PointLL> ring_seg(ring[idx], ring[idx + 1]);
      PointLL intersect;

      // if any ring segment intersects, we're done
      if (seg.Intersect(ring_seg, intersect)) {
        return true;
      }
    }
  }

  // no single ring segment intersected with any shape segment; the shape could still be fully
  // inside the ring though
  return point_in_ring(*shape.begin(), ring, rtree);
}

uint64_t to_value(uint32_t tileid, unsigned short bin) {
  return static_cast<uint64_t>(tileid) | (static_cast<uint64_t>(bin) << 32);
}

// map of tile for map of bin ids & their ring ids
using bins_collector_t =
    std::unordered_map<uint32_t, std::unordered_map<unsigned short, std::vector<size_t>>>;

void correct_ring(std::vector<PointLL>& ring) {
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

std::pair<std::vector<PointLL>, AABB2<PointLL>> PBFToRing(const valhalla::Ring& ring_pbf) {
  std::vector<PointLL> new_ring;
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

  // convert to vector and check length restriction
  double rings_length = 0;
  std::vector<std::pair<std::vector<PointLL>, AABB2<PointLL>>> rings;
  rings.reserve(rings_pbf.size());
  for (const auto& ring_pbf : rings_pbf) {
    const auto& ring = rings.emplace_back(PBFToRing(ring_pbf));
    for (size_t i = 0; i < ring.first.size() - 1; ++i) {
      rings_length += ring.first[i].Distance(ring.first[i + 1]);
    }
  }
  if (rings_length > max_length) {
    throw valhalla_exception_t(167, std::to_string(static_cast<size_t>(max_length)) + " meters");
  }

  std::vector<flatbush::Flatbush<double>> rtrees;
  rtrees.reserve(rings.size());
  for (const auto& ring : rings) {
    flatbush::FlatbushBuilder<double> builder;
    for (size_t i = 0; i < ring.first.size() - 1; ++i) {
      const auto& pt = ring.first[i];
      const auto& next_pt = ring.first[i + 1];
      std::vector<PointLL> pts{pt, next_pt};
      AABB2<PointLL> bb(pts);
      flatbush::Box<double> box = box_from_shape(std::span(ring.first).subspan(i, 2));
      builder.add(box);
    }
    rtrees.push_back(builder.finish());
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
  for (size_t ring_idx = 0; ring_idx < rings.size(); ring_idx++) {
    const auto& ring = rings[ring_idx];
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
        radius_sq = midgard::sqr(radius);
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
        const auto& ring = rings[ring_loc];
        const auto& rtree = rtrees[ring_loc];

        // if we need to check edges individually, and there is no bounding circle, or there is one
        // and it intersects with the ring's bounding box, we need to have a closer look
        if (intersect &&
            (radius == 0 ||
             (radius != 0 && !circle_outside_bounds(circle.first, radius_sq, ring.second)))) {
          if (!intersects) {
            // either we have no circle or we do but it intersected the ring boundary
            if (!edgeinfo)
              edgeinfo = tile->edgeinfo(edge);

            intersects = line_intersects_ring(edgeinfo->shape(), ring.first, rtree);
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
              break;
            }
          } else {
            exclude = true;
            break;
          }
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
