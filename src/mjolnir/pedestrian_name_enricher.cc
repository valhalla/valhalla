#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/pedestrian_name_enricher.h"

#include <boost/geometry.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <optional>
#include <thread>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

// Spacing (in meters) used to resample named road edges into points before indexing
// them in the R-tree. A long road is thus represented by many points along its whole
// length instead of a single midpoint, so a nearby pedestrian edge always finds it.
constexpr double kSampleStepMeters = 10.0;

// Number of nearest sample points to retrieve from the R-tree. Since each named road
// maps to several points (see kSampleStepMeters), we query more points
// and then deduplicate to obtain up to distinct edges.
constexpr unsigned int kMaxSamplePointsToTest = 100;

// Maximum search distance in meters between a named road and a sidewalk
constexpr unsigned long kMaxEnrichDistance = 50;

// R-tree types for spatial indexing of named edges
// We use cartesian coordinates instead geographical because it is clearly faster.
// To approximate coordinates, we just multiply the longitudes by a tile-constant cos(lat).
// Moreover, we use float in the tree (not double) because it increases search and build speed.
using rtree_point_t = boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>;
using rtree_value_t = std::pair<rtree_point_t, uint32_t>; // point + candidate_idx
using rtree_t = boost::geometry::index::rtree<rtree_value_t, boost::geometry::index::rstar<16>>;

// Returns true if the edge `use` is a road-specific type
bool IsRoadUse(Use use) {
  switch (use) {
    case Use::kRoad:
    case Use::kRamp:
    case Use::kTurnChannel:
    case Use::kTrack:
    case Use::kDriveway:
    case Use::kAlley:
    case Use::kParkingAisle:
    case Use::kEmergencyAccess:
    case Use::kDriveThru:
    case Use::kCuldesac:
    case Use::kLivingStreet:
    case Use::kServiceRoad:
      return true;
    default:
      return false;
  }
}

// Stores the name and full geometry needed to score one road candidate.
struct NamedEdgeCandidate {
  std::string name;
  std::vector<PointLL> shape;
};

// R-tree struct with associated candidates
struct NamedEdgesTree {
  rtree_t rtree;
  std::vector<NamedEdgeCandidate> candidates;
};

// Returns the midpoint along the polyline (at 50% of cumulative distance).
// Falls back to the geometric middle vertex if trim_polyline fails.
PointLL PolylineMidpoint(const std::vector<PointLL>& shape) {
  assert(!shape.empty());

  auto midpoints = valhalla::midgard::trim_polyline(shape.begin(), shape.end(), 0.5, 0.5);
  if (midpoints.empty()) { // should not appear
    LOG_ERROR("Bug trim_polyline: midpoint of an non-empty edge not found");
    return PointLL(shape[shape.size() / 2]);
  }

  return PointLL(midpoints.front());
}

// Builds an R-tree of named road candidates near the target pedestrian edges.
// Loads existing tiles intersecting candidate bounding box across all hierarchy levels,
// rejects road shapes outside candidate bounding box, and indexes only samples inside
// sample bounding box. Candidate names/shapes are stored for later scoring.
//
// Known limitation: candidate tiles are selected spatially from sample bounding box. A long edge
// stored in a more distant origin tile may therefore be missed even if its shape later crosses
// candidate bounding box.
NamedEdgesTree BuildNamedEdgesTree(GraphReader& reader,
                                   const AABB2<PointLL>& candidate_bb,
                                   const AABB2<PointLL>& sample_bb,
                                   float cos_lat) {
  NamedEdgesTree tree;

  const auto candidate_tile_ids = TileHierarchy::GetGraphIds(sample_bb);

  [[maybe_unused]] const size_t candidate_tiles_found = candidate_tile_ids.size();
  [[maybe_unused]] size_t candidate_tiles_loaded = 0;
  [[maybe_unused]] size_t candidate_edges_examined = 0;
  [[maybe_unused]] size_t shapes_intersecting_candidate_bb = 0;
  [[maybe_unused]] size_t generated_samples = 0;
  [[maybe_unused]] size_t retained_samples = 0;
  [[maybe_unused]] size_t candidates_without_retained_sample = 0;

  std::vector<rtree_value_t> named_edge_values;

  for (const auto& candidate_tile_id : candidate_tile_ids) {
    // skip hierarchy tiles that do not exist at the current build stage
    if (!reader.DoesTileExist(candidate_tile_id))
      continue;

    auto candidate_tile = reader.GetGraphTile(candidate_tile_id);
    if (!candidate_tile)
      continue;

    candidate_tiles_loaded++;

    const uint32_t edge_count = candidate_tile->header()->directededgecount();

    for (uint32_t candidate_edge_idx = 0; candidate_edge_idx < edge_count; ++candidate_edge_idx) {
      candidate_edges_examined++;

      const DirectedEdge* candidate_directed_edge = candidate_tile->directededge(candidate_edge_idx);

      // skip shortcut edges
      if (candidate_directed_edge->is_shortcut())
        continue;

      // only process road edges
      if (!IsRoadUse(candidate_directed_edge->use()))
        continue;

      EdgeInfo candidate_edgeinfo = candidate_tile->edgeinfo(candidate_directed_edge);

      // skip unnamed edges
      auto names = candidate_edgeinfo.GetNames();
      if (names.empty())
        continue;

      // skip empty shapes
      const auto& candidate_shape = candidate_edgeinfo.shape();
      if (candidate_shape.empty())
        continue;

      // skip road shapes that cannot intersect the maximum candidate area
      AABB2<PointLL> shape_bb(candidate_shape);
      if (!shape_bb.Intersects(candidate_bb))
        continue;

      shapes_intersecting_candidate_bb++;

      // resample the road and index only points close enough to the target pedestrian edges;
      // each retained sample points back to the compact candidate index
      const uint32_t candidate_idx = static_cast<uint32_t>(tree.candidates.size());
      const size_t values_before = named_edge_values.size();

      const auto sampled_points =
          resample_spherical_polyline(candidate_shape, kSampleStepMeters, true);

      generated_samples += sampled_points.size();

      for (const auto& sample : sampled_points) {
        if (!sample_bb.Contains(sample))
          continue;

        ++retained_samples;
        named_edge_values.emplace_back(rtree_point_t(sample.lng() * cos_lat, sample.lat()),
                                       candidate_idx);
      }

      // discard candidates with no sampled point inside the buffered sample area
      if (named_edge_values.size() == values_before) {
        ++candidates_without_retained_sample;
        continue;
      }

      tree.candidates.push_back({
          names.front(),
          candidate_shape,
      });
    }
  }

  LOG_DEBUG("Named-road R-Tree: tiles_found={}, tiles_loaded={}, edges_examined={}, "
            "shapes_in_bb={}, generated_samples={}, retained_samples={}, "
            "candidates_without_sample={}, indexed_candidates={}",
            candidate_tiles_found, candidate_tiles_loaded, candidate_edges_examined,
            shapes_intersecting_candidate_bb, generated_samples, retained_samples,
            candidates_without_retained_sample, tree.candidates.size());

  // bulk-load the R-tree (faster than individual inserts in the for loop)
  tree.rtree = rtree_t(named_edge_values.begin(), named_edge_values.end());
  return tree;
}

// Computes a length-weighted average distance (in meters) from polyline 'from' to a set
// of polylines 'tos'. Each vertex of 'from' is projected onto the NEAREST of all 'tos',
// and every segment contributes the mean of its two endpoint distances, weighted by its
// share of the total length.
//
// Comparing 'from' against several polylines at once matters because Valhalla splits a
// street into several named road edges sharing one name: grouping them here lets a long
// pedestrian edge match the whole street instead of being rejected because no single road
// edge spans its full length.
//
// NB: This is a non-symmetrical "distance".
//
// Example: pedestrian polyline 'from' = [A, B, C], scored against a single road in 'tos'
//
//                            C            A--B is long, B--C is short.
//                           /|            a, b, c = closest distances from
//                          / |              each vertex down to the nearest 'to'.
//   'from' =  A-----------B  |
//             |           |  |
//             a           b  c
//             |           |  |
//   'to' = ---+-----------+--+---------   (the named road, roughly parallel)
//
//   L_ab = length(A,B) = 30 m
//   L_bc = length(B,C) = 10 m
//   L = 40 m (total length)
//   Each segment i contributes the mean of its endpoint distances, weighted by L_i/L:
//
//     score = (L_ab/L) * (a + b)/2  +  (L_bc/L) * (b + c)/2
//           = 0.75   * (a + b)/2  +  0.25   * (b + c)/2
//
//   So the long A--B segment dominates the score; a lone far-away vertex on a
//   tiny segment barely moves it (unlike a Hausdorff / max-based distance).
float AverageDistanceToPolylines(const std::vector<PointLL>& from,
                                 const std::vector<const std::vector<PointLL>*>& tos) {
  if (from.size() < 2 || tos.empty())
    return std::numeric_limits<float>::max();

  // per vertex of 'from', distance to the nearest of all the street's pieces
  std::vector<float> vertex_dist(from.size(), std::numeric_limits<float>::max());
  for (size_t i = 0; i < from.size(); ++i) {
    for (const auto* to : tos) {
      if (to->empty())
        continue;
      vertex_dist[i] =
          std::min(vertex_dist[i], static_cast<float>(std::get<1>(from[i].ClosestPoint(*to))));
    }
  }

  float weighted_sum = 0.f;
  float total_len = 0.f;
  for (size_t i = 0; i + 1 < from.size(); ++i) {
    float seg_len = from[i].Distance(from[i + 1]);
    weighted_sum += seg_len * 0.5f * (vertex_dist[i] + vertex_dist[i + 1]);
    total_len += seg_len;
  }

  if (total_len <= 0.f)
    return std::numeric_limits<float>::max();

  return weighted_sum / total_len;
}

/// Finds the best matching street name for a pedestrian edge by querying the nearest
// sampled points in the R-tree. Candidate road edges are deduplicated and grouped by
// name (a street is often split into several edges), then each street is scored against
// the pedestrian edge with the length-weighted polyline distance; the closest one wins.
std::string FindNearestName(const NamedEdgesTree& tree,
                            const std::vector<PointLL>& pedestrian_shape,
                            std::vector<rtree_value_t>& results,
                            const float cos_lat) {
  PointLL center = PolylineMidpoint(pedestrian_shape);
  results.clear(); // clear content but memory is kept

  // resample the pedestrian edge so long straight segments are sampled too, giving a
  // more accurate polyline distance (the metric only projects vertices of 'from')
  auto sampled_pedestrian =
      valhalla::midgard::resample_spherical_polyline(pedestrian_shape, kSampleStepMeters, true);

  // find the nearest candidate sample points in the rtree
  tree.rtree.query(boost::geometry::index::nearest(rtree_point_t(center.lng() * cos_lat,
                                                                 center.lat()),
                                                   kMaxSamplePointsToTest),
                   std::back_inserter(results));

  // group distinct candidates by street name; several samples may map to the
  // same candidate, and several candidates may share the same street name
  std::set<uint32_t> tested_candidates;
  std::map<std::string, std::vector<const std::vector<PointLL>*>> shapes_by_name;
  for (const auto& [pt, candidate_idx] : results) {
    // already grouped this edge through another sample point
    if (tested_candidates.count(candidate_idx))
      continue;
    tested_candidates.insert(candidate_idx);

    const auto& candidate = tree.candidates[candidate_idx];
    shapes_by_name[candidate.name].push_back(&candidate.shape);
  }

  // the street name with the best aggregated score wins
  float best_score = kMaxEnrichDistance;
  std::string best_name;
  for (const auto& [name, shapes] : shapes_by_name) {
    float score = AverageDistanceToPolylines(sampled_pedestrian, shapes);
    if (score < best_score) {
      best_score = score;
      best_name = name;
    }
  }

  return best_name; // can be empty if all tested streets are > kMaxEnrichDistance
}

// Unit pedestrian enrichment of an edge
// All enrichments are bulk-loaded in one pass at the end
struct Enrichment {
  uint32_t edge_num;
  std::string name;
};

// Returns true if the edge `use` is a pedestrian-specific type that should be enriched
bool IsPedestrianUseToEnrich(Use use) {
  switch (use) {
    case Use::kSidewalk:
      return true;
    default:
      return false;
  }
}

// Stores target pedestrian edges indices and their aggregate geographic bounding box.
struct PedestrianTargets {
  std::vector<uint32_t> edge_indices;
  std::optional<AABB2<PointLL>> bounding_box;
};

// Collects unnamed pedestrian edges to enrich and computes their aggregate bounding box.
PedestrianTargets CollectPedestrianTargets(const graph_tile_ptr& tile) {
  PedestrianTargets targets;

  for (uint32_t edge_idx = 0; edge_idx < tile->header()->directededgecount(); ++edge_idx) {
    const auto* edge = tile->directededge(edge_idx);

    // skip shortcuts and edge uses that are not eligible for enrichment
    if (edge->is_shortcut() || !IsPedestrianUseToEnrich(edge->use()))
      continue;

    auto edge_info = tile->edgeinfo(edge);

    // skip pedestrian edges that already have a name
    if (!edge_info.GetNames().empty())
      continue;

    // skip pedestrian edges without usable geometry
    const auto& shape = edge_info.shape();
    if (shape.empty())
      continue;

    targets.edge_indices.push_back(edge_idx);
    AABB2<PointLL> shape_bb(shape);
    if (targets.bounding_box) {
      // extend the aggregate bounding box with the current pedestrian shape
      targets.bounding_box->Expand(shape_bb);
    } else {
      // initialize the aggregate bounding box from the first pedestrian shape
      targets.bounding_box = shape_bb;
    }
  }

  return targets;
}

// Worker function that processes a subset of tiles
void EnrichWorker(const boost::property_tree::ptree& pt,
                  const std::vector<GraphId>& tileset,
                  const size_t thread_num,
                  std::atomic<size_t>& next_tile,
                  std::atomic<size_t>& total_enriched) {
  try {
    const std::string tile_dir = pt.get<std::string>("tile_dir");

    GraphReader reader(pt);

    size_t enriched_edges = 0;

    [[maybe_unused]] size_t nb_tiles_treated = 0;
    while (true) {
      // Each thread grabs the next available tile
      size_t tile_num = next_tile.fetch_add(1);
      if (tile_num >= tileset.size()) {
        break;
      }

      size_t enriched_edges_in_tile = 0;

      const GraphId tile_id = tileset[tile_num];
      GraphTileBuilder tilebuilder(tile_dir, tile_id, true, false);

      // Load the tile via GraphReader to access edge info
      graph_tile_ptr tile;
      tile = reader.GetGraphTile(tile_id);
      if (!tile) {
        continue;
      }

      // collect target pedestrian edges and their aggregate bounding box
      auto targets = CollectPedestrianTargets(tile);
      LOG_DEBUG("Target tile level={}, tileid={}: sidewalks_to_enrich={}",
                static_cast<unsigned int>(tile_id.level()), tile_id.tileid(),
                targets.edge_indices.size());
      if (targets.edge_indices.empty())
        continue;

      // expand by the maximum accepted distance to include nearby named-road candidates
      const auto candidate_bb = ExpandMeters(*targets.bounding_box, kMaxEnrichDistance);
      // expand by one sample step to retain samples around the candidate-area boundary
      const auto sample_bb = ExpandMeters(candidate_bb, kSampleStepMeters);

      // tile-constant cos_lat to approximate coord as cartesian coordinates,
      // using the center of the target tile
      auto bounds = TileHierarchy::levels().back().tiles.TileBounds(tile_id.tileid());
      const float cos_lat = std::cos((bounds.miny() + bounds.maxy()) * 0.5 * M_PI / 180.0);

      std::vector<Enrichment> enrichments;
      enrichments.reserve(targets.edge_indices.size());

      [[maybe_unused]] auto t_build_start = std::chrono::steady_clock::now();
      // build one candidate tree shared by all target pedestrian edges in this tile
      NamedEdgesTree tree = BuildNamedEdgesTree(reader, candidate_bb, sample_bb, cos_lat);
      [[maybe_unused]] auto t_build_end = std::chrono::steady_clock::now();

      std::vector<rtree_value_t> results;
      results.reserve(kMaxSamplePointsToTest);

      [[maybe_unused]] size_t queries_with_results = 0;
      [[maybe_unused]] size_t queries_without_name = 0;

      // find the best road name for each previously collected pedestrian edge
      for (const uint32_t edge_num : targets.edge_indices) {
        const DirectedEdge* directed_edge = tile->directededge(edge_num);
        EdgeInfo edge_info = tile->edgeinfo(directed_edge);

        // only process unnamed pedestrian edges
        if (!edge_info.GetNames().empty())
          continue;

        // skip empty shapes
        const auto& shape = edge_info.shape();
        if (shape.empty()) {
          continue;
        }

        auto name = FindNearestName(tree, shape, results, cos_lat);

        if (!results.empty())
          ++queries_with_results;

        if (name.empty()) {
          ++queries_without_name;
          continue;
        } else {
          enrichments.push_back({edge_num, name});
        }
      }

      LOG_DEBUG("Target tile level={}, tileid={}: targets={}, queries_with_results={}, "
                "queries_without_name={}, enrichments={}",
                static_cast<unsigned int>(tile_id.level()), tile_id.tileid(),
                targets.edge_indices.size(), queries_with_results, queries_without_name,
                enrichments.size());

      [[maybe_unused]] auto t_search_end = std::chrono::steady_clock::now();

      // apply all enrichments
      if (!enrichments.empty()) {
        for (const auto& e : enrichments) {
          GraphId current_edge_id(tile_id.tileid(), tile_id.level(), e.edge_num);
          tilebuilder.AddNameToEdge(current_edge_id, e.name);
          enriched_edges_in_tile++;
        }

        tilebuilder.RecomputeEdgeInfoOffsets();
        tilebuilder.StoreTileData();
      }

      [[maybe_unused]] auto t_apply_end = std::chrono::steady_clock::now();

      LOG_DEBUG("Tile {} (Thread {}), : build={}ms, search={}ms, apply={}ms, enriched={}", tile_num,
                thread_num,
                std::chrono::duration_cast<std::chrono::milliseconds>(t_build_end - t_build_start)
                    .count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(t_search_end - t_build_end)
                    .count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(t_apply_end - t_search_end)
                    .count(),
                enriched_edges_in_tile);

      enriched_edges += enriched_edges_in_tile;
      nb_tiles_treated++;
    }

    LOG_DEBUG("Thread " + std::to_string(thread_num) + " finished: it enriched " +
              std::to_string(enriched_edges) + " pedestrian edges in " +
              std::to_string(nb_tiles_treated) + " tile(s)");

    total_enriched += enriched_edges;
  } catch (const std::exception& e) {
    LOG_ERROR("Thread " + std::to_string(thread_num) + " error: " + e.what());
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

void EnrichPedestrianEdgeNames(const boost::property_tree::ptree& pt) {
  LOG_INFO("Starting pedestrian edge name enrichment...");

  const size_t total_threads =
      pt.get<size_t>("mjolnir.concurrency", std::thread::hardware_concurrency());

  // Get the set of tiles at the local level
  auto mjolnir_pt = pt.get_child("mjolnir");
  // Force GraphReader to read tiles from disk, not from an archived extract
  mjolnir_pt.erase("tile_extract");
  GraphReader reader(mjolnir_pt);
  const auto& levels = TileHierarchy::levels();
  const size_t local_level = levels.back().level;
  auto tileset = reader.GetTileSet(local_level);

  std::vector<GraphId> tile_vec(tileset.begin(), tileset.end());
  // Sort tile_vec by descending edge count (largest first for better load balancing).
  std::sort(tile_vec.begin(), tile_vec.end(), [&reader](const GraphId& a, const GraphId& b) {
    auto ta = reader.GetGraphTile(a);
    auto tb = reader.GetGraphTile(b);
    return (ta ? ta->header()->directededgecount() : 0) >
           (tb ? tb->header()->directededgecount() : 0);
  });
  LOG_INFO("Found " + std::to_string(tile_vec.size()) + " tiles at level " +
           std::to_string(local_level));

  std::atomic<size_t> next_tile{0};
  std::atomic<size_t> total_enriched{0};

  std::vector<std::thread> threads(total_threads);
  for (size_t thread_num = 0; thread_num < total_threads; ++thread_num) {
    threads[thread_num] = std::thread(EnrichWorker, std::cref(mjolnir_pt), std::cref(tile_vec),
                                      thread_num, std::ref(next_tile), std::ref(total_enriched));
  }

  // Join all threads
  for (auto& thread : threads) {
    thread.join();
  }

  LOG_INFO("Pedestrian edge name enrichment complete. Enriched " +
           std::to_string(total_enriched.load()) + " pedestrian edges.");
}

} // namespace mjolnir
} // namespace valhalla