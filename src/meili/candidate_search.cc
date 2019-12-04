#include "meili/candidate_search.h"
#include "baldr/tilehierarchy.h"
#include "meili/geometry_helpers.h"

using namespace valhalla::midgard;

namespace valhalla {

namespace meili {

CandidateQuery::CandidateQuery(baldr::GraphReader& graphreader) : reader_(graphreader) {
}

std::vector<std::vector<baldr::PathLocation>>
CandidateQuery::QueryBulk(const std::vector<midgard::PointLL>& locations,
                          float radius,
                          sif::EdgeFilter filter) {
  std::vector<std::vector<baldr::PathLocation>> results;
  results.reserve(locations.size());
  for (const auto& location : locations) {
    results.push_back(Query(location, radius, filter));
  }
  return results;
}

template <typename edgeid_iterator_t>
std::vector<baldr::PathLocation>
CandidateQuery::WithinSquaredDistance(const midgard::PointLL& location,
                                      float sq_search_radius,
                                      edgeid_iterator_t edgeid_begin,
                                      edgeid_iterator_t edgeid_end,
                                      sif::EdgeFilter edgefilter) const {
  std::vector<baldr::PathLocation> candidates;
  std::unordered_set<baldr::GraphId> visited_nodes;
  midgard::projector_t projector(location);
  const baldr::GraphTile* tile = nullptr;

  for (auto it = edgeid_begin; it != edgeid_end; it++) {
    const auto& edgeid = *it;
    if (!edgeid.Is_Valid()) {
      continue;
    }

    // Get the edge and its opposing edge. Transition edges are not
    // allowed so we do not need to check node levels.
    const auto opp_edgeid = reader_.GetOpposingEdgeId(edgeid, tile);
    if (!opp_edgeid.Is_Valid()) {
      continue;
    }
    const auto* opp_edge = tile->directededge(opp_edgeid);

    // Make sure it's the last one since we need the tile of this edge
    const auto* edge = reader_.directededge(edgeid, tile);
    if (!edge) {
      continue;
    }

    // Get at the shape
    auto shape = tile->edgeinfo(edge->edgeinfo_offset()).lazy_shape();
    if (shape.empty()) {
      // Otherwise Project will fail
      continue;
    }

    // Projection information
    midgard::PointLL point;
    float sq_distance = 0.f;
    size_t segment;
    float offset;

    baldr::GraphId snapped_node;
    baldr::PathLocation correlated(baldr::Location(location, baldr::Location::StopType::BREAK));

    // For avoiding recomputing projection later
    const bool edge_included = !edgefilter || edgefilter(edge) != 0.f;

    if (edge_included) {
      std::tie(point, sq_distance, segment, offset) = helpers::Project(projector, shape);

      if (sq_distance <= sq_search_radius) {
        const float dist = edge->forward() ? offset : 1.f - offset;
        if (dist == 1.f) {
          snapped_node = edge->endnode();
        } else if (dist == 0.f) {
          snapped_node = opp_edge->endnode();
        }
        correlated.edges.emplace_back(edgeid, dist, point, sq_distance);
      }
    }

    bool oppedge_included = !edgefilter || edgefilter(opp_edge) != 0.f;

    // Correlate its opp edge
    if (oppedge_included) {
      // No need to project again if we already did it above
      if (!edge_included) {
        std::tie(point, sq_distance, segment, offset) = helpers::Project(projector, shape);
      }
      if (sq_distance <= sq_search_radius) {
        const float dist = opp_edge->forward() ? offset : 1.f - offset;
        if (dist == 1.f) {
          snapped_node = opp_edge->endnode();
        } else if (dist == 0.f) {
          snapped_node = edge->endnode();
        }
        correlated.edges.emplace_back(opp_edgeid, dist, point, sq_distance);
      }
    }

    if (correlated.edges.size()) {
      // Add back if it is an edge correlated or it's a node correlated
      // but it's not added yet
      if (!snapped_node.Is_Valid() || visited_nodes.insert(snapped_node).second) {
        candidates.emplace_back(std::move(correlated));
      }
    }
  }

  return candidates;
}

// Add each road linestring's line segments into grid. Only one side
// of directed edges is added
void IndexBin(const baldr::GraphTile& tile,
              const int32_t bin_index,
              baldr::GraphReader& reader,
              CandidateGridQuery::grid_t& grid) {
  // Get the edges within the specified bin.
  auto edge_ids = tile.GetBin(bin_index);
  for (const auto& edge_id : edge_ids) {
    // Get the right tile (edges in a bin can be in a different tile if they
    // pass through the tile but do not start or end in the tile). Skip if
    // tile is null.
    const auto* bin_tile =
        edge_id.tileid() == tile.header()->graphid().tileid() ? &tile : reader.GetGraphTile(edge_id);
    if (bin_tile == nullptr) {
      continue;
    }

    // Get the edge shape and add to grid. Use lazy_shape to avoid allocations
    // NOTE: bins do not contain transition edges and transit connection edges
    auto shape = bin_tile->edgeinfo(bin_tile->directededge(edge_id)->edgeinfo_offset()).lazy_shape();
    if (!shape.empty()) {
      PointLL v = shape.pop();
      while (!shape.empty()) {
        const PointLL u = v;
        v = shape.pop();
        grid.AddLineSegment(edge_id, {u, v});
      }
    }
  }
}

CandidateGridQuery::CandidateGridQuery(baldr::GraphReader& reader,
                                       float cell_width,
                                       float cell_height)
    : CandidateQuery(reader), cell_width_(cell_width), cell_height_(cell_height), grid_cache_() {
  bin_level_ = baldr::TileHierarchy::levels().rbegin()->second.level;
}

CandidateGridQuery::~CandidateGridQuery() {
}

inline const CandidateGridQuery::grid_t*
CandidateGridQuery::GetGrid(const int32_t bin_id,
                            const Tiles<PointLL>& tiles,
                            const Tiles<PointLL>& bins) const {
  // Check if the bin is in the cache
  const auto it = grid_cache_.find(bin_id);
  if (it != grid_cache_.end()) {
    return &(it->second);
  }

  // Not in the cache. Get the tile and Index the bin within the tile.
  int32_t ndiv = tiles.nsubdivisions();
  auto rc = bins.GetRowColumn(bin_id);
  int32_t tile_id = tiles.TileId(rc.second / ndiv, rc.first / ndiv);
  baldr::GraphId tileid(tile_id, bin_level_, 0);
  auto tile = reader_.GetGraphTile(tileid);
  if (!tile) {
    return nullptr;
  }

  // Compute bin index within the tile (row-ordered)
  int32_t bin_row = rc.first % ndiv;
  int32_t bin_col = rc.second % ndiv;
  int32_t bin_index = (bin_row * ndiv) + bin_col;

  // Insert the bin into the cache and index the bin
  const auto inserted =
      grid_cache_.emplace(bin_id, grid_t(tile->BoundingBox(), cell_width_, cell_height_));
  IndexBin(*tile, bin_index, reader_, inserted.first->second);
  return &(inserted.first->second);
}

std::unordered_set<baldr::GraphId>
CandidateGridQuery::RangeQuery(const AABB2<midgard::PointLL>& range) const {
  // Get the tiles object from the tile hierarchy and create the bin tiles
  // (subdivisions within the tile)
  Tiles<PointLL> tiles = baldr::TileHierarchy::levels().rbegin()->second.tiles;
  Tiles<PointLL> bins(tiles.TileBounds(), tiles.SubdivisionSize());

  // Get a list of bins within the range. These are "tile Ids" that must
  // be resolved to a Graph Id (tile) / bin combination
  auto bin_list = bins.TileList(range);

  // Iterate through the bins and query grids to get results
  std::unordered_set<baldr::GraphId> result;
  for (auto bin_id : bin_list) {
    auto grid = GetGrid(bin_id, tiles, bins);
    if (grid) {
      const auto set = grid->Query(range);
      result.insert(set.begin(), set.end());
    }
  }
  return result;
}

std::vector<baldr::PathLocation> CandidateGridQuery::Query(const midgard::PointLL& location,
                                                           float sq_search_radius,
                                                           sif::EdgeFilter filter) const {
  if (!location.IsValid()) {
    throw std::invalid_argument("Expect a valid location");
  }

  const auto range = midgard::ExpandMeters(location, std::sqrt(sq_search_radius));
  const auto edgeids = RangeQuery(range);
  return WithinSquaredDistance(location, sq_search_radius, edgeids.begin(), edgeids.end(), filter);
}

} // namespace meili
} // namespace valhalla
