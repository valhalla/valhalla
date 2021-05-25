#include "meili/candidate_search.h"
#include "baldr/tilehierarchy.h"
#include "meili/geometry_helpers.h"

using namespace valhalla::midgard;

namespace valhalla {

namespace meili {

struct CandidateCollector {
public:
  explicit CandidateCollector(baldr::GraphReader& reader) : reader_(reader) {
  }

  template <typename edgeid_iterator_t>
  std::vector<baldr::PathLocation> WithinSquaredDistance(const midgard::PointLL& location,
                                                         baldr::Location::StopType stop_type,
                                                         float sq_search_radius,
                                                         edgeid_iterator_t edgeid_begin,
                                                         edgeid_iterator_t edgeid_end,
                                                         const sif::cost_ptr_t& costing) const;

private:
  baldr::GraphReader& reader_;
};

template <typename edgeid_iterator_t>
std::vector<baldr::PathLocation>
CandidateCollector::WithinSquaredDistance(const midgard::PointLL& location,
                                          baldr::Location::StopType stop_type,
                                          float sq_search_radius,
                                          edgeid_iterator_t edgeid_begin,
                                          edgeid_iterator_t edgeid_end,
                                          const sif::cost_ptr_t& costing) const {
  std::vector<baldr::PathLocation> candidates;
  std::unordered_set<baldr::GraphId> visited_nodes;
  midgard::projector_t projector(location);
  graph_tile_ptr tile;

  for (auto it = edgeid_begin; it != edgeid_end; it++) {
    const auto& edgeid = *it;
    if (!edgeid.Is_Valid()) {
      continue;
    }

    // Get the edge. Transition edges are not allowed so we do not need to check node levels.
    const auto* edge = reader_.directededge(edgeid, tile);
    if (!edge) {
      continue;
    }

    // Get the opposing edge as well
    auto opp_tile = tile;
    const baldr::DirectedEdge* opp_edge = nullptr;
    const auto opp_edgeid = reader_.GetOpposingEdgeId(edgeid, opp_edge, opp_tile);
    if (!opp_edgeid.Is_Valid()) {
      continue;
    }

    // Get at the shape
    auto shape = tile->edgeinfo(edge).lazy_shape();
    if (shape.empty()) {
      // Otherwise Project will fail
      continue;
    }

    // Projection information
    midgard::PointLL point;
    double sq_distance = 0.0;
    size_t segment;
    double offset;

    baldr::GraphId snapped_node;
    baldr::PathLocation correlated(baldr::Location(location, stop_type));

    // For avoiding recomputing projection later
    const bool edge_included = !costing || costing->Allowed(edge, tile, sif::kDisallowShortcut);

    if (edge_included) {
      std::tie(point, sq_distance, segment, offset) = helpers::Project(projector, shape);

      if (sq_distance <= sq_search_radius) {
        const double dist = edge->forward() ? offset : 1.0 - offset;
        if (dist == 1.0) {
          snapped_node = edge->endnode();
        } else if (dist == 0.0) {
          snapped_node = opp_edge->endnode();
        }
        correlated.edges.emplace_back(edgeid, dist, point, sq_distance);
      }
    }

    bool oppedge_included = !costing || costing->Allowed(opp_edge, opp_tile, sif::kDisallowShortcut);

    // Correlate its opp edge
    if (oppedge_included) {
      // No need to project again if we already did it above
      if (!edge_included) {
        std::tie(point, sq_distance, segment, offset) = helpers::Project(projector, shape);
      }
      if (sq_distance <= sq_search_radius) {
        const double dist = opp_edge->forward() ? offset : 1.0 - offset;
        if (dist == 1.0) {
          snapped_node = opp_edge->endnode();
        } else if (dist == 0.0) {
          snapped_node = edge->endnode();
        }
        correlated.edges.emplace_back(opp_edgeid, dist, point, sq_distance);
      }
    }

    // We found some edge candidates within the distance cut off
    if (correlated.edges.size()) {
      // If the candidates are not at a node we just add them if they are at a node
      // we avoid adding them multiple times by remembering the node we snapped to
      // this has two consequences:
      // 1. it allows us to keep the number of edge candidates low which keeps the search fast
      // 2. in routing.cc we will find a route to the node, ie not the candidate edge, which means
      //    the route may not end or begin with the candidates we store here, we will need to
      //    handle this case inside of FindMatchResult which expects a candidate to be used
      if (!snapped_node.Is_Valid() || visited_nodes.insert(snapped_node).second) {
        candidates.emplace_back(std::move(correlated));
      }
    }
  }

  return candidates;
}

// Add each road linestring's line segments into grid. Only one side
// of directed edges is added
void IndexBin(const graph_tile_ptr& tile,
              const int32_t bin_index,
              baldr::GraphReader& reader,
              CandidateGridQuery::grid_t& grid) {
  assert(tile);

  // Get the edges within the specified bin.
  auto edge_ids = tile->GetBin(bin_index);
  for (const auto& edge_id : edge_ids) {
    // Get the right tile (edges in a bin can be in a different tile if they
    // pass through the tile but do not start or end in the tile). Skip if
    // tile is null.
    auto bin_tile = tile;
    reader.GetGraphTile(edge_id, bin_tile);
    if (bin_tile == nullptr) {
      continue;
    }

    // Get the edge shape and add to grid. Use lazy_shape to avoid allocations
    // NOTE: bins do not contain transition edges and transit connection edges
    auto shape = bin_tile->edgeinfo(bin_tile->directededge(edge_id)).lazy_shape();
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
    : reader_(reader), cell_width_(cell_width), cell_height_(cell_height), grid_cache_() {
  bin_level_ = baldr::TileHierarchy::levels().back().level;
}

CandidateGridQuery::~CandidateGridQuery() = default;

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
  IndexBin(tile, bin_index, reader_, inserted.first->second);
  return &(inserted.first->second);
}

std::unordered_set<baldr::GraphId>
CandidateGridQuery::RangeQuery(const AABB2<midgard::PointLL>& range) const {
  // Get the tiles object from the tile hierarchy and create the bin tiles
  // (subdivisions within the tile)
  const Tiles<PointLL>& tiles = baldr::TileHierarchy::levels().back().tiles;
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
                                                           baldr::Location::StopType stop_type,
                                                           float sq_search_radius,
                                                           const sif::cost_ptr_t& costing) const {
  CandidateCollector collector(reader_);
  return Query(location, stop_type, sq_search_radius, costing, collector);
}

} // namespace meili
} // namespace valhalla
