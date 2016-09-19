#include "meili/candidate_search.h"
#include "meili/graph_helpers.h"
#include "meili/geometry_helpers.h"

namespace valhalla {

namespace meili {

CandidateQuery::CandidateQuery(baldr::GraphReader& graphreader):
    reader_(graphreader) {}


std::vector<std::vector<baldr::PathLocation>>
CandidateQuery::QueryBulk(const std::vector<midgard::PointLL>& locations,
                          float radius,
                          sif::EdgeFilter filter)
{
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
                                      sif::EdgeFilter edgefilter) const
{
  std::vector<baldr::PathLocation> candidates;
  std::unordered_set<baldr::GraphId> visited_nodes;
  DistanceApproximator approximator(location);
  const baldr::GraphTile* tile = nullptr;

  for (auto it = edgeid_begin; it != edgeid_end; it++) {
    const auto& edgeid = *it;
    if (!edgeid.Is_Valid()) continue;

    const auto opp_edgeid = helpers::edge_opp_edgeid(reader_, edgeid, tile);
    if (!opp_edgeid.Is_Valid()) continue;
    const auto opp_edge = tile->directededge(opp_edgeid);

    // Make sure it's the last one since we need the tile of this edge
    const auto edge = helpers::edge_directededge(reader_, edgeid, tile);
    if (!edge) continue;

    if (!(edgeid.level() == edge->endnode().level() && edgeid.level() == opp_edgeid.level())) {
      throw std::logic_error("edges feed in candidate filtering should be at the same level as its endnode and opposite edge");
    }

    // NOTE a pointer to edgeinfo is needed here because it returns
    // an unique ptr
    const auto edgeinfo = tile->edgeinfo(edge->edgeinfo_offset());
    const auto& shape = edgeinfo->shape();
    if (shape.empty()) {
      // Otherwise Project will fail
      continue;
    }

    // Projection information
    midgard::PointLL point;
    float sq_distance = 0.f;
    decltype(shape.size()) segment;
    float offset;

    baldr::GraphId snapped_node;
    baldr::PathLocation correlated(baldr::Location(location, baldr::Location::StopType::BREAK));

    // For avoiding recomputing projection later
    const bool edge_included = !edgefilter || edgefilter(edge) != 0.f;

    if (edge_included) {
      std::tie(point, sq_distance, segment, offset) = helpers::Project(location, shape, approximator);

      if (sq_distance <= sq_search_radius) {
        const float dist = edge->forward()? offset : 1.f - offset;
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
      if (!edge_included) {
        std::tie(point, sq_distance, segment, offset) = helpers::Project(location, shape, approximator);
      }

      if (sq_distance <= sq_search_radius) {
        const float dist = opp_edge->forward()? offset : 1.f - offset;
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
void IndexTile(const baldr::GraphTile& tile, CandidateGridQuery::grid_t& grid)
{
  auto edgecount = tile.header()->directededgecount();
  if (edgecount <= 0) {
    return;
  }

  std::unordered_set<uint32_t> visited(edgecount);
  auto edgeid = tile.header()->graphid();
  auto directededge = tile.directededge(0);
  for (size_t idx = 0; idx < edgecount; edgeid++, directededge++, idx++) {
    if (directededge->trans_up()
        || directededge->trans_down()
        || directededge->use() == baldr::Use::kTransitConnection) continue;
    const auto offset = directededge->edgeinfo_offset();
    if (visited.insert(offset).second) {
      const auto edgeinfo = tile.edgeinfo(offset);
      const auto& shape = edgeinfo->shape();
      for (decltype(shape.size()) j = 1; j < shape.size(); ++j) {
        grid.AddLineSegment(edgeid, {shape[j - 1], shape[j]});
      }
    }
  }
}



CandidateGridQuery::CandidateGridQuery(baldr::GraphReader& reader, float cell_width, float cell_height)
    : CandidateQuery(reader),
      hierarchy_(reader.GetTileHierarchy()),
      cell_width_(cell_width),
      cell_height_(cell_height),
      grid_cache_() {}


CandidateGridQuery::~CandidateGridQuery() {}


inline const CandidateGridQuery::grid_t*
CandidateGridQuery::GetGrid(const baldr::GraphId& tile_id) const
{ return GetGrid(reader_.GetGraphTile(tile_id)); }


const CandidateGridQuery::grid_t*
CandidateGridQuery::GetGrid(const baldr::GraphTile* tile) const
{
  if (!tile) {
    return nullptr;
  }

  const auto tile_id = tile->id();
  const auto it = grid_cache_.find(tile_id);
  if (it != grid_cache_.end()) {
    return &(it->second);
  }

  const auto inserted = grid_cache_.emplace(tile_id, grid_t(tile->BoundingBox(hierarchy_), cell_width_, cell_height_));
  IndexTile(*tile, inserted.first->second);
  return &(inserted.first->second);
}


std::unordered_set<baldr::GraphId>
CandidateGridQuery::RangeQuery(const AABB2<midgard::PointLL>& range) const
{
  auto tile_of_minpt = reader_.GetGraphTile(range.minpt()),
       tile_of_maxpt = reader_.GetGraphTile(range.maxpt());

  // If the range is inside a single tile
  //
  // +--------+---------+
  // |        | +----+  |
  // |        | +----+  |
  // +--------+---------+
  // |        |         |
  // |        |         |
  // +--------+---------+
  if (tile_of_minpt == tile_of_maxpt) {
    if (tile_of_minpt) {
      auto grid = GetGrid(tile_of_minpt);
      if (grid)
        return grid->Query(range);
    }
    return std::unordered_set<baldr::GraphId>();
  }

  // Otherwise this range intersects with multiple tiles:
  //
  // NOTE for simplicity we only consider the case that the range is
  // smaller than tile's bounding box
  //
  //   intersects 4 tiles               intersects 2 tiles
  // +---------+-----------+  +-------+---------+  +-------+-------+
  // |      +--+-----+     |  |   +---+----+    |  | +---+ |       |
  // |      |  |     |     |  |   +---+----+    |  | |   | |       |
  // +------+--+-----+-----+  +-------+---------+  +-+---+-+-------+
  // |      |  |     |     |  |       |         |  | +---+ +       |
  // |      +--+-----+     |  |       |         |  |       |       |
  // +---------+-----------+  +-------+---------+  +-------+-------+
  std::unordered_set<baldr::GraphId> result;

  if (tile_of_minpt) {
    auto grid = GetGrid(tile_of_minpt);
    if (grid) {
      const auto& set = grid->Query(range);
      result.insert(set.begin(), set.end());
    }
  }

  if (tile_of_maxpt) {
    auto grid = GetGrid(tile_of_maxpt);
    if (grid) {
      const auto& set = grid->Query(range);
      result.insert(set.begin(), set.end());
    }
  }

  auto tile_of_lefttop = reader_.GetGraphTile(midgard::PointLL(range.minx(), range.maxy()));
  if (tile_of_lefttop
      && tile_of_lefttop != tile_of_minpt
      && tile_of_lefttop != tile_of_maxpt) {
    auto grid = GetGrid(tile_of_lefttop);
    if (grid) {
      const auto& set = grid->Query(range);
      result.insert(set.begin(), set.end());
    }
  }

  auto tile_of_rightbottom = reader_.GetGraphTile(midgard::PointLL(range.maxx(), range.miny()));
  if (tile_of_rightbottom
      && tile_of_rightbottom != tile_of_minpt
      && tile_of_rightbottom != tile_of_maxpt) {
    if(tile_of_rightbottom == tile_of_lefttop)
      throw std::logic_error("The candidate grid range should not be in a single tile");
    auto grid = GetGrid(tile_of_rightbottom);
    if (grid) {
      const auto& set = grid->Query(range);
      result.insert(set.begin(), set.end());
    }
  }

  return result;
}


std::vector<baldr::PathLocation>
CandidateGridQuery::Query(const midgard::PointLL& location,
                          float sq_search_radius,
                          sif::EdgeFilter filter) const
{
  if (!location.IsValid()) {
    throw std::invalid_argument("Expect a valid location");
  }

  const auto& range = helpers::ExpandMeters(location, std::sqrt(sq_search_radius));
  const auto& edgeids = RangeQuery(range);
  return WithinSquaredDistance(location, sq_search_radius,
                               edgeids.begin(), edgeids.end(), filter);
}

}
}
