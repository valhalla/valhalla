// -*- mode: c++ -*-

#include <cmath>
#include <tuple>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/iterator/counting_iterator.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/sif/dynamiccost.h>

#include <mmp/candidate.h>
#include <mmp/grid_range_query.h>
#include <mmp/graph_helpers.h>
#include <mmp/geometry_helpers.h>


namespace mmp
{

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


class CandidateQuery
{
 public:
  CandidateQuery(GraphReader& reader);

  virtual ~CandidateQuery();

  virtual std::vector<Candidate>
  Query(const PointLL& point, float radius, EdgeFilter filter = nullptr) const;

  virtual std::vector<std::vector<Candidate>>
  QueryBulk(const std::vector<PointLL>& points, float radius, EdgeFilter filter = nullptr);

 protected:
  template <typename edgeid_iterator_t> std::vector<Candidate>
  WithinSquaredDistance(const PointLL& location,
                        float sq_search_radius,
                        edgeid_iterator_t edgeid_begin,
                        edgeid_iterator_t edgeid_end,
                        EdgeFilter filter,
                        bool directed) const;

  GraphReader& reader_;
};


CandidateQuery::CandidateQuery(GraphReader& reader)
    : reader_(reader) {}


CandidateQuery::~CandidateQuery() {}


std::vector<Candidate>
CandidateQuery::Query(const PointLL& location,
                      float sq_search_radius,
                      EdgeFilter filter) const
{
  const GraphTile* tile = reader_.GetGraphTile(location);
  if (tile && tile->header()->directededgecount() > 0) {
    // TODO it doesn't work since it is not the right to increase
    // graphids :(
    const auto edgeid_begin = boost::counting_iterator<uint64_t>(tile->id()),
                 edgeid_end = boost::counting_iterator<uint64_t>(static_cast<uint64_t>(tile->id())
                                                                 + tile->header()->directededgecount());
    return WithinSquaredDistance(location, sq_search_radius,
                                 edgeid_begin, edgeid_end, filter, true);
  }
  return {};
}


std::vector<std::vector<Candidate>>
CandidateQuery::QueryBulk(const std::vector<PointLL>& locations,
                          float radius,
                          EdgeFilter filter)
{
  std::vector<std::vector<Candidate>> results;
  results.reserve(locations.size());
  for (const auto& location : locations) {
    results.push_back(Query(location, radius, filter));
  }
  return results;
}


template <typename edgeid_iterator_t>
std::vector<Candidate>
CandidateQuery::WithinSquaredDistance(const PointLL& location,
                                      float sq_search_radius,
                                      edgeid_iterator_t edgeid_begin,
                                      edgeid_iterator_t edgeid_end,
                                      EdgeFilter filter,
                                      bool directed) const
{
  std::vector<Candidate> candidates;
  std::unordered_set<GraphId> visited_nodes, visited_edges;
  DistanceApproximator approximator(location);
  const baldr::GraphTile* tile = nullptr;

  for (auto it = edgeid_begin; it != edgeid_end; it++) {
    // Do a explict cast here as the iterator is probably of type
    // uint64_t
    const auto edgeid = static_cast<GraphId>(*it);
    if (!edgeid.Is_Valid()) continue;

    // Skip if it's visited
    if (directed) {
      if (!visited_edges.insert(edgeid).second) continue;
    }

    const auto opp_edgeid = helpers::edge_opp_edgeid(reader_, edgeid, tile);
    if (!opp_edgeid.Is_Valid()) continue;
    const auto opp_edge = tile->directededge(opp_edgeid);
    assert(opp_edge);
    // Make sure it's the last one since we need the tile of this edge
    const auto edge = helpers::edge_directededge(reader_, edgeid, tile);
    if (!edge) continue;

    if (directed) {
      visited_edges.insert(opp_edgeid);
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
    PointLL point;
    float sq_distance = 0.f;
    decltype(shape.size()) segment;
    float offset;

    GraphId snapped_node;
    Candidate correlated(Location(location, Location::StopType::BREAK));

    // Flag for avoiding recomputing projection later
    bool included = !filter || !filter(edge);

    if (included) {
      std::tie(point, sq_distance, segment, offset) = helpers::Project(location, shape, approximator);

      if (sq_distance <= sq_search_radius) {
        float dist = edge->forward()? offset : 1.f - offset;
        if (dist == 1.f) {
          snapped_node = edge->endnode();
        } else if (dist == 0.f) {
          snapped_node = opp_edge->endnode();
        }
        correlated.CorrelateEdge(Candidate::PathEdge(edgeid, dist));
        correlated.CorrelateVertex(point);
      }
    }

    // Correlate its opp edge
    if (!filter || !filter(opp_edge)) {
      if (!included) {
        std::tie(point, sq_distance, segment, offset) = helpers::Project(location, shape, approximator);
      }

      if (sq_distance <= sq_search_radius) {
        float dist = opp_edge->forward()? offset : 1.f - offset;
        if (dist == 1.f) {
          snapped_node = opp_edge->endnode();
        } else if (dist == 0.f) {
          snapped_node = edge->endnode();
        }
        correlated.CorrelateEdge(Candidate::PathEdge(opp_edgeid, dist));
        correlated.CorrelateVertex(point);
      }
    }

    if (correlated.IsCorrelated()) {
      // Add back if it is an edge correlated or it's a node correlated
      // but it's not added yet
      if (!snapped_node.Is_Valid() || visited_nodes.insert(snapped_node).second) {
        correlated.set_sq_distance(sq_distance);
        candidates.push_back(correlated);
      }
    }
  }

  return candidates;
}


// Add each road linestring's line segments into grid. Only one side
// of directed edges is added
void IndexTile(const GraphTile& tile, GridRangeQuery<GraphId>& grid)
{
  auto edgecount = tile.header()->directededgecount();
  if (edgecount <= 0) {
    return;
  }

  std::unordered_set<uint32_t> visited(edgecount);
  auto edgeid = tile.header()->graphid();
  auto directededge = tile.directededge(0);
  for (size_t idx = 0; idx < edgecount; edgeid++, directededge++, idx++) {
    const auto offset = directededge->edgeinfo_offset();
    if (visited.insert(offset).second) {
      const auto edgeinfo = tile.edgeinfo(offset);
      const auto& shape = edgeinfo->shape();
      for (decltype(shape.size()) j = 1; j < shape.size(); ++j) {
        grid.AddLineSegment(edgeid, LineSegment(shape[j - 1], shape[j]));
      }
    }
  }
}


class CandidateGridQuery: public CandidateQuery
{
 public:
  CandidateGridQuery(GraphReader& reader, float cell_width, float cell_height)
      : CandidateQuery(reader),
        hierarchy_(reader.GetTileHierarchy()),
        cell_width_(cell_width),
        cell_height_(cell_height),
        grid_cache_() {}

  ~CandidateGridQuery() {}

  const GridRangeQuery<GraphId>* GetGrid(GraphId tile_id) const
  { return GetGrid(reader_.GetGraphTile(tile_id)); }

  const GridRangeQuery<GraphId>* GetGrid(const GraphTile* tile_ptr) const
  {
    if (!tile_ptr) {
      return nullptr;
    }

    auto tile_id = tile_ptr->id();
    auto cached = grid_cache_.find(tile_id);
    if (cached != grid_cache_.end()) {
      return &(cached->second);
    }

    auto inserted = grid_cache_.emplace(tile_id, GridRangeQuery<GraphId>(tile_ptr->BoundingBox(hierarchy_), cell_width_, cell_height_));
    IndexTile(*tile_ptr, inserted.first->second);
    return &(inserted.first->second);
  }

  std::unordered_set<GraphId>
  RangeQuery(const AABB2<PointLL>& range) const
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
        if (grid) {
          return grid->Query(range);
        } else {
          // g++-4.9 can't convert {} to empty unordered_set
          // return {};
          return std::unordered_set<GraphId>();
        }
      }
      // g++-4.9 can't convert {} to empty unordered_set
      // return {};
      return std::unordered_set<GraphId>();
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
    std::unordered_set<GraphId> result;

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

    auto tile_of_lefttop = reader_.GetGraphTile(PointLL(range.minx(), range.maxy()));
    if (tile_of_lefttop
        && tile_of_lefttop != tile_of_minpt
        && tile_of_lefttop != tile_of_maxpt) {
      auto grid = GetGrid(tile_of_lefttop);
      if (grid) {
        const auto& set = grid->Query(range);
        result.insert(set.begin(), set.end());
      }
    }

    auto tile_of_rightbottom = reader_.GetGraphTile(PointLL(range.maxx(), range.miny()));
    if (tile_of_rightbottom
        && tile_of_rightbottom != tile_of_minpt
        && tile_of_rightbottom != tile_of_maxpt) {
      assert(tile_of_rightbottom != tile_of_lefttop);
      auto grid = GetGrid(tile_of_rightbottom);
      if (grid) {
        const auto& set = grid->Query(range);
        result.insert(set.begin(), set.end());
      }
    }

    return result;
  }

  std::vector<Candidate>
  Query(const PointLL& location, float sq_search_radius, EdgeFilter filter) const override
  {
    const auto& range = helpers::ExpandMeters(location, std::sqrt(sq_search_radius));
    const auto& edgeids = RangeQuery(range);
    return WithinSquaredDistance(location, sq_search_radius,
                                 edgeids.begin(), edgeids.end(), filter, false);
  }

  std::unordered_map<GraphId, GridRangeQuery<GraphId> >::size_type
  size() const
  { return grid_cache_.size(); }

  void Clear()
  { grid_cache_.clear(); }

 private:
  const TileHierarchy& hierarchy_;
  float cell_width_;
  float cell_height_;
  mutable std::unordered_map<GraphId, GridRangeQuery<GraphId>> grid_cache_;
};

}
