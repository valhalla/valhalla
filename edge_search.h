// -*- mode: c++ -*-

#include <cmath>
#include <tuple>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/iterator/counting_iterator.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/sif/dynamiccost.h>

#include "candidate.h"
#include "grid_range_query.h"
#include "graph_helpers.h"
#include "geometry_helpers.h"


using namespace mm;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


namespace mm {
namespace helpers {

// snapped point, sqaured distance, segment index, offset
template <typename coord_t>
std::tuple<coord_t, float, typename std::vector<coord_t>::size_type, float>
Project(const coord_t& p,
        const typename std::vector<coord_t>& shape,
        const DistanceApproximator& approximator,
        float snap_distance = 0.f)
{
  if (shape.empty()) {
    throw std::invalid_argument("Got empty shape");
  }

  coord_t closest_point(shape.front());
  float closest_distance = approximator.DistanceSquared(closest_point);
  decltype(shape.size()) closest_segment = 0;
  float closest_partial_length = 0.f;
  float total_length = 0.f;

  //for each segment
  for(decltype(shape.size()) i = 0; i < shape.size() - 1; ++i) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto bx = v.first - u.first;
    auto by = v.second - u.second;
    auto sq = bx*bx + by*by;
    const auto scale = sq > 0? (((p.first - u.first)*bx + (p.second - u.second)*by) / sq) : 0.f;
    //projects along the ray before u
    if (scale <= 0.f) {
      bx = u.first;
      by = u.second;
    }//projects along the ray after v
    else if (scale >= 1.f) {
      bx = v.first;
      by = v.second;
    }//projects along the ray between u and v
    else {
      bx = bx*scale + u.first;
      by = by*scale + u.second;
    }
    //check if this point is better
    coord_t point(bx, by);
    const auto distance = approximator.DistanceSquared(point);
    if (distance < closest_distance) {
      closest_point = std::move(point);
      closest_distance = distance;
      closest_segment = i;
      closest_partial_length = total_length;
    }

    //total edge length
    total_length += u.Distance(v);
  }

  // Offset is a float between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += shape[closest_segment].Distance(closest_point);
  float offset = total_length > 0.f? static_cast<float>(closest_partial_length / total_length) : 0.f;
  offset = std::max(0.f, std::min(offset, 1.f));

  // Snapp to vertexes if it's close
  if (total_length * offset <= snap_distance) {
    closest_point = shape.front();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = 0;
    offset = 0.f;
  } else if (total_length * (1.f - offset) <= snap_distance) {
    closest_point = shape.back();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = shape.size() - 1;
    offset = 1.f;
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, offset);
}

}
}


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
  template <typename iterator_t> std::vector<Candidate>
  Filter(const PointLL& location,
         float sq_search_radius,
         iterator_t edge_begin,
         iterator_t edge_end,
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
    const auto begin = boost::counting_iterator<uint64_t>(tile->id()),
                 end = boost::counting_iterator<uint64_t>(static_cast<uint64_t>(tile->id())
                                                          + tile->header()->directededgecount());
    return Filter(location, sq_search_radius, begin, end, filter, true);
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


template <typename iterator_t>
std::vector<Candidate>
CandidateQuery::Filter(const PointLL& location,
                       float sq_search_radius,
                       iterator_t begin,
                       iterator_t end,
                       EdgeFilter filter,
                       bool directed) const
{
  std::vector<Candidate> candidates;
  std::unordered_set<GraphId> visited_nodes, visited_edges;
  DistanceApproximator approximator(location);

  for (auto it = begin; it != end; it++) {
    // Do a explict cast here as the iterator is probably of type
    // uint64_t
    const auto edgeid = static_cast<GraphId>(*it);

    // Skip if it's visited
    if (directed) {
      if (!visited_edges.insert(edgeid).second) continue;
    }

    const auto tile = reader_.GetGraphTile(edgeid);
    if (!tile) continue;

    const auto edge = tile->directededge(edgeid);
    const auto opp_edge = reader_.GetOpposingEdge(edgeid);
    const auto opp_edgeid = reader_.GetOpposingEdgeId(edgeid);

    if (directed) {
      visited_edges.insert(edgeid);
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
    float sq_distance;
    decltype(shape.size()) segment;
    float offset;

    GraphId snapped_node;
    PathLocation correlated(Location(location, Location::StopType::BREAK));

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
        correlated.CorrelateEdge(PathLocation::PathEdge(edgeid, dist));
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
        correlated.CorrelateEdge(PathLocation::PathEdge(opp_edgeid, dist));
        correlated.CorrelateVertex(point);
      }
    }

    if (!correlated.edges().empty()) {
      // Add back if it is an edge correlated or it's a node correlated
      // but it's not added yet
      if (!snapped_node.Is_Valid() || visited_nodes.insert(snapped_node).second) {
        candidates.emplace_back(correlated, sq_distance);
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
    auto offset = directededge->edgeinfo_offset();
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
    const auto& edges = RangeQuery(range);
    return Filter(location, sq_search_radius, edges.begin(), edges.end(), filter, false);
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
