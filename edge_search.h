// -*- mode: c++ -*-

#include <cmath>
#include <tuple>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>

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


using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


std::tuple<PointLL, float, size_t, float>
Project(const PointLL& p, const std::vector<PointLL>& shape, const DistanceApproximator& approximator)
{
  size_t closest_segment = 0.f;
  float closest_distance = std::numeric_limits<float>::max();
  float closest_partial_length = 0.f;
  PointLL closest_point{};
  float total_length = 0.f;

  //for each segment
  for(size_t i = 0; i < shape.size() - 1; ++i) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto bx = v.first - u.first;
    auto by = v.second - u.second;
    const auto scale = ((p.first - u.first)*bx + (p.second - u.second)*by) / (bx*bx + by*by);
    //projects along the ray before u
    if(scale <= 0.f) {
      bx = u.first;
      by = u.second;
    }//projects along the ray after v
    else if(scale >= 1.f) {
      bx = v.first;
      by = v.second;
    }//projects along the ray between u and v
    else {
      bx = bx*scale + u.first;
      by = by*scale + u.second;
    }
    //check if this point is better
    PointLL point(bx, by);
    const auto distance = approximator.DistanceSquared(point);
    if(distance < closest_distance) {
      closest_segment = i;
      closest_distance = distance;
      closest_point = std::move(point);
      closest_partial_length = total_length;
    }

    //total edge length
    total_length += u.Distance(v);
  }

  // Offset is a float between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += shape[closest_segment].Distance(closest_point);
  float offset = static_cast<float>(closest_partial_length / total_length);
  offset = std::max(0.f, std::min(offset, 1.f));
  assert(0.f <= offset && offset <= 1.f);

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, offset);
}


// Some helpers
inline const DirectedEdge*
get_directededge(const GraphTile& tile, const GraphId edgeid)
{
  return tile.directededge(edgeid);
}


inline std::unique_ptr<const EdgeInfo>
get_edgeinfo_ptr(const GraphTile& tile, const DirectedEdge* directededge)
{
  auto offset = directededge->edgeinfo_offset();
  return tile.edgeinfo(offset);
}


inline std::unique_ptr<const EdgeInfo>
get_edgeinfo_ptr(const GraphTile& tile, const GraphId edgeid)
{
  auto edge = get_directededge(tile, edgeid);
  return get_edgeinfo_ptr(tile, edge);
}


inline GraphId
get_edgeid(const GraphTile& tile, const DirectedEdge* directededge)
{
  auto edgeid = tile.header()->graphid();
  edgeid.fields.id = directededge - tile.directededge(0);
  return edgeid;
}


const DirectedEdge* GetOpposingEdge(GraphReader& reader, const DirectedEdge* edge) {
  //get the node at the end of this edge
  const auto node_id = edge->endnode();
  //we are looking for the nth edge exiting this node
  const auto opposing_index = edge->opp_index();
  //the node could be in another tile so we grab that
  const auto tile = reader.GetGraphTile(node_id);
  //grab the nth edge leaving the node
  return tile->directededge(tile->node(node_id)->edge_index() + opposing_index);
}


class CandidateQuery
{
 public:
  CandidateQuery(GraphReader& reader)
      : reader_(reader) {
  }

  virtual ~CandidateQuery() {
  }

  virtual std::vector<Candidate> Query(const PointLL& point, float radius, EdgeFilter filter = nullptr);

  virtual std::vector<std::vector<Candidate>>
  QueryBulk(const std::vector<PointLL>& points, float radius, EdgeFilter filter = nullptr);

 protected:
  GraphReader& reader_;
};


std::vector<Candidate>
CandidateQuery::Query(const PointLL& location,
                      float sq_search_radius,
                      EdgeFilter filter)
{
  const GraphTile* tile = reader_.GetGraphTile(location);
  if(!tile || tile->header()->directededgecount() == 0) {
    return {};
  }

  std::vector<Candidate> candidates;
  std::unordered_set<uint32_t> visited(tile->header()->directededgecount());
  DistanceApproximator approximator(location);

  //for each edge
  const auto first_node = tile->node(0);
  const auto last_node  = first_node + tile->header()->nodecount();
  for (auto start_node = first_node; start_node < last_node; start_node++) {

    //for each edge at this node
    const auto first_edge = tile->directededge(start_node->edge_index());
    const auto last_edge  = first_edge + start_node->edge_count();
    for (auto edge = first_edge; edge < last_edge; edge++) {

      //we haven't looked at this edge yet and its not junk
      if (visited.insert(edge->edgeinfo_offset()).second && (!filter || !filter(edge))) {
        auto edgeinfo_ptr = get_edgeinfo_ptr(*tile, edge);
        auto shape = edgeinfo_ptr->shape();
        PointLL point;
        float sq_distance;
        size_t segment;
        float offset;
        std::tie(point, sq_distance, segment, offset) = Project(location, shape, approximator);
        if (sq_distance <= sq_search_radius) {
          PathLocation correlated(Location(location, Location::StopType::BREAK));
          correlated.CorrelateVertex(point);

          auto edgeid = get_edgeid(*tile, edge);
          correlated.CorrelateEdge({edgeid, edge->forward()? offset : 1.f - offset});

          auto opp_edgeid = reader_.GetOpposingEdgeId(edgeid);
          if (!filter || !filter(get_directededge(*tile, opp_edgeid))) {
            correlated.CorrelateEdge({opp_edgeid, edge->forward()? 1.f - offset : offset});
          }

          candidates.emplace_back(correlated, std::sqrt(sq_distance));
        }
      }
    }
  }

  return candidates;
}


std::vector<std::vector<Candidate>>
CandidateQuery::QueryBulk(const std::vector<PointLL>& locations,
                          float radius,
                          EdgeFilter filter)
{
  std::vector<std::vector<Candidate>> results;
  for (const auto& location : locations) {
    results.push_back(Query(location, radius));
  }
  return results;
}


inline float TranslateLatitudeInMeters(float lat, float meters)
{
  float offset = meters / kMetersPerDegreeLat;
  return lat + offset;
}


inline float TranslateLatitudeInMeters(const PointLL& lnglat, float meters)
{
  return TranslateLatitudeInMeters(lnglat.lat(), meters);
}


inline float TranslateLongitudeInMeters(const PointLL& lnglat, float meters)
{
  float offset = meters / DistanceApproximator::MetersPerLngDegree(lnglat.lat());
  return lnglat.lng() + offset;
}


inline BoundingBox ExtendByMeters(const BoundingBox& bbox, float meters)
{
  // TODO negative meters may cause problems
  PointLL minpt(TranslateLongitudeInMeters(bbox.minpt(), -meters),
                TranslateLatitudeInMeters(bbox.minpt(), -meters));
  PointLL maxpt(TranslateLongitudeInMeters(bbox.maxpt(), meters),
                TranslateLatitudeInMeters(bbox.maxpt(), meters));
  return {minpt, maxpt};
}


inline BoundingBox ExtendByMeters(const PointLL& pt, float meters)
{
  // TODO negative meters may cause problems
  PointLL minpt(TranslateLongitudeInMeters(pt, -meters),
                TranslateLatitudeInMeters(pt, -meters));
  PointLL maxpt(TranslateLongitudeInMeters(pt, meters),
                TranslateLatitudeInMeters(pt, meters));
  return {minpt, maxpt};
}


// Add each road linestring's line segments into grid. Only one side
// of directed edges is added
void IndexTile(const GraphTile& tile, GridRangeQuery<GraphId>* grid_ptr)
{
  std::unordered_set<uint32_t> visited(tile.header()->directededgecount());
  const auto first_node = tile.node(0);
  const auto last_node  = first_node + tile.header()->nodecount();
  for (auto start_node = first_node; start_node < last_node; start_node++) {
    const auto first_edge = tile.directededge(start_node->edge_index());
    const auto last_edge  = first_edge + start_node->edge_count();
    for (auto edge = first_edge; edge < last_edge; edge++) {
      if (visited.insert(edge->edgeinfo_offset()).second) {
        auto shape = get_edgeinfo_ptr(tile, edge)->shape();
        if (shape.empty()) {
          continue;
        }
        auto edgeid = get_edgeid(tile, edge);
        for (decltype(shape.size()) i = 0; i < shape.size() - 1; ++i) {
          grid_ptr->AddLineSegment(edgeid, LineSegment(shape[i], shape[i+1]));
        }
      }
    }
  }
}


class CandidateGridQuery: public CandidateQuery
{
 private:
  const TileHierarchy& hierarchy_;
  float cell_width_;
  float cell_height_;
  std::unordered_map<GraphId, GridRangeQuery<GraphId>> grid_cache_;

 public:
  CandidateGridQuery(GraphReader& reader,
                     float cell_width, float cell_height)
      : CandidateQuery(reader),
        hierarchy_(reader.GetTileHierarchy()),
        cell_width_(cell_width),
        cell_height_(cell_height) {
  }

  ~CandidateGridQuery() {
  }

  const GridRangeQuery<GraphId>* GetGrid(GraphId tile_id)
  {
    return GetGrid(reader_.GetGraphTile(tile_id));
  }

  const GridRangeQuery<GraphId>* GetGrid(const GraphTile* tile_ptr)
  {
    if (!tile_ptr) {
      return nullptr;
    }

    auto tile_id = tile_ptr->id();
    auto cached = grid_cache_.find(tile_id);
    if (cached != grid_cache_.end()) {
      return &(cached->second);
    }

    GridRangeQuery<GraphId> grid(tile_ptr->BoundingBox(hierarchy_), cell_width_, cell_height_);
    // TODO does rehash change pointer addresses?
    auto inserted = grid_cache_.emplace(tile_id, std::move(grid));
    auto grid_ptr = &(inserted.first->second);
    IndexTile(*tile_ptr, grid_ptr);
    return grid_ptr;
  }

  std::unordered_set<GraphId>
  RangeQuery(const BoundingBox& range)
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
          return {};
        }
      }
      return {};
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
        auto set = grid->Query(range);
        result.insert(set.begin(), set.end());
      }
    }

    if (tile_of_maxpt) {
      auto grid = GetGrid(tile_of_maxpt);
      if (grid) {
        auto set = grid->Query(range);
        result.insert(set.begin(), set.end());
      }
    }

    auto tile_of_lefttop = reader_.GetGraphTile(PointLL(range.minx(), range.maxy()));
    if (tile_of_lefttop
        && tile_of_lefttop != tile_of_minpt
        && tile_of_lefttop != tile_of_maxpt) {
      auto grid = GetGrid(tile_of_lefttop);
      if (grid) {
        auto set = grid->Query(range);
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
        auto set = grid->Query(range);
        result.insert(set.begin(), set.end());
      }
    }

    return result;
  }


  std::vector<Candidate>
  Query(const PointLL& location, float sq_search_radius, EdgeFilter filter) override
  {
    const GraphTile* tile = reader_.GetGraphTile(location);
    if(!tile || tile->header()->directededgecount() == 0) {
      return {};
    }

    std::vector<Candidate> candidates;
    DistanceApproximator approximator(location);
    auto range = ExtendByMeters(location, std::sqrt(sq_search_radius));

    for (const auto edgeid : RangeQuery(range)) {
      auto edge = get_directededge(*tile, edgeid);
      if (!filter || !filter(edge)) {
        auto edgeinfo_ptr = get_edgeinfo_ptr(*tile, edgeid);
        const auto& shape = edgeinfo_ptr->shape();
        PointLL point;
        float sq_distance;
        size_t segment;
        float offset;
        std::tie(point, sq_distance, segment, offset) = Project(location, shape, approximator);
        if (sq_distance <= sq_search_radius) {
          PathLocation correlated(Location(location, Location::StopType::BREAK));
          correlated.CorrelateVertex(point);
          correlated.CorrelateEdge({edgeid, edge->forward()? offset : 1.f - offset});

          auto opp_edgeid = reader_.GetOpposingEdgeId(edgeid);
          if (!filter || !filter(get_directededge(*tile, opp_edgeid))) {
            correlated.CorrelateEdge({opp_edgeid, edge->forward()? 1.f - offset : offset});
          }

          candidates.emplace_back(correlated, sq_distance);
        }
      }
    }
    return candidates;
  }
};
