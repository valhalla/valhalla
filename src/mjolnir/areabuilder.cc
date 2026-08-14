#include "baldr/graphconstants.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "midgard/sequence.h"
#include "mjolnir/areabuilder.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/osmway.h"
#include "scoped_timer.h"

#include <geos_c.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

constexpr double kMinAreaSquareMeters = 100.0;
constexpr double kDensifyToleranceMeters = 4.0;
constexpr double kSimplifyToleranceMeters = 1.5;
// TODO: an entrance is a vertex of the polygon, so its distance to it should ideally
// be zero. this is a margin against the floating point error accumulated through the
// projection and GEOS; the exact value is worth revisiting.
constexpr double kEntranceToleranceMeters = 0.01;
constexpr float kTraversalSpeedKph = 12.0f;
constexpr double kEntranceGroupingMeters = 3.0;

namespace {
using namespace valhalla;

// can two points be joined by a straight segment that stays inside the polygon?
bool visible_within(const GEOSPreparedGeometry* prepared,
                    const midgard::Point2d& a,
                    const midgard::Point2d& b) {
  GEOSCoordSequence* seq = GEOSCoordSeq_create(2, 2);
  GEOSCoordSeq_setXY(seq, 0, a.x(), a.y());
  GEOSCoordSeq_setXY(seq, 1, b.x(), b.y());
  GEOSGeometry* segment = GEOSGeom_createLineString(seq);
  const bool visible = segment && GEOSPreparedCovers(prepared, segment) == 1;
  if (segment) {
    GEOSGeom_destroy(segment);
  }
  return visible;
}
/**
 * Computes the medial axis of a polygon. Densifies the boundary,
 * builds a Voronoi diagram of the vertices, keeps the edges inside the
 * polygon, prunes the branches reaching toward the corners, and stitches the
 * survivors into chains. The polygon is expected in a local metric projection.
 */
std::vector<std::vector<midgard::Point2d>> GenerateMedialAxis(const GEOSGeometry* polygon) {

  // densify the polygon boundary so the voronoi diagram has enough points
  GEOSGeometry* densified = GEOSDensify(polygon, kDensifyToleranceMeters);
  if (!densified) {
    return {};
  }

  // compute the voronoi diagram using the densified polygon vertices
  GEOSGeometry* voronoi = GEOSVoronoiDiagram(densified, nullptr, 0.0, GEOS_VORONOI_ONLY_EDGES);
  if (!voronoi) {
    GEOSGeom_destroy(densified);
    return {};
  }

  int num_voronoi_edges = GEOSGetNumGeometries(voronoi);

  // keep only the voronoi edges that fall completely inside the polygon
  const GEOSPreparedGeometry* prepared = GEOSPrepare(polygon);
  std::vector<GEOSGeometry*> medial_edges;
  for (int i = 0; i < num_voronoi_edges; ++i) {
    const GEOSGeometry* edge = GEOSGetGeometryN(voronoi, i);
    if (GEOSPreparedContains(prepared, edge) == 1) {
      medial_edges.push_back(GEOSGeom_clone(edge));
    }
  }
  GEOSPreparedGeom_destroy(prepared);

  // round the coordinates so that vertices that are the same point share a key
  auto vertex_key = [](double x, double y) {
    return std::make_pair(std::llround(x * 1000.0), std::llround(y * 1000.0));
  };

  std::map<std::pair<int64_t, int64_t>, int> vertex_ids;
  std::vector<midgard::Point2d> vertex_coords;

  // lambda function to set and return the id of a vertex from its coords
  auto get_vertex_id = [&](const std::pair<int64_t, int64_t>& key, double x, double y) {
    auto it = vertex_ids.find(key);
    if (it != vertex_ids.end()) {
      return it->second;
    }
    int id = static_cast<int>(vertex_ids.size());
    vertex_ids.emplace(key, id);
    vertex_coords.push_back({x, y});
    return id;
  };
  // segments are pairs of vertex ids, seen dedups them
  // since an undirected segment is the same in both directions
  std::vector<std::pair<int, int>> segments;
  std::set<std::pair<int, int>> seen;

  // iterate through the edges, to fill segments vector with vertices ids
  for (const GEOSGeometry* edge : medial_edges) {
    const GEOSCoordSequence* seq = GEOSGeom_getCoordSeq(edge);
    unsigned int num_points = 0;
    GEOSCoordSeq_getSize(seq, &num_points);
    // itarate points in a edge in pairs
    for (unsigned int p = 0; p + 1 < num_points; ++p) {
      double x1, y1, x2, y2;
      GEOSCoordSeq_getXY(seq, p, &x1, &y1);
      GEOSCoordSeq_getXY(seq, p + 1, &x2, &y2);
      const auto k1 = vertex_key(x1, y1);
      const auto k2 = vertex_key(x2, y2);
      if (k1 == k2) {
        continue;
      }
      const int v1 = get_vertex_id(k1, x1, y1);
      const int v2 = get_vertex_id(k2, x2, y2);
      // dedup, an undirected segment is the same in both directions
      const auto key = std::minmax(v1, v2);
      if (!seen.emplace(key.first, key.second).second) {
        continue;
      }
      segments.push_back({v1, v2});
    }
  }

  // incident segments of each vertex, the degrees are computed once and never updated
  std::vector<std::vector<int>> incident(vertex_ids.size());
  for (size_t s = 0; s < segments.size(); ++s) {
    incident[segments[s].first].push_back(static_cast<int>(s));
    incident[segments[s].second].push_back(static_cast<int>(s));
  }

  // a vertex is important if it is not a pass through vertex
  auto is_important = [&](int v) { return incident[v].size() != 2; };

  // prune the branches, walk from each degree 1 vertex until a fork is reached
  std::vector<bool> removed(segments.size(), false);
  for (size_t v = 0; v < incident.size(); ++v) {
    if (incident[v].size() != 1) {
      continue;
    }
    int current_vertex = static_cast<int>(v);
    int current_segment = incident[v].front();
    while (true) {
      removed[current_segment] = true;
      const auto& seg = segments[current_segment];
      const int next_vertex = (seg.first == current_vertex) ? seg.second : seg.first;
      // stop at a fork, continue at degree 2 vertex
      if (is_important(next_vertex)) {
        break;
      }
      // keep walking along the branch
      int next_segment = -1;
      for (int s : incident[next_vertex]) {
        if (s != current_segment) {
          next_segment = s;
          break;
        }
      }
      if (next_segment == -1 || removed[next_segment]) {
        break;
      }
      current_vertex = next_vertex;
      current_segment = next_segment;
    }
  }

  // stitch the surviving segments into chains, cutting at forks and endpoints
  std::vector<bool> used(segments.size(), false);
  std::vector<std::vector<midgard::Point2d>> chains;

  // TODO: this duplicates the pruning walk above, they differ only in whether they
  // mark segments removed or collect their coordinates. could be unified.
  //
  // walks a chain from an important vertex until it reaches another one
  auto walk_chain = [&](int start_vertex, int start_segment) {
    std::vector<midgard::Point2d> chain;
    chain.push_back(vertex_coords[start_vertex]);
    // similar logic as the prune, but instead of removing, collecting
    int current_vertex = start_vertex;
    int current_segment = start_segment;
    while (true) {
      used[current_segment] = true;
      const auto& seg = segments[current_segment];
      const int next_vertex = (seg.first == current_vertex) ? seg.second : seg.first;
      chain.push_back(vertex_coords[next_vertex]);
      if (is_important(next_vertex)) {
        break;
      }
      // pass through vertex, keep walking through the other segment
      int next_segment = -1;
      for (int s : incident[next_vertex]) {
        if (s != current_segment && !removed[s] && !used[s]) {
          next_segment = s;
          break;
        }
      }
      if (next_segment == -1) {
        break;
      }
      current_vertex = next_vertex;
      current_segment = next_segment;
    }
    return chain;
  };

  // start the chains from the important vertices
  for (size_t v = 0; v < incident.size(); ++v) {
    if (!is_important(static_cast<int>(v))) {
      continue;
    }
    for (int s : incident[v]) {
      if (removed[s] || used[s]) {
        continue;
      }
      chains.push_back(walk_chain(static_cast<int>(v), s));
    }
  }

  // any segment still unused belongs to a cycle with no important vertices
  for (size_t s = 0; s < segments.size(); ++s) {
    if (removed[s] || used[s]) {
      continue;
    }
    chains.push_back(walk_chain(segments[s].first, static_cast<int>(s)));
  }

  // clean up the kept edges
  for (GEOSGeometry* edge : medial_edges) {
    GEOSGeom_destroy(edge);
  }
  GEOSGeom_destroy(voronoi);
  GEOSGeom_destroy(densified);
  return chains;
}

/**
 * Turns each traversal line into an OSMWay plus its OSMWayNodes, appending them to
 * the ways and way_nodes files so ConstructEdges picks them up like any other way.
 */
void materialise_traversals(
    const std::vector<std::vector<std::pair<uint64_t, midgard::PointLL>>>& traversal_lines,
    const std::unordered_set<uint64_t>& entrance_ids,
    uint32_t name_index,
    uint64_t& next_synthetic_way_id,
    midgard::sequence<mjolnir::OSMWay>& ways,
    midgard::sequence<mjolnir::OSMWayNode>& way_nodes) {

  // a node shared by more than one line is a junction, and so is every line end
  std::unordered_map<uint64_t, uint32_t> node_use_count;
  for (const auto& line : traversal_lines) {
    for (const auto& [node_id, ll] : line) {
      ++node_use_count[node_id];
    }
  }

  for (const auto& line : traversal_lines) {
    // where this way will land in the file, which is what its nodes reference
    const uint32_t way_index = static_cast<uint32_t>(ways.size());
    // TODO: entrance nodes and restored-perimeter ways get generic pedestrian
    // attributes instead of preserving their original OSM tags. Re-emitting an
    // entrance from scratch loses whatever the real node had; a proper fix would
    // look up the original and merge, rather than overwrite.
    for (size_t i = 0; i < line.size(); ++i) {
      const auto& [node_id, ll] = line[i];
      mjolnir::OSMNode osm_node{node_id};
      osm_node.set_latlng(ll.lng(), ll.lat());
      osm_node.set_synthetic(entrance_ids.count(node_id) == 0);
      osm_node.set_access(baldr::kPedestrianAccess | baldr::kWheelchairAccess);
      osm_node.intersection_ = i == 0 || i == line.size() - 1 || node_use_count[node_id] > 1;

      way_nodes.push_back({osm_node, way_index, static_cast<uint32_t>(i)});
    }
    // TODO: the traversal ways get hardcoded attributes (service road class, footway
    // use, fixed speed). Ideally they'd inherit the area's own attributes, so the
    // crossing reflects what the square is actually like instead of a generic footway.
    mjolnir::OSMWay w{next_synthetic_way_id++};
    w.set_node_count(line.size());
    w.set_road_class(baldr::RoadClass::kServiceOther);
    w.set_use(baldr::Use::kFootway);
    w.set_speed(kTraversalSpeedKph);
    w.set_pedestrian_forward(true);
    w.set_pedestrian_backward(true);
    if (name_index != 0) {
      w.set_name_index(name_index);
    }
    ways.push_back(w);
  }
}
} // namespace
namespace valhalla {
namespace mjolnir {

void AreaBuilder::BuildAreas(const boost::property_tree::ptree& /*pt*/,
                             const std::string& ways_file,
                             const std::string& way_nodes_file,
                             OSMData& osmdata) {
  SCOPED_TIMER();

  // way_id > relation_id, to know which relation each area way belongs to
  std::unordered_map<uint64_t, uint64_t> way_to_relation;
  way_to_relation.reserve(osmdata.area_relations.size());
  for (const auto& entry : osmdata.area_relations) {
    way_to_relation[entry.second.way_id] = entry.first;
  }

  midgard::sequence<OSMWay> ways(ways_file, false);
  midgard::sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  size_t skipped_entrances = 0;

  // the following maps are all keyed by area id: the relation id when the area comes
  // from a multipolygon relation, or the way id when it is a simple closed way.
  //
  // the shape of each member way, one point list per way
  std::unordered_map<uint64_t, std::vector<std::vector<midgard::PointLL>>> area_ways;
  // the perimeter nodes that are shared with other ways, the entrance candidates
  std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, midgard::PointLL>>> area_shared_nodes;
  // the name index of the area, inherited by the traversals we generate for it
  std::unordered_map<uint64_t, uint32_t> area_name_indices;
  // where the area's ways live in the ways file, so we can edit them later
  std::unordered_map<uint64_t, std::vector<size_t>> area_way_indices;

  // for each intersection node, which pedestrian ways pass through it and at which
  // position, to detect ways that cross through an area
  struct WayVisit {
    uint64_t way_id;
    uint32_t position;
  };
  std::unordered_map<uint64_t, std::vector<WayVisit>> pedestrian_node_ways;

  // where each pedestrian way's nodes live in the way_nodes file, to fetch the
  // in-between nodes of crossing candidates later
  struct WaySpan {
    size_t first_index;
    size_t last_index;
  };
  std::unordered_map<uint64_t, WaySpan> pedestrian_way_spans;

  // first pass for areas, collect each area's shape, and remember which nodes lie on some perimeter.
  std::unordered_set<uint64_t> area_perimeter_nodes;

  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index =
        first_way_node_index + way.node_count() - way_node.way_shape_node_index - 1;

    if (way.area()) {
      auto it = way_to_relation.find(way.way_id());
      uint64_t area_key = (it != way_to_relation.end()) ? it->second : way.way_id();
      // TODO: this takes the name from the member way, which is right for areas that
      // are a single closed way but wrong for relations: there the name lives on the
      // relation itself, not on its members, so relation-based areas either inherit a
      // member's name or end up unnamed. Carrying the relation name through
      // area_relations would fix it.
      if (way.name_index() != 0) {
        area_name_indices[area_key] = way.name_index();
      }
      area_way_indices[area_key].push_back(way_node.way_index);
      std::vector<midgard::PointLL> shape;
      for (auto node_idx = first_way_node_index; node_idx <= last_way_node_index; node_idx++) {
        const auto& node = (*way_nodes[node_idx]).node;
        shape.push_back(node.latlng());
        if (node.intersection()) {
          area_shared_nodes[area_key].push_back({node.osmid_, node.latlng()});
          area_perimeter_nodes.insert(node.osmid_);
        }
      }
      area_ways[area_key].push_back(std::move(shape));
    }
    current_way_node_index = last_way_node_index + 1;
  }

  // second pass over all non-area pedestrian ways, keeping only the ones that share a
  // node with some area perimeter
  current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index =
        first_way_node_index + way.node_count() - way_node.way_shape_node_index - 1;

    if (!way.area() && (way.pedestrian_forward() || way.pedestrian_backward())) {
      bool shares_area_node = false;
      for (auto node_idx = first_way_node_index; node_idx <= last_way_node_index; node_idx++) {
        const auto& node = (*way_nodes[node_idx]).node;
        if (node.intersection() && area_perimeter_nodes.count(node.osmid_)) {
          shares_area_node = true;
          pedestrian_node_ways[node.osmid_].push_back(
              {way.way_id(), static_cast<uint32_t>(node_idx - first_way_node_index)});
        }
      }
      // only ways that share a node with an area can be crossing candidates, so we only need
      // to remember where their nodes live in the file for those
      if (shares_area_node) {
        pedestrian_way_spans[way.way_id()] = {first_way_node_index, last_way_node_index};
      }
    }
    current_way_node_index = last_way_node_index + 1;
  }

  // with the areas and the pedestrian ways touching them collected, assemble each
  // area's polygons, generate a traversal skeleton along their medial axis, connect
  // the entrances to it, and materialise the result as routable ways
  initGEOS(nullptr, nullptr);

  uint64_t next_synthetic_osm_id = osmdata.max_way_id + 1;
  uint64_t next_synthetic_node_id = osmdata.max_node_id + 1;
  for (const auto& [area_id, shapes] : area_ways) {
    if (shapes.empty() || shapes.front().empty()) {
      continue;
    }

    // the perimeter nodes of this area that a pedestrian way also passes through,
    // deduped since a node can appear in several of the area's member ways
    std::vector<std::pair<uint64_t, midgard::PointLL>> entrances;
    std::unordered_set<uint64_t> seen_entrances;
    auto shared_it = area_shared_nodes.find(area_id);
    if (shared_it == area_shared_nodes.end()) {
      continue;
    }
    for (const auto& [node_id, ll] : shared_it->second) {
      if (pedestrian_node_ways.count(node_id) && seen_entrances.insert(node_id).second) {
        entrances.push_back({node_id, ll});
      }
    }
    if (entrances.empty()) {
      continue;
    }

    // pedestrian way id -> the positions in that way's own node sequence where it
    // hits this area's perimeter
    std::unordered_map<uint64_t, std::vector<uint32_t>> way_hits;
    // detect footways that may cross through the area boundary
    for (const auto& [node_id, ll] : entrances) {
      auto ways_it = pedestrian_node_ways.find(node_id);
      if (ways_it == pedestrian_node_ways.end()) {
        continue;
      }
      for (const auto& visit : ways_it->second) {
        way_hits[visit.way_id].push_back(visit.position);
      }
    }
    // TODO: a pedestrian way that runs exactly along the perimeter, or crosses in
    // a straight line with only two nodes, isn't detected as a mapped path. Its
    // perimeter hits are consecutive and it has no in-between node inside the
    // polygon.
    //
    // ways whose hits are non consecutive, so they left the perimeter and came back:
    // those are the ones worth testing geometrically for paths inside the area
    std::vector<uint64_t> crossing_candidates;
    for (auto& [way_id, positions] : way_hits) {
      if (positions.size() < 2) {
        continue;
      }
      std::sort(positions.begin(), positions.end());
      for (size_t i = 0; i + 1 < positions.size(); ++i) {
        if (positions[i + 1] - positions[i] > 1) {
          crossing_candidates.push_back(way_id);
          break;
        }
      }
    }
    // reference point for the local projection, first point of the first way
    const midgard::PointLL center = shapes.front().front();
    const midgard::AzimuthalEquidistant projection(center);

    // project the entrances to meters and build a GEOS point for each one
    std::vector<std::pair<uint64_t, midgard::Point2d>> entrances_m;
    entrances_m.reserve(entrances.size());
    std::vector<GEOSGeometry*> entrance_points;
    entrance_points.reserve(entrances.size());
    for (const auto& [node_id, entrance] : entrances) {
      const midgard::Point2d p = projection.project(entrance);
      entrances_m.push_back({node_id, p});
      entrance_points.push_back(GEOSGeom_createPointFromXY(p.x(), p.y()));
    }

    // project each way to meters and create a GEOS line for each one
    std::vector<GEOSGeometry*> lines;
    for (const auto& shape : shapes) {
      if (shape.size() < 2) {
        continue;
      }
      // sequence of meter coords of the nodes of a way
      GEOSCoordSequence* seq = GEOSCoordSeq_create(shape.size(), 2);
      for (size_t i = 0; i < shape.size(); i++) {
        const midgard::Point2d p = projection.project(shape[i]);
        GEOSCoordSeq_setXY(seq, i, p.x(), p.y());
      }
      // create a line with these points
      GEOSGeometry* line = GEOSGeom_createLineString(seq);
      if (line) {
        lines.push_back(line);
      }
    }
    if (lines.empty()) {
      for (GEOSGeometry* point : entrance_points) {
        GEOSGeom_destroy(point);
      }
      continue;
    }
    // with the lines we created, returns a collection of polygons
    GEOSGeometry* polygons = GEOSPolygonize_valid(lines.data(), lines.size());
    int num_polygons = polygons ? GEOSGetNumGeometries(polygons) : 0;

    // keep the polygons that are not filling a hole of another polygon
    std::vector<GEOSGeometry*> final_polygons;
    final_polygons.reserve(num_polygons);
    for (int i = 0; i < num_polygons; ++i) {
      final_polygons.push_back(GEOSGeom_clone(GEOSGetGeometryN(polygons, i)));
    }

    LOG_DEBUG("area " + std::to_string(area_id) + ": " + std::to_string(lines.size()) + " ways -> " +
              std::to_string(final_polygons.size()) + " polygon(s)");

    bool generated_any = false;
    bool restore_perimeter = false;
    for (GEOSGeometry* poly : final_polygons) {
      double area = 0;
      GEOSArea(poly, &area);
      // skip areas that are too small to be worth routing through
      if (area < kMinAreaSquareMeters) {
        restore_perimeter = true;
        continue;
      }

      // a relation can hold several polygons, so keep the entrances lying on this one
      std::vector<std::pair<uint64_t, midgard::Point2d>> poly_entrances;
      for (size_t i = 0; i < entrance_points.size(); ++i) {
        double distance = 0;
        GEOSDistance(poly, entrance_points[i], &distance);
        if (distance < kEntranceToleranceMeters) {
          poly_entrances.push_back(entrances_m[i]);
        }
      }

      // an area nobody can reach is not worth generating
      if (poly_entrances.empty()) {
        continue;
      }
      const GEOSPreparedGeometry* prepared_poly = GEOSPrepare(poly);
      // does any in-between node of the way fall strictly inside this polygon?
      // if so the area already has its paths mapped, so we skip it
      bool has_interior_paths = false;
      for (const uint64_t candidate : crossing_candidates) {
        const auto span_it = pedestrian_way_spans.find(candidate);
        if (span_it == pedestrian_way_spans.end()) {
          continue;
        }
        for (size_t idx = span_it->second.first_index;
             idx <= span_it->second.last_index && !has_interior_paths; ++idx) {
          const midgard::Point2d p = projection.project((*way_nodes[idx]).node.latlng());
          GEOSGeometry* point = GEOSGeom_createPointFromXY(p.x(), p.y());
          if (GEOSContains(poly, point) == 1) {
            has_interior_paths = true;
          }
          GEOSGeom_destroy(point);
        }
        if (has_interior_paths) {
          GEOSPreparedGeom_destroy(prepared_poly);
          break;
        }
      }
      if (has_interior_paths) {
        LOG_DEBUG("area " + std::to_string(area_id) +
                  ": polygon already has mapped paths inside, skipping");
        continue;
      }

      // group entrances that sit within kEntranceGroupingMeters of each other,
      // keeping per group the real entrance closest to the group centroid
      {
        const double group_dist_sq = kEntranceGroupingMeters * kEntranceGroupingMeters;
        std::vector<int> group_of(poly_entrances.size(), -1);
        int num_groups = 0;
        for (size_t i = 0; i < poly_entrances.size(); ++i) {
          for (size_t j = 0; j < i && group_of[i] == -1; ++j) {
            // two entrances are grouped if they are within the allowed distance and there is
            // line of sight (no polygon wall blocking the path between them)
            if (poly_entrances[i].second.DistanceSquared(poly_entrances[j].second) < group_dist_sq &&
                visible_within(prepared_poly, poly_entrances[i].second, poly_entrances[j].second)) {
              group_of[i] = group_of[j];
            }
          }
          // if the current entrance is not close to any existing group, create a new one
          if (group_of[i] == -1) {
            group_of[i] = num_groups++;
          }
        }
        // only calculate centroids if there are groups
        if (num_groups < static_cast<int>(poly_entrances.size())) {
          std::vector<double> cx(num_groups, 0.0), cy(num_groups, 0.0);
          std::vector<int> count(num_groups, 0);
          // sum the coords of all entrances in the same group
          for (size_t i = 0; i < poly_entrances.size(); ++i) {
            cx[group_of[i]] += poly_entrances[i].second.x();
            cy[group_of[i]] += poly_entrances[i].second.y();
            count[group_of[i]]++;
          }
          std::vector<int> representative(num_groups, -1);
          std::vector<double> best(num_groups, std::numeric_limits<double>::max());
          // find which entrance node is closest to the centroid
          for (size_t i = 0; i < poly_entrances.size(); ++i) {
            const int g = group_of[i];
            const midgard::Point2d centroid(cx[g] / count[g], cy[g] / count[g]);
            const double d = poly_entrances[i].second.DistanceSquared(centroid);
            // if this entrance is the closest to the centroid, update it as representative
            if (d < best[g]) {
              best[g] = d;
              representative[g] = static_cast<int>(i);
            }
          }
          // rebuild the original list keeping only one representative per group
          std::vector<std::pair<uint64_t, midgard::Point2d>> grouped;
          grouped.reserve(num_groups);
          for (int g = 0; g < num_groups; ++g) {
            grouped.push_back(poly_entrances[representative[g]]);
          }
          poly_entrances = std::move(grouped);
        }
      }
      auto medial_axis = GenerateMedialAxis(poly);
      // chain index -> the vertex indices in it where an entrance connects, which
      // must survive the simplification below
      std::unordered_map<size_t, std::unordered_set<size_t>> protected_points;
      const size_t original_chains = medial_axis.size();

      // connect each entrance to the nearest vertex it can reach without leaving the polygon
      for (const auto& [entrance_id, entrance] : poly_entrances) {
        // all skeleton vertices as candidates, sorted by distance to the entrance
        std::vector<std::tuple<double, size_t, size_t>> candidates;
        for (size_t c = 0; c < original_chains; ++c) {
          for (size_t i = 0; i < medial_axis[c].size(); ++i) {
            candidates.emplace_back(medial_axis[c][i].DistanceSquared(entrance), c, i);
          }
        }
        std::sort(candidates.begin(), candidates.end());

        bool connected = false;
        // take the closest one whose connecting segment stays inside the polygon
        for (const auto& [dist_sq, c, i] : candidates) {
          const auto& target = medial_axis[c][i];
          if (!visible_within(prepared_poly, {entrance.x(), entrance.y()}, target)) {
            continue;
          }
          // remember this point must survive simplification
          protected_points[c].insert(i);
          medial_axis.push_back({{entrance.x(), entrance.y()}, target});
          connected = true;
          break;
        }
        if (!connected) {
          LOG_DEBUG("Area entrance " + std::to_string(entrance_id) +
                    " could not see any medial axis vertex, skipping it");
          ++skipped_entrances;
        }
      }

      // simplify each chain, protecting the points where entrances connect
      for (size_t c = 0; c < medial_axis.size(); ++c) {
        std::unordered_set<size_t> keep;
        auto it = protected_points.find(c);
        if (it != protected_points.end()) {
          keep = it->second;
        }
        midgard::Polyline2<midgard::Point2d>::Generalize(medial_axis[c], kSimplifyToleranceMeters,
                                                         keep);
      }
      // rounded position -> node id, pre-seeded with the entrances so that skeleton
      // points landing on one reuse its real OSM id instead of getting a synthetic
      // one. that is what hooks the traversal onto the existing footway
      std::map<std::pair<int64_t, int64_t>, uint64_t> point_ids;
      for (const auto& [entrance_id, entrance] : poly_entrances) {
        point_ids.emplace(std::make_pair(std::llround(entrance.x() * 1000.0),
                                         std::llround(entrance.y() * 1000.0)),
                          entrance_id);
      }

      // build the traversal lines as node id + lat/lon, projecting back as we go
      std::vector<std::vector<std::pair<uint64_t, midgard::PointLL>>> traversal_lines;
      for (const auto& chain : medial_axis) {
        if (chain.size() < 2) {
          continue;
        }
        std::vector<std::pair<uint64_t, midgard::PointLL>> line;
        for (const auto& p : chain) {
          const auto key = std::make_pair(std::llround(p.x() * 1000.0), std::llround(p.y() * 1000.0));
          auto inserted = point_ids.emplace(key, next_synthetic_node_id);
          if (inserted.second) {
            ++next_synthetic_node_id;
          }
          const midgard::PointLL ll = projection.project_inverse(p);
          line.push_back({inserted.first->second, ll});
        }
        traversal_lines.push_back(std::move(line));
      }

      LOG_DEBUG("polygon: " + std::to_string(poly_entrances.size()) + " entrance(s), " +
                std::to_string(traversal_lines.size()) + " traversal line(s)");

      // the entrance nodes are real OSM nodes, everything else we generated
      std::unordered_set<uint64_t> entrance_ids;
      entrance_ids.reserve(poly_entrances.size());
      for (const auto& [entrance_id, entrance] : poly_entrances) {
        entrance_ids.insert(entrance_id);
      }

      auto name_it = area_name_indices.find(area_id);
      const uint32_t name_index = (name_it != area_name_indices.end()) ? name_it->second : 0;

      materialise_traversals(traversal_lines, entrance_ids, name_index, next_synthetic_osm_id, ways,
                             way_nodes);
      generated_any = true;
      GEOSPreparedGeom_destroy(prepared_poly);
    }

    // no traversal was generated for this area (too small), so give its perimeter back
    // to the graph

    // TODO: for now we only give the perimeter back for small areas. Areas skipped
    // for other reasons (no entrances, mapped paths inside) are dropped entirely.
    // Some of those, especially ones with mapped paths, might still want their
    // perimeter routable
    if (!generated_any && restore_perimeter) {
      auto indices_it = area_way_indices.find(area_id);
      if (indices_it != area_way_indices.end()) {
        for (size_t way_index : indices_it->second) {
          auto way = *ways[way_index];
          way.set_area(false);
          way.set_road_class(baldr::RoadClass::kServiceOther);
          way.set_use(baldr::Use::kFootway);
          way.set_speed(kTraversalSpeedKph);
          way.set_pedestrian_forward(true);
          way.set_pedestrian_backward(true);
          ways[way_index] = way;
        }
      }
    }

    // clean up
    for (GEOSGeometry* point : entrance_points) {
      GEOSGeom_destroy(point);
    }
    for (GEOSGeometry* poly : final_polygons) {
      GEOSGeom_destroy(poly);
    }
    if (polygons) {
      GEOSGeom_destroy(polygons);
    }
    for (GEOSGeometry* line : lines) {
      GEOSGeom_destroy(line);
    }
  }

  LOG_INFO(std::to_string(skipped_entrances) +
           " area entrances were skipped (could not connect to a medial axis vertex)");
  finishGEOS();
}
} // namespace mjolnir
} // namespace valhalla
