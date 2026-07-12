#include "mjolnir/areabuilder.h"
#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "midgard/sequence.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/osmway.h"
#include "scoped_timer.h"

#include <geos_c.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace valhalla {
namespace mjolnir {

// TODO: make this a config option
constexpr double kMinAreaSquareMeters = 100.0;
constexpr double kDensifyToleranceMeters = 1.0;
constexpr double kSimplifyToleranceMeters = 1.5;

void AreaBuilder::BuildAreas(const boost::property_tree::ptree& /*pt*/,
                             const std::string& ways_file,
                             const std::string& way_nodes_file,
                             OSMData& osmdata) {
  SCOPED_TIMER();

  // way_id > relation_id, to know which relation each area way belongs to
  std::unordered_map<uint64_t, uint64_t> way_to_relation;
  for (const auto& entry : osmdata.area_relations) {
    way_to_relation[entry.second.way_id] = entry.first;
  }

  midgard::sequence<OSMWay> ways(ways_file, false);
  midgard::sequence<OSMWayNode> way_nodes(way_nodes_file, false);

  std::unordered_map<uint64_t, std::vector<std::vector<midgard::PointLL>>> area_ways;

  // grab the way and its first node
  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index =
        first_way_node_index + way.node_count() - way_node.way_shape_node_index - 1;

    // if it is an area, collect and store its shape
    if (way.area()) {
      std::vector<midgard::PointLL> shape;
      for (auto node_idx = first_way_node_index; node_idx <= last_way_node_index; node_idx++) {
        shape.push_back((*way_nodes[node_idx]).node.latlng());
      }
      auto it = way_to_relation.find(way.way_id());
      uint64_t area_key = (it != way_to_relation.end()) ? it->second : way.way_id();
      area_ways[area_key].push_back(std::move(shape));
    }
    current_way_node_index = last_way_node_index + 1;
  }

  initGEOS(nullptr, nullptr);

  for (const auto& [relation_id, shapes] : area_ways) {
    if (shapes.empty() || shapes.front().empty()) {
      continue;
    }
    // reference point for the local projection, first point of the first way
    const midgard::PointLL center = shapes.front().front();
    midgard::DistanceApproximator<midgard::PointLL> aprox(center);
    const double meter_per_lng = aprox.GetMetersPerLngDegree();

    // project each way to meters and create a GEOS line for each one
    std::vector<GEOSGeometry*> lines;
    for (const auto& shape : shapes) {
      if (shape.size() < 2) {
        continue;
      }
      // sequence of meter coords of the nodes of a way
      GEOSCoordSequence* seq = GEOSCoordSeq_create(shape.size(), 2);
      for (size_t i = 0; i < shape.size(); i++) {
        double x = (shape[i].lng() - center.lng()) * meter_per_lng;
        double y = (shape[i].lat() - center.lat()) * midgard::kMetersPerDegreeLat;
        GEOSCoordSeq_setX(seq, i, x);
        GEOSCoordSeq_setY(seq, i, y);
      }
      // create a line with these points
      GEOSGeometry* line = GEOSGeom_createLineString(seq);
      if (line) {
        lines.push_back(line);
      }
    }
    if (lines.empty()) {
      continue;
    }
    // with the lines we created, returns a collection of polygons
    GEOSGeometry* polygons = GEOSPolygonize(lines.data(), lines.size());
    int num_polygons = polygons ? GEOSGetNumGeometries(polygons) : 0;

    // keep the polygons that are not filling a hole of another polygon
    std::vector<GEOSGeometry*> final_polygons;
    for (int i = 0; i < num_polygons; ++i) {
      const GEOSGeometry* poly_i = GEOSGetGeometryN(polygons, i);
      GEOSGeometry* point = GEOSPointOnSurface(poly_i);
      bool fills_a_hole = false;
      for (int j = 0; j < num_polygons; ++j) {
        if (i == j || GEOSGetNumInteriorRings(GEOSGetGeometryN(polygons, j)) == 0) {
          continue;
        }
        const GEOSGeometry* exterior = GEOSGetExteriorRing(GEOSGetGeometryN(polygons, j));
        GEOSGeometry* shell = GEOSGeom_createPolygon(GEOSGeom_clone(exterior), nullptr, 0);
        char inside_shell = GEOSContains(shell, point);
        char inside_poly = GEOSContains(GEOSGetGeometryN(polygons, j), point);
        GEOSGeom_destroy(shell);
        if (inside_shell == 1 && inside_poly == 0) {
          fills_a_hole = true;
          break;
        }
      }
      GEOSGeom_destroy(point);
      if (!fills_a_hole) {
        final_polygons.push_back(GEOSGeom_clone(poly_i));
      }
    }

    fprintf(stderr, "area %lu: %zu ways -> %d raw polygon(s) -> %zu final polygon(s)\n", relation_id,
            lines.size(), num_polygons, final_polygons.size());
    for (size_t o = 0; o < final_polygons.size(); ++o) {
      int n_holes = GEOSGetNumInteriorRings(final_polygons[o]);
      fprintf(stderr, "   polygon %zu has %d hole(s)\n", o, n_holes);
    }

    for (GEOSGeometry* poly : final_polygons) {
      double area = 0;
      GEOSArea(poly, &area);
      // skip areas that are too small to be worth routing through
      if (area < kMinAreaSquareMeters) {
        continue;
      }
      GenerateMedialAxis(poly, center, meter_per_lng);
    }

    // clean up
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

  finishGEOS();
}

void AreaBuilder::GenerateMedialAxis(const GEOSGeometry* polygon,
                                     const midgard::PointLL& /*center*/,
                                     double /*meters_per_lng_degree*/) {

  initGEOS(nullptr, nullptr);
  double area = 0;
  GEOSArea(polygon, &area);
  fprintf(stderr, "generating medial axis for polygon with area %f\n", area);

  // densify the polygon boundary so the voronoi diagram has enough points
  GEOSGeometry* densified = GEOSDensify(polygon, kDensifyToleranceMeters);
  if (!densified) {
    return;
  }

  // count the points before and after to verify the densification, just for testing
  const GEOSGeometry* exterior = GEOSGetExteriorRing(densified);
  int num_points = GEOSGeomGetNumPoints(exterior);
  int n_points = GEOSGeomGetNumPoints(GEOSGetExteriorRing(polygon));
  fprintf(stderr, "medial axis: densified polygon exterior has %d points, and the no densified %d \n",
          num_points, n_points);

  // compute the voronoi diagram using the densified polygon vertices
  GEOSGeometry* voronoi = GEOSVoronoiDiagram(densified, nullptr, 0.0, GEOS_VORONOI_ONLY_EDGES);
  if (!voronoi) {
    GEOSGeom_destroy(densified);
    return;
  }

  int num_voronoi_edges = GEOSGetNumGeometries(voronoi);

  // keep only the voronoi edges that fall completely inside the polygon
  std::vector<GEOSGeometry*> medial_edges;
  for (int i = 0; i < num_voronoi_edges; ++i) {
    const GEOSGeometry* edge = GEOSGetGeometryN(voronoi, i);
    if (GEOSContains(polygon, edge) == 1) {
      medial_edges.push_back(GEOSGeom_clone(edge));
    }
  }

  fprintf(stderr, "medial axis: %zu edges inside the polygon (raw medial axis)\n",
          medial_edges.size());

  // round the coordinates so that vertices that are the same point share a key
  auto vertex_key = [](double x, double y) {
    return std::make_pair(std::llround(x * 1000.0), std::llround(y * 1000.0));
  };

  std::map<std::pair<int64_t, int64_t>, int> vertex_ids;
  std::vector<midgard::Point2d> vertex_coords;

  // lamda function to set and return the id of a vertex from its coords
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

  // the degree of a vertex is the size of the list
  size_t deg1 = 0, deg2 = 0, deg3plus = 0;
  for (const auto& inc : incident) {
    if (inc.size() == 1) {
      deg1++;
    } else if (inc.size() == 2) {
      deg2++;
    } else if (inc.size() >= 3) {
      deg3plus++;
    }
  }
  fprintf(stderr, "medial axis: %zu vertices, %zu segments (deg1=%zu deg2=%zu deg3+=%zu)",
          vertex_ids.size(), segments.size(), deg1, deg2, deg3plus);

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
      if (incident[next_vertex].size() != 2) {
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

  size_t kept = 0;
  for (bool r : removed) {
    if (!r) {
      kept++;
    }
  }
  fprintf(stderr, " after prune removed %zu)\n", segments.size() - kept);

  // stitch the surviving segments into chains, cutting at forks and endpoints
  std::vector<bool> used(segments.size(), false);
  std::vector<std::vector<midgard::Point2d>> chains;

  // a vertex is important if it is not a pass through vertex
  auto is_important = [&](int v) { return incident[v].size() != 2; };

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

  // simplify with douglas peucker to eliminate redundant points
  size_t points_before = 0;
  size_t points_after = 0;
  for (auto& chain : chains) {
    points_before += chain.size();
    midgard::Polyline2<midgard::Point2d>::Generalize(chain, kSimplifyToleranceMeters);
    points_after += chain.size();
  }

  fprintf(stderr, "medial axis: %zu chains from %zu simplified from %zu to %zu points\n",
          chains.size(), kept, points_before, points_after);

  // clean up the kept edges
  for (GEOSGeometry* edge : medial_edges) {
    GEOSGeom_destroy(edge);
  }
  GEOSGeom_destroy(voronoi);
  GEOSGeom_destroy(densified);
  finishGEOS();
}

} // namespace mjolnir
} // namespace valhalla