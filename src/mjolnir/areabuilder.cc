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
#include <unordered_set>
#include <utility>
#include <vector>

namespace valhalla {
namespace mjolnir {

constexpr double kMinAreaSquareMeters = 100.0;
constexpr double kDensifyToleranceMeters = 1.0;
constexpr double kSimplifyToleranceMeters = 1.5;
constexpr double kEntranceToleranceMeters = 0.5;

void AreaBuilder::BuildAreas(const boost::property_tree::ptree& pt,
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
  std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, midgard::PointLL>>> area_shared_nodes;
  std::unordered_set<uint64_t> pedestrian_nodes;

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
      auto it = way_to_relation.find(way.way_id());
      uint64_t area_key = (it != way_to_relation.end()) ? it->second : way.way_id();

      std::vector<midgard::PointLL> shape;
      for (auto node_idx = first_way_node_index; node_idx <= last_way_node_index; node_idx++) {
        const auto& node = (*way_nodes[node_idx]).node;
        shape.push_back(node.latlng());
        if (node.intersection()) {
          area_shared_nodes[area_key].push_back({node.osmid_, node.latlng()});
        }
      }
      area_ways[area_key].push_back(std::move(shape));

    } else if (way.pedestrian_forward() || way.pedestrian_backward()) {
      for (auto node_idx = first_way_node_index; node_idx <= last_way_node_index; node_idx++) {
        pedestrian_nodes.insert((*way_nodes[node_idx]).node.osmid_);
      }
    }
    current_way_node_index = last_way_node_index + 1;
  }

  initGEOS(nullptr, nullptr);

  for (const auto& [relation_id, shapes] : area_ways) {
    if (shapes.empty() || shapes.front().empty()) {
      continue;
    }

    std::vector<midgard::PointLL> entrances;
    std::unordered_set<uint64_t> seen_entrances;
    auto shared_it = area_shared_nodes.find(relation_id);
    if (shared_it != area_shared_nodes.end()) {
      for (const auto& [node_id, ll] : shared_it->second) {
        if (pedestrian_nodes.count(node_id) && seen_entrances.insert(node_id).second) {
          entrances.push_back(ll);
        }
      }
    }
    if (entrances.empty()) {
      continue;
    }

    // reference point for the local projection, first point of the first way
    const midgard::PointLL center = shapes.front().front();
    midgard::DistanceApproximator<midgard::PointLL> aprox(center);
    const double meter_per_lng = aprox.GetMetersPerLngDegree();

    // project the entrances to meters and build a GEOS point for each one
    std::vector<midgard::Point2d> entrances_m;
    std::vector<GEOSGeometry*> entrance_points;
    for (const auto& entrance : entrances) {
      const double x = (entrance.lng() - center.lng()) * meter_per_lng;
      const double y = (entrance.lat() - center.lat()) * midgard::kMetersPerDegreeLat;
      entrances_m.push_back({x, y});
      entrance_points.push_back(GEOSGeom_createPointFromXY(x, y));
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
      for (GEOSGeometry* point : entrance_points) {
        GEOSGeom_destroy(point);
      }
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

    for (GEOSGeometry* poly : final_polygons) {
      double area = 0;
      GEOSArea(poly, &area);
      // skip areas that are too small to be worth routing through
      if (area < kMinAreaSquareMeters) {
        continue;
      }

      // a relation can hold several polygons, so keep the entrances lying on this one
      std::vector<midgard::Point2d> poly_entrances;
      for (size_t i = 0; i < entrance_points.size(); ++i) {
        double distance = 0;
        GEOSDistance(poly, entrance_points[i], &distance);
        if (distance < kEntranceToleranceMeters) {
          poly_entrances.push_back(entrances_m[i]);
        }
      }

      fprintf(stderr, "   polygon with %d hole(s) and %zu of the %zu entrance(s)\n",
              GEOSGetNumInteriorRings(poly), poly_entrances.size(), entrances_m.size());

      // an area nobody can reach is not worth generating
      if (poly_entrances.empty()) {
        continue;
      }

      auto medial_axis = GenerateMedialAxis(poly);
      std::unordered_map<size_t, std::unordered_set<size_t>> protected_points;
      const size_t original_chains = medial_axis.size();

      // connect each entrance to the nearest existing vertex of the medial axis
      for (const auto& entrance : poly_entrances) {
        double best_dist_sq = std::numeric_limits<double>::max();
        size_t best_chain = 0;
        size_t best_index = 0;
        for (size_t c = 0; c < original_chains; ++c) {
          for (size_t i = 0; i < medial_axis[c].size(); ++i) {
            const auto& p = medial_axis[c][i];
            const double dx = p.x() - entrance.x();
            const double dy = p.y() - entrance.y();
            const double d = dx * dx + dy * dy;
            if (d < best_dist_sq) {
              best_dist_sq = d;
              best_chain = c;
              best_index = i;
            }
          }
        }
        const auto& target = medial_axis[best_chain][best_index];
        // remember this point must survive simplification
        protected_points[best_chain].insert(best_index);
        medial_axis.push_back({{entrance.x(), entrance.y()}, target});
      }

      fprintf(stderr, "   %zu chains after connecting %zu entrances\n", medial_axis.size(),
              poly_entrances.size());

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

      size_t total_points = 0;
      for (const auto& chain : medial_axis) {
        total_points += chain.size();
      }
      fprintf(stderr, "   simplified to %zu chains, %zu points total\n", medial_axis.size(),
              total_points);
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

  finishGEOS();
}

std::vector<std::vector<midgard::Point2d>>
AreaBuilder::GenerateMedialAxis(const GEOSGeometry* polygon) {

  double area = 0;
  GEOSArea(polygon, &area);
  fprintf(stderr, "generating medial axis for polygon with area %f\n", area);

  // densify the polygon boundary so the voronoi diagram has enough points
  GEOSGeometry* densified = GEOSDensify(polygon, kDensifyToleranceMeters);
  if (!densified) {
    return {};
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
    return {};
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

  // clean up the kept edges
  for (GEOSGeometry* edge : medial_edges) {
    GEOSGeom_destroy(edge);
  }
  GEOSGeom_destroy(voronoi);
  GEOSGeom_destroy(densified);
  return chains;
}

} // namespace mjolnir
} // namespace valhalla