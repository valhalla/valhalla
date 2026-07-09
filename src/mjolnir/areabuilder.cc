#include "mjolnir/areabuilder.h"
#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/osmway.h"
#include "scoped_timer.h"

#include <geos_c.h>

#include <unordered_map>
#include <vector>

namespace valhalla {
namespace mjolnir {

constexpr double kMinAreaSquareMeters = 100.0;
constexpr double kDensifyToleranceMeters = 1.0;

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
                                     const midgard::PointLL& center,
                                     double meters_per_lng_degree) {

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

  // TODO: prune, simplify

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