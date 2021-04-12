
#include "baldr/json.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "tyr/serializers.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

using namespace valhalla::baldr::json;

namespace {
using rgba_t = std::tuple<float, float, float>;
}

namespace valhalla {
namespace tyr {


bool intersects(const midgard::GeoPoint<double>& a,
                const midgard::GeoPoint<double>& b,
                const midgard::GeoPoint<double>& c,
                const midgard::GeoPoint<double>& d) {
  double xa = a.lng(), xb = b.lng(), xc = c.lng(), xd = d.lng();
  double ya = a.lat(), yb = b.lat(), yc = c.lat(), yd = d.lat();

  double denom = (xd-xc)*(yb-ya) - (yd-yc)*(xb-xa);

  constexpr double tol = 1e-20;

  // parallel check
  if (fabs(denom) < tol)
    return false;

  double t0 = (yd-yc)*(xa-xc) - (xd-xc)*(ya-yc);
  t0 = t0 / denom;

  if ((t0 < tol) || (t0 > (1-tol)))
    return false;

  double t1 = (xb-xa)*(yc-ya) - (yb-ya)*(xc-xa);
  t1 = t1 / denom;

  if ((t1 < tol) || (t1 > (1-tol)))
    return false;

  return true;
}

double dist(const midgard::PointLL & a, const midgard::PointLL & b) {
  double alat_rad = a.lat() * M_PI / 180.0;
  double alon_rad = a.lng() * M_PI / 180.0;
  double blat_rad = b.lat() * M_PI / 180.0;
  double blon_rad = b.lng() * M_PI / 180.0;

  double dlat = (blat_rad - alat_rad);
  double dlon = (blon_rad - alon_rad);

  double w = sin(dlat/2)*sin(dlat/2) + sin(dlon/2)*sin(dlon/2) * cos(alat_rad) * cos(blat_rad);
  double earth_radius = 6378160.187;
  double c = 2 * asin(sqrt(w));
  return earth_radius * c;
}

std::tuple<int, int> self_intersects(const std::vector<midgard::GeoPoint<double>>& points) {
  int intersections_checked = 0;
  int intersections = 0;
  for (size_t i = 1; i < points.size(); i++) {
    const midgard::GeoPoint<double>& ia(points[i-1]);
    const midgard::GeoPoint<double>& ib(points[i]);
//    if (dist(midgard::PointLL(-117.1959639, 33.7699379), ia) < 1000) {
//      printf("{ %.8f, %.8f },\n", ia.lng(), ia.lat());
//    }
    for (size_t j = i + 1; j < points.size(); j++) {
      const midgard::GeoPoint<double>& ja(points[j-1]);
      const midgard::GeoPoint<double>& jb(points[j]);
      intersections_checked++;
      if (intersects(ia, ib, ja, jb)) {
        intersections++;
        printf("Intersection:\n");
        printf("Line: (%.7f, %.7f), (%.7f, %.7f)\n", ia.lat(), ia.lng(), ib.lat(), ib.lng());
        printf("Line: (%.7f, %.7f), (%.7f, %.7f)\n", ja.lat(), ja.lng(), jb.lat(), jb.lng());
      }
    }
  }

  return std::make_tuple(intersections_checked, intersections);
}

bool check_for_intersections(std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                             midgard::GriddedData<2>::contours_t& contours) {

  clock_t start_time = clock();

  int polygons_checked = 0;
  int self_intersecting_polygons = 0;
  int total_intersections_checked = 0;
  int total_intersections = 0;

  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& feature_collection = contours[contour_index];
    for (const auto& feature : feature_collection) {
      for (const auto& contour : feature) {
        std::vector<midgard::GeoPoint<double>> points;
        for (const auto& coord : contour) {
          points.emplace_back(coord);
        }
        polygons_checked++;
        int intersections_checked = 0;
        int intersections = 0;
        std::tie(intersections_checked, intersections) = self_intersects(points);
        total_intersections_checked += intersections_checked;
        total_intersections += intersections;
        if (intersections)
          self_intersecting_polygons++;
      }
    }
  }

  printf("Total intersections checked: %d, total intersections: %d\n", total_intersections_checked, total_intersections);
  printf("Polygons checked: %d, num self-intersecting: %d\n", polygons_checked, self_intersecting_polygons);

  return self_intersecting_polygons > 0;
}

std::string serializeIsochrones(const Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                midgard::GriddedData<2>::contours_t& contours,
                                bool polygons,
                                bool show_locations) {

  check_for_intersections(intervals, contours);

  // for each contour interval
  int i = 0;
  auto features = array({});
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& feature_collection = contours[contour_index];

    // color was supplied
    std::stringstream hex;
    if (!std::get<3>(interval).empty()) {
      hex << "#" << std::get<3>(interval);
    } // or we computed it..
    else {
      auto h = i * (150.f / intervals.size());
      auto c = .5f;
      auto x = c * (1 - std::abs(std::fmod(h / 60.f, 2.f) - 1));
      auto m = .25f;
      rgba_t color = h < 60 ? rgba_t{m + c, m + x, m}
                            : (h < 120 ? rgba_t{m + x, m + c, m} : rgba_t{m, m + c, m + x});
      hex << "#" << std::hex << static_cast<int>(std::get<0>(color) * 255 + .5f) << std::hex
          << static_cast<int>(std::get<1>(color) * 255 + .5f) << std::hex
          << static_cast<int>(std::get<2>(color) * 255 + .5f);
    }
    ++i;

    // for each feature on that interval
    for (const auto& feature : feature_collection) {
      // for each contour in that feature
      auto geom = array({});
      for (const auto& contour : feature) {
        // make some geometry
        auto coords = array({});
        for (const auto& coord : contour) {
          coords->push_back(array({fp_t{coord.first, 6}, fp_t{coord.second, 6}}));
        }
        // its either a ring
        if (polygons) {
          geom->emplace_back(coords);
          // or a single line, if someone has more than one contour per feature they messed up
        } else {
          geom = coords;
        }
      }
      // add a feature
      features->emplace_back(map({
          {"type", std::string("Feature")},
          {"geometry", map({
                           {"type", std::string(polygons ? "Polygon" : "LineString")},
                           {"coordinates", geom},
                       })},
          {"properties", map({
                             {"metric", std::get<2>(interval)},
                             {"contour", static_cast<uint64_t>(std::get<1>(interval))},
                             {"color", hex.str()},            // lines
                             {"fill", hex.str()},             // geojson.io polys
                             {"fillColor", hex.str()},        // leaflet polys
                             {"opacity", fp_t{.33f, 2}},      // lines
                             {"fill-opacity", fp_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fp_t{.33f, 2}},  // leaflet polys
                         })},
      }));
    }
  }
  // Add input and snapped locations to the geojson
  if (show_locations) {
    int idx = 0;
    for (const auto& location : request.options().locations()) {
      // first add all snapped points as MultiPoint feature per origin point
      auto snapped_points_array = array({});
      std::unordered_set<midgard::PointLL> snapped_points;
      for (const auto& path_edge : location.path_edges()) {
        const midgard::PointLL& snapped_current =
            midgard::PointLL(path_edge.ll().lng(), path_edge.ll().lat());
        // remove duplicates of path_edges in case the snapped object is a node
        if (snapped_points.insert(snapped_current).second) {
          snapped_points_array->push_back(
              array({fp_t{snapped_current.lng(), 6}, fp_t{snapped_current.lat(), 6}}));
        }
      };
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("snapped")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry",
            map({{"type", std::string("MultiPoint")}, {"coordinates", snapped_points_array}})}}));

      // then each user input point as separate Point feature
      const valhalla::LatLng& input_latlng = location.ll();
      const auto input_array = array({fp_t{input_latlng.lng(), 6}, fp_t{input_latlng.lat(), 6}});
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("input")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry", map({{"type", std::string("Point")}, {"coordinates", input_array}})}}));
      idx++;
    }
  }

  // make the collection
  auto feature_collection = map({
      {"type", std::string("FeatureCollection")},
      {"features", features},
  });

  if (request.options().has_id()) {
    feature_collection->emplace("id", request.options().id());
  }

  std::stringstream ss;
  ss << *feature_collection;
  return ss.str();
}
} // namespace tyr
} // namespace valhalla
