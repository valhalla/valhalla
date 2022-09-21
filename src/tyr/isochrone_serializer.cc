
#include "baldr/json.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "tyr/serializers.h"

#include <cmath>
#include <sstream>
#include <utility>

using namespace valhalla::baldr::json;

namespace {
using rgba_t = std::tuple<float, float, float>;
}

namespace valhalla {
namespace tyr {

std::string serializeIsochrones(const Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                midgard::GriddedData<2>::contours_t& contours,
                                bool polygons,
                                bool show_locations) {
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
          coords->push_back(array({fixed_t{coord.first, 6}, fixed_t{coord.second, 6}}));
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
                             {"contour", baldr::json::float_t{std::get<1>(interval)}},
                             {"color", hex.str()},               // lines
                             {"fill", hex.str()},                // geojson.io polys
                             {"fillColor", hex.str()},           // leaflet polys
                             {"opacity", fixed_t{.33f, 2}},      // lines
                             {"fill-opacity", fixed_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fixed_t{.33f, 2}},  // leaflet polys
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
      for (const auto& path_edge : location.correlation().edges()) {
        const midgard::PointLL& snapped_current =
            midgard::PointLL(path_edge.ll().lng(), path_edge.ll().lat());
        // remove duplicates of path_edges in case the snapped object is a node
        if (snapped_points.insert(snapped_current).second) {
          snapped_points_array->push_back(
              array({fixed_t{snapped_current.lng(), 6}, fixed_t{snapped_current.lat(), 6}}));
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
      const auto input_array =
          array({fixed_t{input_latlng.lng(), 6}, fixed_t{input_latlng.lat(), 6}});
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

  if (request.options().has_id_case()) {
    feature_collection->emplace("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    feature_collection->emplace("warnings", serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *feature_collection;

  return ss.str();
}
} // namespace tyr
} // namespace valhalla
