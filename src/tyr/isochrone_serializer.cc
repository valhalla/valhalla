
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

template <class coord_t>
std::string
serializeIsochrones(const Api& request,
                    const typename midgard::GriddedData<coord_t>::contours_t& grid_contours,
                    bool polygons,
                    const std::unordered_map<float, std::string>& colors,
                    bool show_locations) {
  // for each contour interval
  int i = 0;
  auto features = valhalla::baldr::json::array({});
  for (const auto& interval : grid_contours) {
    auto color_itr = colors.find(interval.first);
    // color was supplied
    std::stringstream hex;
    if (color_itr != colors.end() && !color_itr->second.empty()) {
      hex << "#" << color_itr->second;
    } // or we computed it..
    else {
      auto h = i * (150.f / grid_contours.size());
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
    for (const auto& feature : interval.second) {
      // for each contour in that feature
      auto geom = valhalla::baldr::json::array({});
      for (const auto& contour : feature) {
        // make some geometry
        auto coords = valhalla::baldr::json::array({});
        for (const auto& coord : contour) {
          coords->push_back(
              valhalla::baldr::json::array({fp_t{coord.first, 6}, fp_t{coord.second, 6}}));
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
      features->emplace_back(valhalla::baldr::json::map({
          {"type", std::string("Feature")},
          {"geometry", valhalla::baldr::json::map({
                           {"type", std::string(polygons ? "Polygon" : "LineString")},
                           {"coordinates", geom},
                       })},
          {"properties", valhalla::baldr::json::map({
                             {"contour", static_cast<uint64_t>(interval.first)},
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
      auto snapped_points_array = valhalla::baldr::json::array({});
      std::unordered_set<midgard::PointLL> snapped_points;
      for (const auto& path_edge : location.path_edges()) {
        const midgard::PointLL& snapped_current =
            midgard::PointLL(path_edge.ll().lng(), path_edge.ll().lat());
        // remove duplicates of path_edges in case the snapped object is a node
        if (snapped_points.insert(snapped_current).second) {
          snapped_points_array->push_back(valhalla::baldr::json::array(
              {fp_t{snapped_current.lng(), 6}, fp_t{snapped_current.lat(), 6}}));
        }
      };
      features->emplace_back(valhalla::baldr::json::map(
          {{"type", std::string("Feature")},
           {"properties",
            valhalla::baldr::json::map(
                {{"type", std::string("snapped")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry", valhalla::baldr::json::map({{"type", std::string("MultiPoint")},
                                                    {"coordinates", snapped_points_array}})}}));

      // then each user input point as separate Point feature
      const valhalla::LatLng& input_latlng = location.ll();
      const auto input_array =
          valhalla::baldr::json::array({fp_t{input_latlng.lng(), 6}, fp_t{input_latlng.lat(), 6}});
      features->emplace_back(valhalla::baldr::json::map(
          {{"type", std::string("Feature")},
           {"properties",
            valhalla::baldr::json::map(
                {{"type", std::string("input")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry", valhalla::baldr::json::map(
                            {{"type", std::string("Point")}, {"coordinates", input_array}})}}));
      idx++;
    }
  }

  // make the collection
  auto feature_collection = valhalla::baldr::json::map({
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

template std::string
serializeIsochrones<midgard::Point2>(const Api&,
                                     const midgard::GriddedData<midgard::Point2>::contours_t&,
                                     bool,
                                     const std::unordered_map<float, std::string>&,
                                     bool);
template std::string
serializeIsochrones<midgard::PointLL>(const Api&,
                                      const midgard::GriddedData<midgard::PointLL>::contours_t&,
                                      bool,
                                      const std::unordered_map<float, std::string>&,
                                      bool);

} // namespace tyr
} // namespace valhalla
