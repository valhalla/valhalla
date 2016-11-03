#include "baldr/geojson.h"
#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>

#include <sstream>
#include <iomanip>
#include <cmath>
#include <utility>

namespace {
  using rgba_t = std::tuple<float,float,float>;
}

namespace valhalla {
namespace baldr {
namespace json {

template <class coord_t>
MapPtr to_geojson(const typename midgard::GriddedData<coord_t>::contours_t& grid_contours, const std::vector<std::string>& colors) {
  //for each contour interval
  int i = 0;
  auto color_itr = colors.cbegin();
  auto features = array({});
  for(const auto& contours : grid_contours) {
    //color was supplied
    std::stringstream hex;
    if(color_itr != colors.cend() && !color_itr->empty()) {
      hex << *color_itr;
      color_itr++;
    }//or we computed it..
    else {
      auto h = i * (150.f / grid_contours.size());
      auto c = .5f;
      auto x = c * (1 - std::abs(std::fmod(h/60.f, 2.f) - 1));
      auto m = .25f;
      rgba_t color = h < 60 ? rgba_t{m + c, m + x, m} : (h < 120 ? rgba_t{m + x, m + c, m} : rgba_t{m, m + c, m + x});
      hex << "#" << std::hex << static_cast<int>(std::get<0>(color)*255 + .5f) <<
                    std::hex << static_cast<int>(std::get<1>(color)*255 + .5f) <<
                    std::hex << static_cast<int>(std::get<2>(color)*255 + .5f);
    }
    ++i;

    //for each contour on that interval
    for(const auto& contour : contours.second) {
      //make some geometry
      auto coords = array({});
      for(const auto& coord : contour)
        coords->push_back(array({fp_t{coord.first, 6}, fp_t{coord.second, 6}}));
      auto polygon = contour.front() == contour.back();
      if(polygon)
        coords = array({coords});
      //add a feature
      features->emplace_back(
        map({
          {"type", std::string("Feature")},
          {"geometry", map({
            {"type", std::string(polygon ? "Polygon" : "LineString")},
            {"coordinates", coords},
          })},
          {"properties", map({
            {"contour", static_cast<uint64_t>(contours.first)},
            { polygon ? "fill" : "color", hex.str()},
            { polygon ? "fill-opacity" : "opacity", json::fp_t{.33f, 2}},
          })},
        })
      );
    }
  }
  //make the collection
  auto feature_collection = map({
    {"type", std::string("FeatureCollection")},
    {"features", features},
  });
  return feature_collection;
}

template MapPtr to_geojson<midgard::Point2>(const midgard::GriddedData<midgard::Point2>::contours_t&, const std::vector<std::string>&);
template MapPtr to_geojson<midgard::PointLL>(const midgard::GriddedData<midgard::PointLL>::contours_t&, const std::vector<std::string>&);

}
}
}
