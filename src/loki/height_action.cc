#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <boost/property_tree/ptree.hpp>

#include "midgard/logging.h"
#include "baldr/location.h"
#include "midgard/util.h"
#include "midgard/encoded.h"
#include "baldr/rapidjson_utils.h"

#include "loki/worker.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;


namespace {

  json::ArrayPtr serialize_range_height(const std::vector<float>& ranges, const std::vector<double>& heights, const double no_data_value) {
  auto array = json::array({});
    //for each posting
    auto range = ranges.cbegin();

    for (const auto height : heights) {
      auto element = json::array({json::fp_t{*range,0}});
      if(height == no_data_value)
        element->push_back(nullptr);
      else {
        element->push_back({json::fp_t{height, 0}});
      }
      array->push_back(element);
      ++range;
    }
    return array;
}

  json::ArrayPtr serialize_height(const std::vector<double>& heights, const double no_data_value) {
    auto array = json::array({});

    for (const auto height : heights) {
      //add all heights's to an array
      if(height == no_data_value)
        array->push_back(nullptr);
      else array->push_back({json::fp_t{height, 0}});
    }

    return array;
  }

  json::ArrayPtr serialize_shape(const std::vector<PointLL>& shape) {
    auto array = json::array({});
    for(const auto& p : shape) {
      array->emplace_back(json::map({
        {"lon", json::fp_t{p.first, 6}},
        {"lat", json::fp_t{p.second, 6}}
      }));
    }
    return array;
  }

}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_height(rapidjson::Document& request) {
      //get some parameters
      range = GetFromRapidJson(request, "/range", false);
      auto input_shape = GetOptionalFromRapidJson<rapidjson::Value::Array>(request, "/shape");
      encoded_polyline = GetOptionalFromRapidJson<std::string>(request, "/encoded_polyline");
      auto resample_distance = GetOptionalFromRapidJson<double>(request, "/resample_distance");

      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape)
            shape.push_back(baldr::Location::FromRapidJson(latlng).latlng_);
        }//compressed shape
        else if (encoded_polyline) {
          shape = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
        }//no shape
        else
          throw valhalla_exception_t{310};

        //not enough shape
        if (shape.size() < 1)
          throw valhalla_exception_t{311};
      }
      catch (...) {
        throw valhalla_exception_t{312};
      }

      //resample the shape
      bool resampled = false;
      if(resample_distance) {
        if(*resample_distance < min_resample)
          throw valhalla_exception_t{313, " " + std::to_string(min_resample) + " meters"};
        if(shape.size() > 1) {
          //resample the shape but make sure to keep the first and last shapepoint
          auto last = shape.back();
          shape = midgard::resample_spherical_polyline(shape, *resample_distance);
          shape.emplace_back(std::move(last));
          //reencode it for display if they sent it encoded
          if(encoded_polyline)
            *encoded_polyline = midgard::encode(shape);
          resampled = true;
        }
      }

      //there are limits though
      if(shape.size() > max_elevation_shape) {
        throw valhalla_exception_t{314, " (" + std::to_string(shape.size()) +
            (resampled ? " after resampling" : "") + "). The limit is " + std::to_string(max_elevation_shape)};
      }
    }

    /* example height with range response:
    {
      "shape": [ {"lat": 40.712433, "lon": -76.504913}, {"lat": 40.712276, "lon": -76.605263} ],
      "range_height": [ [0,303], [8467,275], [25380,198] ]
    }
    */
    json::MapPtr loki_worker_t::height(rapidjson::Document& request) {
      init_height(request);
      //get the elevation of each posting
      std::vector<double> heights = sample.get_all(shape);
      if (!healthcheck)
        valhalla::midgard::logging::Log("sample_count::" + std::to_string(shape.size()), " [ANALYTICS] ");
      boost::optional<std::string> id = GetOptionalFromRapidJson<std::string>(request, "/id");
      auto json = json::map({});

      //get the distances between the postings
      if (range) {
        std::vector<float> ranges; ranges.reserve(shape.size()); ranges.emplace_back(0);
        for(auto point = std::next(shape.cbegin()); point != shape.cend(); ++point)
          ranges.emplace_back(ranges.back() +  point->Distance(*std::prev(point)));
        json = json::map({
          {"range_height", serialize_range_height(ranges, heights, sample.get_no_data_value())}
        });
      }//just the postings
      else {
        json = json::map({
          {"height", serialize_height(heights, sample.get_no_data_value())}
        });
      }
      //send back the shape as well
      if(encoded_polyline)
        json->emplace("encoded_polyline", *encoded_polyline);
      else
        json->emplace("shape", serialize_shape(shape));
      if (id)
        json->emplace("id", *id);

      return json;
    }
  }
}
