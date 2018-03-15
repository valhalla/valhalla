#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "loki/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_height(valhalla_request_t& request) {
      //get some parameters
      auto input_shape = rapidjson::get_optional<rapidjson::Value::Array>(request.document, "/shape");
      auto resample_distance = rapidjson::get_optional<double>(request.document, "/resample_distance");

      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape)
            shape.push_back(baldr::Location::FromRapidJson(latlng).latlng_);
        }//compressed shape
        else if (request.options.has_encoded_polyline()) {
          shape = midgard::decode<std::vector<midgard::PointLL> >(request.options.encoded_polyline());
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
          if(request.options.has_encoded_polyline())
            request.options.set_encoded_polyline(midgard::encode(shape));
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
    std::string loki_worker_t::height(valhalla_request_t& request) {
      init_height(request);
      //get the elevation of each posting
      std::vector<double> heights = sample.get_all(shape);
      if (!request.options.do_not_track())
        valhalla::midgard::logging::Log("sample_count::" + std::to_string(shape.size()), " [ANALYTICS] ");

      //get the distances between the postings if desired
      std::vector<float> ranges;
      if (request.options.range()) {
        ranges.reserve(shape.size()); ranges.emplace_back(0);
        for(auto point = std::next(shape.cbegin()); point != shape.cend(); ++point)
          ranges.emplace_back(ranges.back() +  point->Distance(*std::prev(point)));
      }

      return tyr::serializeHeight(request, shape, heights, ranges);
    }
  }
}
