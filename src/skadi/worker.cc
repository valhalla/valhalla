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

#include "skadi/worker.h"
#include "skadi/sample.h"
#include "tyr/actor.h"

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
  namespace skadi {

    skadi_worker_t::skadi_worker_t (const boost::property_tree::ptree& config):
      sample(config.get<std::string>("additional_data.elevation", "test/data/")), range(false),
      max_shape(config.get<size_t>("service_limits.skadi.max_shape")), min_resample(config.get<float>("service_limits.skadi.min_resample")),
      long_request(config.get<float>("skadi.logging.long_request")), healthcheck(false), action_str("'height'"){
    }

    skadi_worker_t::~skadi_worker_t(){}


    void skadi_worker_t::init_request(rapidjson::Document& request) {
      //flag healthcheck requests; do not send to logstash
      healthcheck = GetFromRapidJson(request,"/healthcheck", false);
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
      if(shape.size() > max_shape) {
        throw valhalla_exception_t{314, " (" + std::to_string(shape.size()) +
            (resampled ? " after resampling" : "") + "). The limit is " + std::to_string(max_shape)};
      }
    }

    void skadi_worker_t::cleanup() {
      shape.clear();
      encoded_polyline.reset();
    }

    /* example height with range response:
    {
      "shape": [ {"lat": 40.712433, "lon": -76.504913}, {"lat": 40.712276, "lon": -76.605263} ],
      "range_height": [ [0,303], [8467,275], [25380,198] ]
    }
    */
    json::MapPtr skadi_worker_t::height(rapidjson::Document& request) {
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

#ifdef HAVE_HTTP
    worker_t::result_t skadi_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt_function) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Skadi Request " + std::to_string(info.id));
      boost::optional<std::string> jsonp;
      //request should look like:
      //  http://host:port/height?json={shape:[{lat: ,lon: }],range=false} OR http://host:port/height?json={encoded_polyline:"SOMESTUFF",range:true}
      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        auto action = PATH_TO_ACTION.find(request.path);
        if(action == PATH_TO_ACTION.cend())
          return jsonify_error(valhalla_exception_t{304, action_str}, info, jsonp);

        // Set the interrupt function
        service_worker_t::set_interrupt(interrupt_function);

        //parse the query's json
        auto request_rj = from_request(request);
        jsonp = GetOptionalFromRapidJson<std::string>(request_rj, "/jsonp");
        init_request(request_rj);
        worker_t::result_t result;
        switch (action->second) {
          case HEIGHT:
            result = to_response(height(request_rj), jsonp, info);
            break;
          default:
            return jsonify_error({305}, info, jsonp);
        }
        //get processing time for skadi
        auto e = std::chrono::system_clock::now();
        std::chrono::duration<float, std::milli> elapsed_time = e - s;
        //log request if greater than X (ms)
        auto do_not_track = request.headers.find("DNT");
        bool allow_tracking = do_not_track == request.headers.cend() || do_not_track->second != "1";
        if (!healthcheck && allow_tracking && (elapsed_time.count() / shape.size()) > long_request) {
          LOG_WARN("skadi::request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
          LOG_WARN("skadi::request exceeded threshold::"+ rapidjson::to_string(request_rj));
          midgard::logging::Log("valhalla_skadi_long_request", " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error(e, info, jsonp);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({399, std::string(e.what())}, info, jsonp);
      }
    }

    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from the http server
      auto upstream_endpoint = config.get<std::string>("skadi.service.proxy") + "_out";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
      auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

      //listen for requests
      zmq::context_t context;
      skadi_worker_t skadi_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, "ipc://TODO", loopback_endpoint, interrupt_endpoint,
        std::bind(&skadi_worker_t::work, std::ref(skadi_worker), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        std::bind(&skadi_worker_t::cleanup, std::ref(skadi_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
#endif

  }
}
