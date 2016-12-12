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
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/midgard/util.h>
#include <valhalla/midgard/encoded.h>

#include "skadi/service.h"
#include "skadi/sample.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;


namespace {

  const std::unordered_map<std::string, skadi_worker_t::ACTION_TYPE> ACTION{
    {"/height", skadi_worker_t::HEIGHT}
  };

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  boost::property_tree::ptree from_request(const skadi_worker_t::ACTION_TYPE& action, const http_request_t& request) {
    boost::property_tree::ptree pt;

    //throw the json into the ptree
    try {
      auto json = request.query.find("json");
      if(json != request.query.end() && json->second.size()) {
        std::istringstream is(json->second.front());
        boost::property_tree::read_json(is, pt);
      }//no json parameter, check the body
      else if(!request.body.empty()) {
        std::istringstream is(request.body);
        boost::property_tree::read_json(is, pt);
      }
    }
    catch(...) {
      throw valhalla_exception_t{400, 300};
    }

    //throw the query params into the ptree
    for(const auto& kv : request.query) {
      //skip json or empty entries
      if(kv.first == "json" || kv.first.size() == 0 || kv.second.size() == 0)
        continue;

      //turn single value entries into single key value
      if(kv.second.size() == 1) {
        pt.add(kv.first, kv.second.front());
        continue;
      }

      //make an array of values for this key
      boost::property_tree::ptree array;
      for(const auto& value : kv.second) {
        boost::property_tree::ptree element;
        element.put("", value);
        array.push_back(std::make_pair("", element));
      }
      pt.add_child(kv.first, array);
    }

    return pt;
  }

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

  json::ArrayPtr serialize_shape(const std::list<PointLL>& shape) {
    auto array = json::array({});
    for(const auto& p : shape) {
      array->emplace_back(json::map({
        {"lon", json::fp_t{p.first, 6}},
        {"lat", json::fp_t{p.second, 6}}
      }));
    }
    return array;
  }

  worker_t::result_t jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info, const boost::optional<std::string>& jsonp) {

    //build up the json map
    auto json_error = json::map({});
    json_error->emplace("status", exception.status_code_body);
    json_error->emplace("status_code", static_cast<uint64_t>(exception.status_code));
    json_error->emplace("error", std::string(exception.error_code_message));
    json_error->emplace("error_code", static_cast<uint64_t>(exception.error_code));

    //serialize it
    std::stringstream ss;
    if(jsonp)
      ss << *jsonp << '(';
    ss << *json_error;
    if(jsonp)
      ss << ')';

    worker_t::result_t result{false};
    http_response_t response(exception.status_code, exception.status_code_body, ss.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());

    return result;
  }
}

namespace valhalla {
  namespace skadi {

    skadi_worker_t::skadi_worker_t (const boost::property_tree::ptree& config):
      sample(config.get<std::string>("additional_data.elevation", "test/data/")), range(false),
      max_shape(config.get<size_t>("service_limits.skadi.max_shape")), min_resample(config.get<float>("service_limits.skadi.min_resample")),
      long_request(config.get<float>("skadi.logging.long_request")){
    }

    skadi_worker_t::~skadi_worker_t(){}

    worker_t::result_t skadi_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const worker_t::interrupt_function_t&) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Skadi Request " + std::to_string(info.id));
      //request should look like:
      //  http://host:port/height?json={shape:[{lat: ,lon: }],range=false} OR http://host:port/height?json={encoded_polyline:"SOMESTUFF",range:true}
      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

        //block all but get and post
        if(request.method != method_t::POST && request.method != method_t::GET)
          return jsonify_error({405, 301}, info, jsonp);

        auto action = ACTION.find(request.path);
        if(action == ACTION.cend())
          return jsonify_error({404, 304, action->first}, info, jsonp);

        //parse the query's json
        auto request_pt = from_request(action->second, request);
        jsonp = request_pt.get_optional<std::string>("jsonp");
        init_request(action->second, request_pt);
        worker_t::result_t result{false};
        switch (action->second) {
          case HEIGHT:
            result = elevation(request_pt, info);
            break;
          default:
            return jsonify_error({501, 305}, info, jsonp);
        }
        //get processing time for skadi
        auto e = std::chrono::system_clock::now();
        std::chrono::duration<float, std::milli> elapsed_time = e - s;
        //log request if greater than X (ms)
        auto do_not_track = request.headers.find("DNT");
        bool allow_tracking = do_not_track == request.headers.cend() || do_not_track->second != "1";
        if (allow_tracking && (elapsed_time.count() / shape.size()) > long_request) {
          std::stringstream ss;
          boost::property_tree::json_parser::write_json(ss, request_pt, false);
          LOG_WARN("skadi::request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
          LOG_WARN("skadi::request exceeded threshold::"+ ss.str());
          midgard::logging::Log("valhalla_skadi_long_request", " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({e.status_code, e.error_code, e.extra}, info, jsonp);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({400, 399, std::string(e.what())}, info, jsonp);
      }
    }

    void skadi_worker_t::cleanup() {
      jsonp = boost::none;
      shape.clear();
      encoded_polyline.reset();
    }

    void skadi_worker_t::init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request) {
      //get some parameters
      range = request.get<bool>("range", false);
      auto input_shape = request.get_child_optional("shape");
      encoded_polyline = request.get_optional<std::string>("encoded_polyline");
      auto resample_distance = request.get_optional<double>("resample_distance");

      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape)
            shape.push_back(baldr::Location::FromPtree(latlng.second).latlng_);
        }//compressed shape
        else if (encoded_polyline) {
          shape = midgard::decode<std::list<midgard::PointLL> >(*encoded_polyline);
        }//no shape
        else
          throw valhalla_exception_t{400, 310};

        //not enough shape
        if (shape.size() < 1)
          throw valhalla_exception_t{400, 311};
      }
      catch (...) {
        throw valhalla_exception_t{400, 312};
      }

      //resample the shape
      bool resampled = false;
      if(resample_distance) {
        if(*resample_distance < min_resample)
          throw valhalla_exception_t{400, 313, " " + std::to_string(min_resample) + " meters"};
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
        throw valhalla_exception_t{400, 314, " (" + std::to_string(shape.size()) +
            (resampled ? " after resampling" : "") + "). The limit is " + std::to_string(max_shape)};
      }
    }

    /* example height with range response:
    {
      "shape": [ {"lat": 40.712433, "lon": -76.504913}, {"lat": 40.712276, "lon": -76.605263} ],
      "range_height": [ [0,303], [8467,275], [25380,198] ]
    }
    */
    worker_t::result_t skadi_worker_t::elevation(const boost::property_tree::ptree& request, http_request_info_t& request_info) {
      //get the elevation of each posting
      std::vector<double> heights = sample.get_all(shape);
      valhalla::midgard::logging::Log("sample_count::" + std::to_string(shape.size()), " [ANALYTICS] ");
      boost::optional<std::string> id = request.get_optional<std::string>("id");
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

      //jsonp callback if need be
      std::ostringstream stream;
      auto jsonp = request.get_optional<std::string>("jsonp");
      if(jsonp)
        stream << *jsonp << '(';
      stream << *json;
      if(jsonp)
        stream << ')';

      worker_t::result_t result{false};
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());

      return result;
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
  }
}
