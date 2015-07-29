#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/edgeinfo.h>

#include "skadi/service.h"
#include "skadi/sample.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;


namespace {
  enum ACTION_TYPE {ELEVATION, PROFILE, VERSION};
  const std::unordered_map<std::string, ACTION_TYPE> ACTION{
    {"/elevation", ELEVATION},
    {"/profile", PROFILE}
  };
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  boost::property_tree::ptree from_request(const ACTION_TYPE& action, const http_request_t& request) {
    boost::property_tree::ptree pt;
    //throw the json into the ptree
    auto json = request.query.find("json");
    if(json != request.query.end() && json->second.size()) {
      std::istringstream is(json->second.front());
      boost::property_tree::read_json(is, pt);
    }//no json parameter, check the body
    else if(!request.body.empty()) {
      std::istringstream is(request.body);
      boost::property_tree::read_json(is, pt);
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

  //TODO: throw this in the header to make it testable?
  class skadi_worker_t {
    public:
    skadi_worker_t(const boost::property_tree::ptree& config):
      sample(config.get<std::string>("additional_data.elevation", "test/data/appalachian.vrt")) {
    }

    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Skadi Request " + std::to_string(info.id));
      //request should look like:
      //  http://host:port/profile?json={shape:[{lat: ,lon: }]}&apikey= OR http://host:port/profile?json={encoded_polyline:" xxxx "}&apikey=
      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        auto action = ACTION.find(request.path);
        if(action == ACTION.cend()) {
          worker_t::result_t result{false};
          http_response_t response(404, "Not Found", "Try any of: '/profile' '/elevation'", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }
        //parse the query's json
        auto request_pt = from_request(action->second, request);
        init_request(action->second, request_pt);
        switch (action->second) {
          case PROFILE:
            return elevation(request_pt, action->second, info);
          case ELEVATION:
            return elevation(request_pt, action->second, info);
        }
        worker_t::result_t result{false};
        http_response_t response(501, "Not Implemented", "", headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
      catch(const std::exception& e) {
        LOG_INFO(std::string("Bad Request: ") + e.what());
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what(), headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
    }

    void init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request) {
      //we require shape or encoded polyline
      try {
       //uncompressed shape
       auto input_shape = request.get_child_optional("shape");
       if (input_shape) {
         for(const auto& latlng : *input_shape)
           shape.push_back(baldr::Location::FromPtree(latlng.second).latlng_);

         if(shape.size() < 1)
           throw std::runtime_error("Insufficient shape provided");
         return;
       }
       //compressed shape
       encoded_polyline = request.get_optional<std::string>("encoded_polyline");
       if (encoded_polyline) {
         shape = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
         return;
       }
     }
     catch(...) {
       throw std::runtime_error("Insufficiently specified required parameter '" + std::string(action == PROFILE ? "latlng'" : "shape'"));
     }
     //you forgot something
     throw std::runtime_error("Insufficient shape provided");
   }

  //example profile response:
  /*
  {
    "input_shape":
  [
    {
        "lat": 40.712433,
        "lon": -76.504913
    },
    {
        "lat": 40.712276,
        "lon": -76.605263
    },
    {
        "lat": 40.712124,
        "lon": -76.805695
    }
  ],
  "profile": [
    [
        0,
        303
    ],
    [
        8467,
        275
    ],
    [
        25380,
        198
    ]
  ]
}*/

    worker_t::result_t elevation(const boost::property_tree::ptree& request, const ACTION_TYPE& action, http_request_t::info_t& request_info) {

      //get the distances between the postings
      std::vector<float> ranges; ranges.reserve(shape.size()); ranges.emplace_back(0);
      for(auto point = shape.cbegin() + 1; point != shape.cend(); ++point){
        ranges.emplace_back(ranges.back() +  point->Distance(*(point - 1)));
      }
      //get the elevation of each posting
      std::vector<double> heights = sample.get_all(shape);

      auto json = json::map({});
      switch (action) {
      case PROFILE:
        json = json::map({
          {"profile", serialize_range_height(ranges, heights, sample.get_no_data_value())}
        });
        break;
      case ELEVATION:
        json = json::map({
          {"elevation", serialize_height(heights, sample.get_no_data_value())}
        });
        break;
      }
      if(encoded_polyline)
        json->emplace("input_encoded_polyline", *encoded_polyline);
      else
        json->emplace("input_shape", serialize_shape(shape));

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

    void cleanup() {
      shape.clear();
      encoded_polyline.reset();
    }
    protected:
      std::vector<PointLL> shape;
      boost::optional<std::string> encoded_polyline;
      skadi::sample sample;
  };
}

namespace valhalla {
  namespace skadi {
    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from the http server
      auto upstream_endpoint = config.get<std::string>("skadi.service.proxy") + "_out";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");

      //listen for requests
      zmq::context_t context;
      skadi_worker_t skadi_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, "ipc://TODO", loopback_endpoint,
        std::bind(&skadi_worker_t::work, std::ref(skadi_worker), std::placeholders::_1, std::placeholders::_2),
        std::bind(&skadi_worker_t::cleanup, std::ref(skadi_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
  }
}
