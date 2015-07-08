#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <array>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/srtmtile.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/midgard/distanceapproximator.h>

#include "skadi/service.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;


namespace {
  enum ACTION_TYPE {ELEVATION, VERSION};
  const std::unordered_map<std::string, ACTION_TYPE> ACTION{
    {"/elevation", ELEVATION}
  };
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};

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

   //Walk the shape, return the distance height.
   //Input vector of latlngs or encoded latlngs.
    json::ArrayPtr serialize_shape(const std::vector<midgard::PointLL &> shape) {
      auto array = json::array({});
      std::vector<std::vector<float>> elev;
      for (std::vector<midgard::PointLL &>::iterator i = shape.begin(); i != shape.end(); ++i) {
        try {
            elev getelevation(shape[i], shape[i+1]);
            array->emplace_back(
              json::map({
                {"range", json::fp_t{elev[0], 5}},
                {"height", json::fp_t{elev[1], 3}}
              })
            );
        }
        catch(...) {
          //this really shouldnt ever get hit
          LOG_WARN("");
        }
      }
      return array;
    }

    json::ArrayPtr serialize_shape(const std::string encoded_polyline) {
      auto array = json::array({});
      std::vector<midgard::PointLL &> shape;
      if(encoded_polyline != nullptr) {
        shape = midgard::decode<std::vector<PointLL> >(std::string(encoded_polyline));
      }
      return serialize_shape(shape);
    }

    json::MapPtr serialize(const std::vector<PointLL&> shape) {
      return json::map({
        {"elevation", serialize_shape(shape)},
        {"input_shape", json::fp_t(shape)}
      });
    }

    json::MapPtr serialize(const std::vector<PointLL&> shape, const std::string& reason) {
      return json::map({
        {"elevation", static_cast<std::nullptr_t>(nullptr)},
        {"input_shape", json::fp_t(shape)},
        {"reason", reason}
      });
    }

    json::MapPtr serialize(const std::string encoded_polyline) {
      return json::map({
        {"elevation", serialize_shape(encoded_polyline)},
        {"input_shape", json::fp_t(encoded_polyline)}
      });
    }

    json::MapPtr serialize(const std::string encoded_polyline, const std::string& reason) {
      return json::map({
        {"elevation", static_cast<std::nullptr_t>(nullptr)},
        {"input_shape", json::fp_t(encoded_polyline)},
        {"reason", reason}
      });
    }



  //TODO: throw this in the header to make it testable?
  class skadi_worker_t {
   public:
    skadi_worker_t(const boost::property_tree::ptree& config):config(config){
        }

    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Skadi Request " + std::to_string(info.id));
      //request should look like:
      //  /elevation?shape=&json=&jsonp=

      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        auto action = ACTION.find(request.path);
        if(action == ACTION.cend()) {
          worker_t::result_t result{false};
          http_response_t response(404, "Not Found", "Try: '/elevation'", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }

        //parse the query's json
        auto request_pt = from_request(action->second, request);
       init_request(action->second, request_pt);
        switch (action->second) {
          case ELEVATION:
            return elevation(request_pt, info);
        }

        worker_t::result_t result{false};
        http_response_t response(501, "Not Implemented", "", headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
      catch(const std::exception& e) {
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
           if (request.get_child("shape")) {
             for(const auto& latlng : request.get_child("shape")) {
            //   shape.push_back(midgard::PointLL &::FromPtree(latlng.second));
             }
             if(shape.size() < 1)
               throw std::runtime_error("Insufficient shape provided");

           } else if (request.get_child("encoded_polyline")) {
          //   encoded_polyline.push_back(midgard::PointLL &::FromPtree(request.get_child("encoded_polyline")));
           }
             if(encoded_polyline != "")
               throw std::runtime_error("Insufficient shape provided");

         }
         catch(...) {
           throw std::runtime_error("Insufficiently specified required parameter '" + std::string(action == ELEVATION ? "latlng'" : "shape'"));
         }
       }


    std::vector<std::vector<float>> getelevation(const midgard::PointLL& latlng, const midgard::PointLL& latlng2) {
      std::vector<std::vector<float>> elev;

      float height = valhalla::baldr::SRTMTile(latlng, true);
      float range = midgard::DistanceApproximator::DistanceSquared(latlng, latlng2);

      return elev[range][height];
    }

    //example elevation response:
    /*
    {elevation: [
       {
         range: 0
         height: 388.72873
       },
       {
          range: 4.37751
          height: 397.56055
       }
       ...
       ],
       input_shape: [
           40.0380,
          -76.3059,
          .......
          .......
       ],
       info: {
         status: 0,
         messages: []
      }
    }*/
    worker_t::result_t elevation(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
        std::vector<std::vector<float>> elevation;
        auto json = json::array({});

        if(shape.size() > 1) {

          for(const auto& latlng : shape) {
              try {
                json->emplace_back(serialize(shape));
              }
              catch(const std::exception& e) {
                json->emplace_back(serialize(shape, e.what()));
              }
            }
         } else if (encoded_polyline != "") {
           try {
                 json->emplace_back(serialize(encoded_polyline));
               }
               catch(const std::exception& e) {
                 json->emplace_back(serialize(encoded_polyline, e.what()));
               }
         }

          //jsonp callback if need be
          std::ostringstream stream;
          auto jsonp = request.get_optional<std::string>("jsonp");
          if(jsonp)
            stream << *jsonp << '(';
          stream << *json;
          if(jsonp)
            stream << ')';

          worker_t::result_t result{false};
          http_response_t response(200, "OK", stream.str(), headers_t{CORS, JSON_MIME});
          response.from_info(request_info);
          result.messages.emplace_back(response.to_string());
          return result;
    }

    void cleanup() {
      shape.clear();

    }
   protected:
    boost::property_tree::ptree config;
    std::vector<PointLL &> shape;
    std::string encoded_polyline;
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
