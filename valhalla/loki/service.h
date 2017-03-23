#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphfsreader.h>
#include <valhalla/baldr/connectivity_map.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/sif/costfactory.h>
#include <rapidjson/document.h>

namespace valhalla {
  namespace loki {

    void run_service(const boost::property_tree::ptree& config);
    class loki_worker_t {
     public:
      enum ACTION_TYPE {ROUTE = 0, VIAROUTE = 1, LOCATE = 2, ONE_TO_MANY = 3, MANY_TO_ONE = 4, MANY_TO_MANY = 5,
                        SOURCES_TO_TARGETS = 6, OPTIMIZED_ROUTE = 7, ISOCHRONE = 8, TRACE_ROUTE = 9, TRACE_ATTRIBUTES = 10};
      loki_worker_t(const boost::property_tree::ptree& config);
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const prime_server::worker_t::interrupt_function_t&);
      void cleanup();

     protected:

      prime_server::worker_t::result_t jsonify_error(const baldr::valhalla_exception_t& exception, prime_server::http_request_info_t& request_info) const;
      std::vector<baldr::Location> parse_locations(const rapidjson::Document& request, const std::string& node, unsigned location_parse_error_code = 130,
        boost::optional<baldr::valhalla_exception_t> required_exception = baldr::valhalla_exception_t{400, 110});
      void parse_trace(rapidjson::Document& request);
      void parse_costing(rapidjson::Document& request);
      void locations_from_shape(rapidjson::Document& request);

      void init_locate(rapidjson::Document& request);
      void init_route(rapidjson::Document& request);
      void init_matrix(ACTION_TYPE action, rapidjson::Document& request);
      void init_isochrones(rapidjson::Document& request);
      void init_trace(rapidjson::Document& request);

      prime_server::worker_t::result_t locate(rapidjson::Document& request, prime_server::http_request_info_t& request_info);
      prime_server::worker_t::result_t route(rapidjson::Document& request, prime_server::http_request_info_t& request_info);
      prime_server::worker_t::result_t matrix(ACTION_TYPE action,rapidjson::Document& request, prime_server::http_request_info_t& request_info);
      prime_server::worker_t::result_t isochrones(rapidjson::Document& request, prime_server::http_request_info_t& request_info);
      prime_server::worker_t::result_t trace_route(rapidjson::Document& request, prime_server::http_request_info_t& request_info);

      boost::property_tree::ptree config;
      boost::optional<std::string> jsonp;
      std::vector<baldr::Location> locations;
      std::vector<baldr::Location> sources;
      std::vector<baldr::Location> targets;
      std::vector<midgard::PointLL> shape;
      sif::CostFactory<sif::DynamicCost> factory;
      sif::EdgeFilter edge_filter;
      sif::NodeFilter node_filter;
      valhalla::baldr::GraphFsReader reader;
      valhalla::baldr::connectivity_map_t connectivity_map;
      std::unordered_set<std::string> actions;
      std::string action_str;
      std::unordered_map<std::string, size_t> max_locations;
      std::unordered_map<std::string, float> max_distance;
      size_t max_avoid_locations;
      float long_request;
      // Minimum and maximum walking distances (to validate input).
      size_t min_transit_walking_dis;
      size_t max_transit_walking_dis;
      size_t max_contours;
      size_t max_time;
      size_t max_shape;
      bool healthcheck;
    };
  }
}

#endif //__VALHALLA_LOKI_SERVICE_H__
