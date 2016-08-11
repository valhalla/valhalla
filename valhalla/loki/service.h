#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/connectivity_map.h>
#include <valhalla/sif/costfactory.h>

namespace valhalla {
  namespace loki {

    void run_service(const boost::property_tree::ptree& config);

    class loki_worker_t {
     public:
      enum ACTION_TYPE {ROUTE = 0, VIAROUTE = 1, LOCATE = 2, ONE_TO_MANY = 3, MANY_TO_ONE = 4, MANY_TO_MANY = 5,
                        SOURCES_TO_TARGETS = 6, OPTIMIZED_ROUTE = 7, ISOCHRONE = 8, ATTRIBUTES = 9};
      loki_worker_t(const boost::property_tree::ptree& config);
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:
      void parse_locations(const boost::property_tree::ptree& request);
      void parse_costing(const boost::property_tree::ptree& request);

      void init_locate(const boost::property_tree::ptree& request);
      void init_route(const boost::property_tree::ptree& request);
      void init_matrix(ACTION_TYPE action, boost::property_tree::ptree& request);
      void init_isochrones(const boost::property_tree::ptree& request);
      void init_attributes(const boost::property_tree::ptree& request);

      prime_server::worker_t::result_t locate(const boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t route(boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t matrix(ACTION_TYPE action,boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t isochrones(boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t attributes(const boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);

      boost::property_tree::ptree config;
      std::vector<baldr::Location> locations;
      std::vector<baldr::Location> sources;
      std::vector<baldr::Location> targets;
      sif::CostFactory<sif::DynamicCost> factory;
      sif::EdgeFilter edge_filter;
      sif::NodeFilter node_filter;
      valhalla::baldr::GraphReader reader;
      valhalla::baldr::connectivity_map_t connectivity_map;
      std::string action_str;
      std::unordered_map<std::string, size_t> max_locations;
      std::unordered_map<std::string, float> max_distance;
      float long_request;
      // Minimum and maximum walking distances (to validate input).
      unsigned int min_transit_walking_dis;
      unsigned int max_transit_walking_dis;
      unsigned int max_contours;
      unsigned int max_time;
    };
  }
}

#endif //__VALHALLA_LOKI_SERVICE_H__
