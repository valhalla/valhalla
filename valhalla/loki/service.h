#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>


#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/costfactory.h>

namespace valhalla {
  namespace loki {

    void run_service(const boost::property_tree::ptree& config);

    class loki_worker_t {
     public:
      enum ACTION_TYPE {ROUTE, VIAROUTE, LOCATE, ONE_TO_MANY, MANY_TO_ONE, MANY_TO_MANY};
      loki_worker_t(const boost::property_tree::ptree& config);
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request);
      prime_server::worker_t::result_t locate(const boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t route(const ACTION_TYPE& action, boost::property_tree::ptree& request);
      prime_server::worker_t::result_t matrix(const ACTION_TYPE& action, boost::property_tree::ptree& request);
      void cleanup();
     protected:
      boost::property_tree::ptree config;
      std::vector<baldr::Location> locations;
      sif::CostFactory<sif::DynamicCost> factory;
      sif::EdgeFilter costing_filter;
      valhalla::baldr::GraphReader reader;
      std::unordered_set<std::string> action_set;
      std::string action_str;
    };
  }
}

#endif //__VALHALLA_LOKI_SERVICE_H__
