#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/pathlocation.h>

namespace valhalla {
  namespace thor {

    void run_service(const boost::property_tree::ptree& config);

    class thor_worker_t {
     public:
      enum MATRIX_TYPE { ONE_TO_MANY, MANY_TO_ONE, MANY_TO_MANY };
      thor_worker_t(const boost::property_tree::ptree& config);
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void init_request(const boost::property_tree::ptree& request);
      prime_server::worker_t::result_t trip_path(const std::string &costing, const std::string &request_str, const boost::optional<int> &date_time_type);
      prime_server::worker_t::result_t matrix(const MATRIX_TYPE matrix_type, const std::string &costing, const boost::property_tree::ptree &request, http_request_t::info_t& request_info);
      prime_server::worker_t::result_t optimized_path(const std::vector<PathLocation> correlated, const std::string &costing, const std::string &request_str);

     void cleanup() {
       astar.Clear();
       bidir_astar.Clear();
       multi_modal_astar.Clear();
       locations.clear();
       correlated.clear();
       if(reader.OverCommitted())
       reader.Clear();
     }
     protected:
      valhalla::sif::TravelMode mode;
      boost::property_tree::ptree config;
      std::vector<baldr::Location> locations;
      std::vector<PathLocation> correlated;
      sif::CostFactory<sif::DynamicCost> factory;
      valhalla::sif::cost_ptr_t mode_costing[4];    // TODO - max # of modes?
      valhalla::baldr::GraphReader reader;

      // Path algorithms (TODO - perhaps use a map?))
      thor::PathAlgorithm astar;
      thor::BidirectionalAStar bidir_astar;
      thor::MultiModalPathAlgorithm multi_modal_astar;
      float long_request_route;
      float long_request_manytomany;
      double distance_scale;
      boost::optional<int> date_time_type;
    };
  }
}


#endif //__VALHALLA_THOR_SERVICE_H__
