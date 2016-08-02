#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <vector>
#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/trippathbuilder.h>
#include <valhalla/thor/isochrone.h>


namespace valhalla {
  namespace thor {

    void run_service(const boost::property_tree::ptree& config);

    class thor_worker_t {
     public:
      enum MATRIX_TYPE { ONE_TO_MANY, MANY_TO_ONE, MANY_TO_MANY, OPTIMIZED_ROUTE};
      thor_worker_t(const boost::property_tree::ptree& config);
      virtual ~thor_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:
      std::string init_request(const boost::property_tree::ptree& request);
      prime_server::worker_t::result_t trip_path(const std::string &costing, const std::string &request_str, const boost::optional<int> &date_time_type, const bool header_dnt);
      std::list<valhalla::odin::TripPath> path_arrive_by(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const std::string &request_str);
      std::list<valhalla::odin::TripPath> path_depart_from(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const boost::optional<int> &date_time_type, const std::string &request_str);
      prime_server::worker_t::result_t matrix(const MATRIX_TYPE matrix_type, const std::string &costing, const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t optimized_path(const std::vector<baldr::PathLocation>& correlated, const std::string &costing, const std::string &request_str, const bool header_dnt);
      void update_origin(baldr::PathLocation& origin, bool prior_is_node, const baldr::GraphId& through_edge);
      void get_path(PathAlgorithm* path_algorithm, baldr::PathLocation& origin, baldr::PathLocation& destination, std::vector<thor::PathInfo>& path_edges);
      valhalla::sif::cost_ptr_t get_costing(const boost::property_tree::ptree& request, const std::string& costing);
      thor::PathAlgorithm* get_path_algorithm(const std::string& routetype, const baldr::PathLocation& origin, const baldr::PathLocation& destination);

      valhalla::sif::TravelMode mode;
      boost::property_tree::ptree config;
      std::vector<baldr::Location> locations;
      std::vector<baldr::PathLocation> correlated;
      sif::CostFactory<sif::DynamicCost> factory;
      valhalla::sif::cost_ptr_t mode_costing[4];    // TODO - max # of modes?
      valhalla::baldr::GraphReader reader;
      // Path algorithms (TODO - perhaps use a map?))
      PathAlgorithm astar;
      BidirectionalAStar bidir_astar;
      MultiModalPathAlgorithm multi_modal_astar;
      Isochrone isochrone;
      float long_request;
      boost::optional<int> date_time_type;
    };
  }
}

#endif //__VALHALLA_THOR_SERVICE_H__
