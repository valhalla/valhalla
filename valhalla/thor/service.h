#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/astar.h>
#include <valhalla/thor/multimodal.h>
#include <valhalla/thor/trippathbuilder.h>
#include <valhalla/thor/isochrone.h>


namespace valhalla {
  namespace thor {

    void run_service(const boost::property_tree::ptree& config);

    class thor_worker_t {
     public:
      enum ACTION_TYPE {ROUTE = 0, VIAROUTE = 1, LOCATE = 2, ONE_TO_MANY = 3, MANY_TO_ONE = 4, MANY_TO_MANY = 5,
                        SOURCES_TO_TARGETS = 6, OPTIMIZED_ROUTE = 7, ISOCHRONE = 8, ATTRIBUTES = 9};
      thor_worker_t(const boost::property_tree::ptree& config);
      virtual ~thor_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:

      void update_origin(baldr::PathLocation& origin, bool prior_is_node, const baldr::GraphId& through_edge);
      void get_path(PathAlgorithm* path_algorithm, baldr::PathLocation& origin, baldr::PathLocation& destination, std::vector<thor::PathInfo>& path_edges);
      valhalla::sif::cost_ptr_t get_costing(const boost::property_tree::ptree& request, const std::string& costing);
      thor::PathAlgorithm* get_path_algorithm(const std::string& routetype, const baldr::PathLocation& origin, const baldr::PathLocation& destination);

      std::list<valhalla::odin::TripPath> path_arrive_by(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const std::string &request_str);
      std::list<valhalla::odin::TripPath> path_depart_at(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const boost::optional<int> &date_time_type, const std::string &request_str);

      std::string parse_costing(const boost::property_tree::ptree& request);
      void parse_locations(const boost::property_tree::ptree& request);

      prime_server::worker_t::result_t route(const boost::property_tree::ptree& request, const std::string &request_str, const boost::optional<int> &date_time_type, const bool header_dnt);
      prime_server::worker_t::result_t matrix(ACTION_TYPE matrix_type, const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t optimized_route(const boost::property_tree::ptree& request, const std::string &request_str, const bool header_dnt);
      prime_server::worker_t::result_t isochrone(const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info);
      prime_server::worker_t::result_t attributes(const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info);


      valhalla::sif::TravelMode mode;
      boost::property_tree::ptree config;
      boost::optional<std::string> jsonp;
      std::vector<baldr::Location> locations;
      std::vector<baldr::Location> sources;
      std::vector<baldr::Location> targets;
      std::vector<baldr::PathLocation> correlated;
      std::vector<baldr::PathLocation> correlated_s;
      std::vector<baldr::PathLocation> correlated_t;
      sif::CostFactory<sif::DynamicCost> factory;
      valhalla::sif::cost_ptr_t mode_costing[4];    // TODO - max # of modes?
      valhalla::baldr::GraphReader reader;
      // Path algorithms (TODO - perhaps use a map?))
      AStarPathAlgorithm astar;
      BidirectionalAStar bidir_astar;
      MultiModalPathAlgorithm multi_modal_astar;
      Isochrone isochrone_gen;
      float long_request;
      boost::optional<int> date_time_type;
    };
  }
}

#endif //__VALHALLA_THOR_SERVICE_H__
