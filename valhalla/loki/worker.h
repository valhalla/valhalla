#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <cstdint>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/worker.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/connectivity_map.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/tyr/actor.h>

namespace valhalla {
  namespace loki {

#ifdef HAVE_HTTP
    void run_service(const boost::property_tree::ptree& config);
#endif

    class loki_worker_t : public service_worker_t {
     public:
      loki_worker_t(const boost::property_tree::ptree& config);
#ifdef HAVE_HTTP
      virtual worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt) override;
#endif
      virtual void cleanup() override;

      baldr::json::ArrayPtr locate(rapidjson::Document& request);
      void route(rapidjson::Document& request);
      void matrix(tyr::ACTION_TYPE action, rapidjson::Document& request);
      void isochrones(rapidjson::Document& request);
      void trace(tyr::ACTION_TYPE action, rapidjson::Document& request);
      baldr::json::ArrayPtr transit_available(rapidjson::Document& request);

     protected:

      std::vector<baldr::Location> parse_locations(const rapidjson::Document& request, const std::string& node, unsigned location_parse_error_code = 130,
        boost::optional<valhalla_exception_t> required_exception = valhalla_exception_t{110});
      void parse_trace(rapidjson::Document& request);
      void parse_costing(rapidjson::Document& request);
      void locations_from_shape(rapidjson::Document& request);

      void init_locate(rapidjson::Document& request);
      void init_route(rapidjson::Document& request);
      void init_matrix(tyr::ACTION_TYPE action, rapidjson::Document& request);
      void init_isochrones(rapidjson::Document& request);
      void init_trace(rapidjson::Document& request);
      void init_transit_available(rapidjson::Document& request);

      boost::property_tree::ptree config;
      std::vector<baldr::Location> locations;
      std::vector<baldr::Location> sources;
      std::vector<baldr::Location> targets;
      std::vector<midgard::PointLL> shape;
      sif::CostFactory<sif::DynamicCost> factory;
      sif::EdgeFilter edge_filter;
      sif::NodeFilter node_filter;
      valhalla::baldr::GraphReader reader;
      std::shared_ptr<valhalla::baldr::connectivity_map_t> connectivity_map;
      std::unordered_set<std::string> actions;
      std::string action_str;
      std::unordered_map<std::string, size_t> max_locations;
      std::unordered_map<std::string, float> max_distance;
      std::unordered_map<std::string, float> max_matrix_distance;
      std::unordered_map<std::string, float> max_matrix_locations;
      size_t max_avoid_locations;
      unsigned int max_reachability;
      unsigned int default_reachability;
      unsigned long max_radius;
      unsigned long default_radius;
      float long_request;
      // Minimum and maximum walking distances (to validate input).
      size_t min_transit_walking_dis;
      size_t max_transit_walking_dis;
      size_t max_contours;
      size_t max_time;
      size_t max_shape;
      float max_gps_accuracy;
      float max_search_radius;
      unsigned int max_best_paths;
      size_t max_best_paths_shape;
      bool healthcheck;
    };
  }
}

#endif //__VALHALLA_LOKI_SERVICE_H__
