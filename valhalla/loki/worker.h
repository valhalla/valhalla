#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <cstdint>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/connectivity_map.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/skadi/sample.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace loki {

#ifdef HAVE_HTTP
void run_service(const boost::property_tree::ptree& config);
#endif

class loki_worker_t : public service_worker_t {
public:
  loki_worker_t(const boost::property_tree::ptree& config,
                const std::shared_ptr<baldr::GraphReader>& graph_reader = {});
#ifdef HAVE_HTTP
  virtual worker_t::result_t work(const std::list<zmq::message_t>& job,
                                  void* request_info,
                                  const std::function<void()>& interrupt) override;
  void limits(valhalla_request_t& request) const;
#endif
  virtual void cleanup() override;

  std::string locate(valhalla_request_t& request);
  void route(valhalla_request_t& request);
  void matrix(valhalla_request_t& request);
  void isochrones(valhalla_request_t& request);
  void trace(valhalla_request_t& request);
  std::string height(valhalla_request_t& request);
  std::string transit_available(valhalla_request_t& request);

protected:
  void parse_locations(
      google::protobuf::RepeatedPtrField<odin::Location>* locations,
      boost::optional<valhalla_exception_t> required_exception = valhalla_exception_t{110});
  void parse_trace(valhalla_request_t& request);
  void parse_costing(valhalla_request_t& request);
  void locations_from_shape(valhalla_request_t& request);

  void init_locate(valhalla_request_t& request);
  void init_route(valhalla_request_t& request);
  void init_matrix(valhalla_request_t& request);
  void init_isochrones(valhalla_request_t& request);
  void init_trace(valhalla_request_t& request);
  std::vector<PointLL> init_height(valhalla_request_t& request);
  void init_transit_available(valhalla_request_t& request);

  boost::property_tree::ptree config;
  sif::CostFactory<sif::DynamicCost> factory;
  sif::EdgeFilter edge_filter;
  sif::NodeFilter node_filter;
  std::shared_ptr<baldr::GraphReader> reader;
  std::shared_ptr<baldr::connectivity_map_t> connectivity_map;
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
  size_t max_trace_shape;
  float max_gps_accuracy;
  float max_search_radius;
  unsigned int max_best_paths;
  size_t max_best_paths_shape;
  skadi::sample sample;
  size_t max_elevation_shape;
  float min_resample;
};
} // namespace loki
} // namespace valhalla

#endif //__VALHALLA_LOKI_SERVICE_H__
