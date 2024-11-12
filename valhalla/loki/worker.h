#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/connectivity_map.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/skadi/sample.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace loki {

#ifdef ENABLE_SERVICES
void run_service(const boost::property_tree::ptree& config);
#endif

class loki_worker_t : public service_worker_t {
public:
  loki_worker_t(const boost::property_tree::ptree& config,
                const std::shared_ptr<baldr::GraphReader>& graph_reader = {});
#ifdef ENABLE_SERVICES
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) override;
#endif
  virtual void cleanup() override;

  std::string locate(Api& request);
  void route(Api& request);
  void matrix(Api& request);
  void isochrones(Api& request);
  void trace(Api& request);
  std::string height(Api& request);
  std::string transit_available(Api& request);
  void status(Api& request) const;

  void set_interrupt(const std::function<void()>* interrupt) override;

protected:
  void parse_locations(google::protobuf::RepeatedPtrField<valhalla::Location>* locations,
                       std::optional<valhalla_exception_t> required_exception = valhalla_exception_t{
                           110});
  void parse_trace(Api& request);
  void parse_costing(Api& request, bool allow_none = false);
  void locations_from_shape(Api& request);
  void check_hierarchy_distance(Api& request);

  void init_locate(Api& request);
  void init_route(Api& request);
  void init_matrix(Api& request);
  void init_isochrones(Api& request);
  void init_trace(Api& request);
  std::vector<midgard::PointLL> init_height(Api& request);
  void init_transit_available(Api& request);

  boost::property_tree::ptree config;
  sif::CostFactory factory;
  sif::cost_ptr_t costing;
  std::shared_ptr<baldr::GraphReader> reader;
  std::shared_ptr<baldr::connectivity_map_t> connectivity_map;
  std::unordered_set<Options::Action> actions;
  std::string action_str;
  std::unordered_map<std::string, size_t> max_locations;
  std::unordered_map<std::string, float> max_distance;
  std::unordered_map<std::string, float> max_matrix_distance;
  size_t max_timedep_dist_matrix;
  std::unordered_map<std::string, float> max_matrix_locations;
  size_t max_exclude_locations;
  float max_exclude_polygons_length;
  unsigned int max_reachability;
  unsigned int default_reachability;
  unsigned int max_radius;
  unsigned int default_radius;
  unsigned int default_heading_tolerance;
  unsigned int default_node_snap_tolerance;
  unsigned int default_search_cutoff;
  unsigned int default_street_side_tolerance;
  unsigned int default_street_side_max_distance;
  float default_breakage_distance;
  // Minimum and maximum walking distances (to validate input).
  size_t min_transit_walking_dis;
  size_t max_transit_walking_dis;
  size_t max_contours;
  size_t max_contour_min;
  size_t max_contour_km;
  size_t max_trace_shape;
  float max_gps_accuracy;
  float max_search_radius;
  unsigned int max_trace_alternates;
  size_t max_trace_alternates_shape;
  skadi::sample sample;
  size_t max_elevation_shape;
  float min_resample;
  unsigned int max_alternates;
  bool allow_verbose;

  // add max_distance_disable_hierarchy_culling
  float max_distance_disable_hierarchy_culling;

private:
  std::string service_name() const override {
    return "loki";
  }
};
} // namespace loki
} // namespace valhalla

#endif //__VALHALLA_LOKI_SERVICE_H__
