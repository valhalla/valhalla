#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <cstdint>
#include <tuple>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/location.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astar.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/isochrone.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/thor/multimodal.h>
#include <valhalla/thor/timedep.h>
#include <valhalla/thor/triplegbuilder.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace thor {

#ifdef HAVE_HTTP
void run_service(const boost::property_tree::ptree& config);
#endif

class thor_worker_t : public service_worker_t {
public:
  enum SOURCE_TO_TARGET_ALGORITHM { SELECT_OPTIMAL = 0, COST_MATRIX = 1, TIME_DISTANCE_MATRIX = 2 };
  thor_worker_t(const boost::property_tree::ptree& config,
                const std::shared_ptr<baldr::GraphReader>& graph_reader = {});
  virtual ~thor_worker_t();
#ifdef HAVE_HTTP
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) override;
#endif
  virtual void cleanup() override;

  void route(Api& request);
  std::string matrix(Api& request);
  void optimized_route(Api& request);
  std::string isochrones(Api& request);
  void trace_route(Api& request);
  std::string trace_attributes(Api& request);
  std::string expansion(Api& request);

protected:
  std::vector<std::vector<thor::PathInfo>> get_path(PathAlgorithm* path_algorithm,
                                                    Location& origin,
                                                    Location& destination,
                                                    const std::string& costing,
                                                    const Options& options);
  void log_admin(const TripLeg&);
  sif::cost_ptr_t get_costing(const Costing costing, const Options& options);
  thor::PathAlgorithm* get_path_algorithm(const std::string& routetype,
                                          const Location& origin,
                                          const Location& destination);
  void route_match(Api& request);
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>> map_match(Api& request);
  void path_map_match(const std::vector<meili::MatchResult>& match_results,
                      const std::vector<PathInfo>& path_edges,
                      TripLeg& leg,
                      std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>&
                          route_discontinuities);
  void path_arrive_by(Api& api, const std::string& costing);
  void path_depart_at(Api& api, const std::string& costing);

  void parse_locations(Api& request);
  void parse_measurements(const Api& request);
  std::string parse_costing(const Api& request);
  void parse_filter_attributes(const Api& request, bool is_strict_filter = false);
  sif::TravelMode mode;
  std::vector<meili::Measurement> trace;
  std::vector<PathInfo> m_path_infos;
  std::vector<meili::MatchResults> m_offline_results;
  std::vector<thor::MatchResult> m_temp_enhanced_match_results;
  std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>
      m_temp_route_discontinuities;
  std::vector<std::pair<baldr::GraphId, baldr::GraphId>> m_temp_disconnected_edges;
  std::vector<PathInfo> m_temp_path_edges;
  std::vector<std::pair<std::vector<PathInfo>, std::vector<meili::MatchResult>::const_iterator>>
      m_temp_disjoint_edge_groups;
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>> m_map_match_results;
  sif::CostFactory<sif::DynamicCost> factory;
  sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
  // Path algorithms (TODO - perhaps use a map?))
  AStarPathAlgorithm astar;
  BidirectionalAStar bidir_astar;
  MultiModalPathAlgorithm multi_modal_astar;
  TimeDepForward timedep_forward;
  TimeDepReverse timedep_reverse;
  Isochrone isochrone_gen;
  std::shared_ptr<meili::MapMatcher> matcher;
  float long_request;
  float max_timedep_distance;
  std::unordered_map<std::string, float> max_matrix_distance;
  SOURCE_TO_TARGET_ALGORITHM source_to_target_algorithm;
  meili::MapMatcherFactory matcher_factory;
  std::shared_ptr<baldr::GraphReader> reader;
  AttributesController controller;
};

} // namespace thor
} // namespace valhalla

#endif //__VALHALLA_THOR_SERVICE_H__
