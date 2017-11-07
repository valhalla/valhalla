#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <cstdint>
#include <vector>
#include <tuple>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/worker.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/astar.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/thor/multimodal.h>
#include <valhalla/thor/trippathbuilder.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/isochrone.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/tyr/actor.h>

namespace valhalla {
namespace thor {

#ifdef HAVE_HTTP
void run_service(const boost::property_tree::ptree& config);
#endif

// <Confidence score, raw score, match results, trip path> tuple indexes
constexpr size_t kConfidenceScoreIndex = 0;
constexpr size_t kRawScoreIndex = 1;
constexpr size_t kMatchResultsIndex = 2;
constexpr size_t kTripPathIndex = 3;

class thor_worker_t : public service_worker_t{
 public:
  enum SHAPE_MATCH {
    EDGE_WALK = 0,
    MAP_SNAP = 1,
    WALK_OR_SNAP = 2
  };
  enum SOURCE_TO_TARGET_ALGORITHM {
    SELECT_OPTIMAL = 0,
    COST_MATRIX = 1,
    TIME_DISTANCE_MATRIX = 2
  };
  static const std::unordered_map<std::string, SHAPE_MATCH> STRING_TO_MATCH;
  thor_worker_t(const boost::property_tree::ptree& config);
  virtual ~thor_worker_t();
#ifdef HAVE_HTTP
  virtual worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt) override;
#endif
  virtual void cleanup() override;

  std::list<odin::TripPath> route(const boost::property_tree::ptree& request,
             const boost::optional<int> &date_time_type);
  baldr::json::MapPtr matrix(tyr::ACTION_TYPE matrix_type, const boost::property_tree::ptree& request);
  std::list<odin::TripPath> optimized_route(const boost::property_tree::ptree& request);
  baldr::json::MapPtr isochrones(const boost::property_tree::ptree& request);
  odin::TripPath trace_route(const boost::property_tree::ptree& request);
  baldr::json::MapPtr trace_attributes(const boost::property_tree::ptree& request);

 protected:

  std::vector<thor::PathInfo> get_path(PathAlgorithm* path_algorithm, baldr::PathLocation& origin,
                baldr::PathLocation& destination, const std::string& costing);
  void log_admin(odin::TripPath&);
  valhalla::sif::cost_ptr_t get_costing(
      const boost::property_tree::ptree& request, const std::string& costing);
  thor::PathAlgorithm* get_path_algorithm(
      const std::string& routetype, const baldr::PathLocation& origin,
      const baldr::PathLocation& destination);
  odin::TripPath route_match(const AttributesController& controller);
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>> map_match(
      const AttributesController& controller, bool trace_attributes_action = false,
      uint32_t best_paths = 1);

  std::list<odin::TripPath> path_arrive_by(
      std::vector<baldr::PathLocation>& correlated, const std::string &costing);
  std::list<odin::TripPath> path_depart_at(
      std::vector<baldr::PathLocation>& correlated, const std::string &costing,
      const boost::optional<int> &date_time_type);

  void parse_locations(const boost::property_tree::ptree& request);
  void parse_measurements(const boost::property_tree::ptree& request);
  void parse_trace_config(const boost::property_tree::ptree& request);
  std::string parse_costing(const boost::property_tree::ptree& request);
  void filter_attributes(const boost::property_tree::ptree& request, AttributesController& controller);

  valhalla::sif::TravelMode mode;
  std::vector<baldr::Location> locations;
  std::vector<meili::Measurement> trace;
  std::vector<baldr::PathLocation> correlated;
  std::vector<baldr::PathLocation> correlated_s;
  std::vector<baldr::PathLocation> correlated_t;
  sif::CostFactory<sif::DynamicCost> factory;
  valhalla::sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
  // Path algorithms (TODO - perhaps use a map?))
  AStarPathAlgorithm astar;
  BidirectionalAStar bidir_astar;
  MultiModalPathAlgorithm multi_modal_astar;
  Isochrone isochrone_gen;
  std::shared_ptr<meili::MapMatcher> matcher;
  float long_request;
  std::unordered_map<std::string, float> max_matrix_distance;
  SOURCE_TO_TARGET_ALGORITHM source_to_target_algorithm;
  boost::optional<int> date_time_type;
  valhalla::meili::MapMatcherFactory matcher_factory;
  valhalla::baldr::GraphReader& reader;
  std::unordered_set<std::string> trace_customizable;
  boost::property_tree::ptree trace_config;

  bool healthcheck;
  std::vector<uint32_t> optimal_order;
};

}
}

#endif //__VALHALLA_THOR_SERVICE_H__
