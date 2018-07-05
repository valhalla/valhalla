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
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astar.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/isochrone.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/thor/multimodal.h>
#include <valhalla/thor/timedep.h>
#include <valhalla/thor/trippathbuilder.h>
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
  thor_worker_t(const boost::property_tree::ptree& config);
  virtual ~thor_worker_t();
#ifdef HAVE_HTTP
  virtual worker_t::result_t work(const std::list<zmq::message_t>& job,
                                  void* request_info,
                                  const std::function<void()>& interrupt) override;
#endif
  virtual void cleanup() override;

  std::list<odin::TripPath> route(valhalla_request_t& request);
  std::string matrix(valhalla_request_t& request);
  std::list<odin::TripPath> optimized_route(valhalla_request_t& request);
  std::string isochrones(valhalla_request_t& request);
  odin::TripPath trace_route(valhalla_request_t& request);
  std::string trace_attributes(valhalla_request_t& request);

protected:
  std::vector<thor::PathInfo> get_path(PathAlgorithm* path_algorithm,
                                       odin::Location& origin,
                                       odin::Location& destination,
                                       const std::string& costing);
  void log_admin(const odin::TripPath&);
  valhalla::sif::cost_ptr_t get_costing(const rapidjson::Document& request,
                                        const std::string& costing);
  thor::PathAlgorithm* get_path_algorithm(const std::string& routetype,
                                          const odin::Location& origin,
                                          const odin::Location& destination);
  odin::TripPath route_match(valhalla_request_t& request, const AttributesController& controller);
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>>
  map_match(valhalla_request_t& request,
            const AttributesController& controller,
            uint32_t best_paths = 1);

  std::list<odin::TripPath>
  path_arrive_by(google::protobuf::RepeatedPtrField<valhalla::odin::Location>& correlated,
                 const std::string& costing);
  std::list<odin::TripPath>
  path_depart_at(google::protobuf::RepeatedPtrField<valhalla::odin::Location>& correlated,
                 const std::string& costing);

  void parse_locations(valhalla_request_t& request);
  void parse_measurements(const valhalla_request_t& request);
  std::string parse_costing(const valhalla_request_t& request);
  void filter_attributes(const valhalla_request_t& request, AttributesController& controller);

  valhalla::sif::TravelMode mode;
  std::vector<meili::Measurement> trace;
  sif::CostFactory<sif::DynamicCost> factory;
  valhalla::sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
  // Path algorithms (TODO - perhaps use a map?))
  AStarPathAlgorithm astar;
  BidirectionalAStar bidir_astar;
  MultiModalPathAlgorithm multi_modal_astar;
  TimeDepForward timedep_forward;
  TimeDepReverse timedep_reverse;
  Isochrone isochrone_gen;
  std::shared_ptr<meili::MapMatcher> matcher;
  float long_request;
  std::unordered_map<std::string, float> max_matrix_distance;
  SOURCE_TO_TARGET_ALGORITHM source_to_target_algorithm;
  valhalla::meili::MapMatcherFactory matcher_factory;
  valhalla::baldr::GraphReader& reader;
};

} // namespace thor
} // namespace valhalla

#endif //__VALHALLA_THOR_SERVICE_H__
