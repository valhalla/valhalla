#ifndef __VALHALLA_THOR_SERVICE_H__
#define __VALHALLA_THOR_SERVICE_H__

#include <tuple>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/attributes_controller.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/location.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astar_bss.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/centroid.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/isochrone.h>
#include <valhalla/thor/multimodal.h>
#include <valhalla/thor/timedistancebssmatrix.h>
#include <valhalla/thor/timedistancematrix.h>
#include <valhalla/thor/triplegbuilder.h>
#include <valhalla/thor/unidirectional_astar.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace thor {

#ifdef ENABLE_SERVICES
void run_service(const boost::property_tree::ptree& config);
#endif

class thor_worker_t : public service_worker_t {
public:
  enum SOURCE_TO_TARGET_ALGORITHM { SELECT_OPTIMAL = 0, COST_MATRIX = 1, TIME_DISTANCE_MATRIX = 2 };
  thor_worker_t(const boost::property_tree::ptree& config,
                const std::shared_ptr<baldr::GraphReader>& graph_reader = {});
  virtual ~thor_worker_t();
#ifdef ENABLE_SERVICES
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) override;
#endif
  virtual void cleanup() override;

  static void adjust_scores(valhalla::Options& options);

  void route(Api& request);
  std::string matrix(Api& request);
  void optimized_route(Api& request);
  std::string isochrones(Api& request);
  void trace_route(Api& request);
  std::string trace_attributes(Api& request);
  std::string expansion(Api& request);
  void centroid(Api& request);
  void status(Api& request) const;

  void set_interrupt(const std::function<void()>* interrupt) override;

protected:
  std::vector<std::vector<thor::PathInfo>> get_path(PathAlgorithm* path_algorithm,
                                                    Location& origin,
                                                    Location& destination,
                                                    const std::string& costing,
                                                    const Options& options);
  void log_admin(const TripLeg&);
  thor::PathAlgorithm* get_path_algorithm(const std::string& routetype,
                                          const Location& origin,
                                          const Location& destination,
                                          const Options& options);
  thor::MatrixAlgorithm*
  get_matrix_algorithm(Api& request, const bool has_time, const std::string& costing);
  void route_match(Api& request);
  /**
   * Returns the results of the map match where the first float is the normalized
   * match score (based on alternatives), the second is the raw score (the cost)
   * and the final is the list of match results (how the trace points were matched)
   * @param request   The request to map match (options.shape)
   * @return the match results and scores
   */
  std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>> map_match(Api& request);

  void path_arrive_by(Api& api, const std::string& costing);
  void path_depart_at(Api& api, const std::string& costing);
  void parse_measurements(const Api& request);
  std::string parse_costing(const Api& request);

  void build_route(
      const std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>&
          paths,
      const std::vector<meili::MatchResult>& match_results,
      Options& options,
      Api& request);

  void build_trace(
      const std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>&
          paths,
      std::vector<meili::MatchResult>& match_results,
      Options& options,
      Api& request);

  sif::TravelMode mode;
  std::vector<meili::Measurement> trace;
  sif::CostFactory factory;
  sif::mode_costing_t mode_costing;

  // Path algorithms (TODO - perhaps use a map?))
  BidirectionalAStar bidir_astar;
  AStarBSSAlgorithm bss_astar;
  MultiModalPathAlgorithm multi_modal_astar;
  TimeDepForward timedep_forward;
  TimeDepReverse timedep_reverse;

  // Time distance matrix
  CostMatrix costmatrix_;
  TimeDistanceMatrix time_distance_matrix_;
  TimeDistanceBSSMatrix time_distance_bss_matrix_;

  Isochrone isochrone_gen;
  std::shared_ptr<meili::MapMatcher> matcher;
  float max_timedep_distance;
  std::unordered_map<std::string, float> max_matrix_distance;
  SOURCE_TO_TARGET_ALGORITHM source_to_target_algorithm;
  bool costmatrix_allow_second_pass;
  std::shared_ptr<baldr::GraphReader> reader;
  meili::MapMatcherFactory matcher_factory;
  baldr::AttributesController controller;
  Centroid centroid_gen;

private:
  std::string service_name() const override {
    return "thor";
  }
};

} // namespace thor
} // namespace valhalla

#endif //__VALHALLA_THOR_SERVICE_H__
