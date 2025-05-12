// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_H_
#define MMP_MAP_MATCHER_H_

#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/config.h>
#include <valhalla/meili/emission_cost_model.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/meili/routing.h>
#include <valhalla/meili/state.h>
#include <valhalla/meili/topk_search.h>
#include <valhalla/meili/transition_cost_model.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace meili {

// A facade that connects everything
class MapMatcher final {
public:
  MapMatcher(const Config& config,
             baldr::GraphReader& graphreader,
             CandidateQuery& candidatequery,
             const sif::mode_costing_t& mode_costing,
             sif::TravelMode travelmode);

  ~MapMatcher();

  void Clear();

  baldr::GraphReader& graphreader() const {
    return graphreader_;
  }

  const CandidateQuery& candidatequery() const {
    return candidatequery_;
  }

  const StateContainer& state_container() const {
    return container_;
  }

  const EmissionCostModel& emission_cost_model() const {
    return emission_cost_model_;
  }

  const TransitionCostModel& transition_cost_model() const {
    return transition_cost_model_;
  }

  sif::TravelMode travelmode() const {
    return travelmode_;
  }

  const Config& config() const {
    return config_;
  }

  sif::cost_ptr_t costing() const {
    return mode_costing_[static_cast<size_t>(travelmode_)];
  }

  std::vector<MatchResults> OfflineMatch(const std::vector<Measurement>& measurements,
                                         uint32_t k = 1);

  /**
   * Set a callback that will throw when the map-matching should be aborted
   * @param interrupt_callback  the function to periodically call to see if we should abort
   */
  void set_interrupt(const std::function<void()>* interrupt_callback) {
    interrupt_ = interrupt_callback;
    graphreader_.SetInterrupt(interrupt_);
  }

private:
  std::unordered_map<StateId::Time, std::vector<Measurement>>
  AppendMeasurements(const std::vector<Measurement>& measurements);

  StateId::Time AppendMeasurement(const Measurement& measurement, const float sq_max_search_radius);

  void RemoveRedundancies(const std::vector<StateId>& result,
                          const std::vector<MatchResult>& results);

  Config config_;

  baldr::GraphReader& graphreader_;

  CandidateQuery& candidatequery_;

  const sif::mode_costing_t mode_costing_;

  sif::TravelMode travelmode_;

  // Interrupt callback. Can be set to interrupt if connection is closed.
  const std::function<void()>* interrupt_;

  ViterbiSearch vs_;

  TopKSearch ts_;

  StateContainer container_;

  EmissionCostModel emission_cost_model_;

  TransitionCostModel transition_cost_model_;
};

/**
 * Here we return the vector of edge segments between the source and target states. If its a node to
 * node route (meaning no real edge is traversed) then we use the target_result to say what edge the
 * segment should use
 * @param source         source state to use to find the route
 * @param target         target state which candidate in the next column to fetch the route for
 * @param route          a place to put the edge segments as we create them
 * @param target_result  in case we have a node to node route we have a no-op edge segment to return
 * @return  the vector of segments representing the route between source and target
 */
bool MergeRoute(const State& source,
                const State& target,
                std::vector<EdgeSegment>& route,
                const MatchResult& target_result);
/**
 * This loops over all of the states in the match and returns the vector of edge segments representing
 * the matched path.
 * @param mapmatcher     The matcher with which the match was computed
 * @param match_results  The matched points
 * @return  The vector of edge segments representing the match path
 */
std::vector<EdgeSegment> ConstructRoute(const MapMatcher& mapmatcher,
                                        const std::vector<MatchResult>& match_results);

template <typename segment_iterator_t>
std::vector<std::vector<midgard::PointLL>> ConstructRouteShapes(baldr::GraphReader& graphreader,
                                                                segment_iterator_t begin,
                                                                segment_iterator_t end);

} // namespace meili
} // namespace valhalla
#endif // MMP_MAP_MATCHER_H_
