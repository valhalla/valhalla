// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_H_
#define MMP_MAP_MATCHER_H_

#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/candidate_search.h>
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
  MapMatcher(const boost::property_tree::ptree& config,
             baldr::GraphReader& graphreader,
             CandidateQuery& candidatequery,
             const sif::cost_ptr_t* mode_costing,
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

  const boost::property_tree::ptree& config() const {
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
  }

private:
  std::unordered_map<StateId::Time, std::vector<Measurement>>
  AppendMeasurements(const std::vector<Measurement>& measurements);

  StateId::Time AppendMeasurement(const Measurement& measurement, const float sq_max_search_radius);

  void RemoveRedundancies(const std::vector<StateId>& result);
  // void RemoveRedundancies(const MatchResults& path, std::vector<StateId>& result);

  boost::property_tree::ptree config_;

  baldr::GraphReader& graphreader_;

  CandidateQuery& candidatequery_;

  const sif::cost_ptr_t* mode_costing_;

  sif::TravelMode travelmode_;

  // Interrupt callback. Can be set to interrupt if connection is closed.
  const std::function<void()>* interrupt_;

  ViterbiSearch vs_;

  TopKSearch ts_;

  StateContainer container_;

  EmissionCostModel emission_cost_model_;

  TransitionCostModel transition_cost_model_;
};

bool MergeRoute(std::vector<EdgeSegment>& route, const State& source, const State& target);

std::vector<EdgeSegment> MergeRoute(const State& source, const State& target);

template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(const MapMatcher& mapmatcher, match_iterator_t begin, match_iterator_t end);

template <typename segment_iterator_t>
std::vector<std::vector<midgard::PointLL>> ConstructRouteShapes(baldr::GraphReader& graphreader,
                                                                segment_iterator_t begin,
                                                                segment_iterator_t end);

} // namespace meili
} // namespace valhalla
#endif // MMP_MAP_MATCHER_H_
