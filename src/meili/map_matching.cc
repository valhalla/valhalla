#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/pathlocation.h>

#include "meili/candidate_search.h"
#include "meili/routing.h"
#include "meili/graph_helpers.h"
#include "meili/geometry_helpers.h"
#include "meili/graph_helpers.h"
#include "meili/match_result.h"
#include "meili/map_matching.h"


namespace {

inline float
GreatCircleDistance(const valhalla::meili::Measurement& left,
                    const valhalla::meili::Measurement& right)
{ return left.lnglat().Distance(right.lnglat()); }

}


namespace valhalla {
namespace meili {

State::State(const StateId id, const Time time, const baldr::PathLocation& candidate)
    : id_(id),
      time_(time),
      candidate_(candidate),
      labelset_(nullptr),
      label_idx_() {}


void
State::route(const std::vector<const State*>& states,
             baldr::GraphReader& graphreader,
             float max_route_distance,
             const midgard::DistanceApproximator& approximator,
             float search_radius,
             sif::cost_ptr_t costing,
             std::shared_ptr<const sif::EdgeLabel> edgelabel,
             const float turn_cost_table[181]) const
{
  // Prepare locations
  std::vector<baldr::PathLocation> locations;
  locations.reserve(1 + states.size());
  locations.push_back(candidate_);
  for (const auto state : states) {
    locations.push_back(state->candidate());
  }

  // Route
  labelset_ = std::make_shared<LabelSet>(std::ceil(max_route_distance));
  // TODO pass labelset_ as shared_ptr
  const auto& results = find_shortest_path(
      graphreader, locations, 0, *labelset_,
      approximator, search_radius,
      costing, edgelabel, turn_cost_table);

  // Cache results
  label_idx_.clear();
  uint16_t dest = 1;  // dest at 0 is remained for the origin
  for (const auto state : states) {
    const auto it = results.find(dest);
    if (it != results.end()) {
      label_idx_[state->id()] = it->second;
    }
    dest++;
  }
}


const Label*
State::last_label(const State& state) const
{
  const auto it = label_idx_.find(state.id());
  if (it != label_idx_.end()) {
    return &labelset_->label(it->second);
  }
  return nullptr;
}


MapMatching::MapMatching(baldr::GraphReader& graphreader,
                         const sif::cost_ptr_t* mode_costing,
                         const sif::TravelMode mode,
                         float sigma_z,
                         float beta,
                         float breakage_distance,
                         float max_route_distance_factor,
                         float turn_penalty_factor)
    : graphreader_(graphreader),
      mode_costing_(mode_costing),
      mode_(mode),
      measurements_(),
      states_(),
      sigma_z_(sigma_z),
      inv_double_sq_sigma_z_(1.f / (sigma_z_ * sigma_z_ * 2.f)),
      beta_(beta),
      inv_beta_(1.f / beta_),
      breakage_distance_(breakage_distance),
      max_route_distance_factor_(max_route_distance_factor),
      turn_penalty_factor_(turn_penalty_factor),
      turn_cost_table_{0.f}
{
  if (sigma_z_ <= 0.f) {
    throw std::invalid_argument("Expect sigma_z to be positive");
  }

  if (beta_ <= 0.f) {
    throw std::invalid_argument("Expect beta to be positive");
  }

  if (turn_penalty_factor_ < 0.f) {
    throw std::invalid_argument("Expect turn penalty factor to be nonnegative");
  }

  if (0.f < turn_penalty_factor_) {
    for (int i = 0; i <= 180; ++i) {
      turn_cost_table_[i] = turn_penalty_factor_ * std::exp(-i/45.f);
    }
  }
}


MapMatching::MapMatching(baldr::GraphReader& graphreader,
                         const sif::cost_ptr_t* mode_costing,
                         const sif::TravelMode mode,
                         const boost::property_tree::ptree& config)
    : MapMatching(graphreader, mode_costing, mode,
                  config.get<float>("sigma_z"),
                  config.get<float>("beta"),
                  config.get<float>("breakage_distance"),
                  config.get<float>("max_route_distance_factor"),
                  config.get<float>("turn_penalty_factor")) {}


MapMatching::~MapMatching()
{ Clear(); }


void
MapMatching::Clear()
{
  measurements_.clear();
  states_.clear();
  ViterbiSearch<State>::Clear();
}


inline float
MapMatching::MaxRouteDistance(const State& left, const State& right) const
{
  const auto mmt_distance = GreatCircleDistance(measurement(left), measurement(right));
  return std::min(mmt_distance * max_route_distance_factor_, breakage_distance_);
}


float
MapMatching::TransitionCost(const State& left, const State& right) const
{
  if (!left.routed()) {
    std::shared_ptr<const sif::EdgeLabel> edgelabel;
    const auto prev_stateid = predecessor(left.id());
    if (prev_stateid != kInvalidStateId) {
      const auto& prev_state = state(prev_stateid);
      if (!prev_state.routed()) {
        // When ViterbiSearch calls this method, the left state is
        // guaranteed to be optimal, its pedecessor is therefore
        // guaranteed to be expanded (and routed). When
        // NaiveViterbiSearch calls this method, the previous column,
        // where the pedecessor of the left state stays, are
        // guaranteed to be all expanded (and routed).
        throw std::logic_error("The predecessor of current state must have been routed."
                               " Check if you have misused the TransitionCost method");
      }
      const auto label = prev_state.last_label(left);
      edgelabel = label? label->edgelabel : nullptr;
    } else {
      edgelabel = nullptr;
    }
    const midgard::DistanceApproximator approximator(measurement(right).lnglat());

    // NOTE TransitionCost is a mutable method and will change
    // cached routes of a state. We should be careful with it and
    // do not use it for purposes like getting transition cost of
    // two *arbitrary* states.
    left.route(unreached_states_[right.time()], graphreader_,
               MaxRouteDistance(left, right),
               approximator, measurement(right).search_radius(),
               costing(), edgelabel, turn_cost_table_);
  }
  // TODO: test it state.route(...); assert(state.routed());

  const auto label = left.last_label(right);
  if (label) {
    const auto mmt_distance = GreatCircleDistance(measurement(left), measurement(right));
    return CalculateTransitionCost(label->turn_cost, label->cost, mmt_distance);
  }

  return -1.f;
}


inline float
MapMatching::EmissionCost(const State& state) const
{ return CalculateEmissionCost(state.candidate().edges.front().score); }


inline double
MapMatching::CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const
{ return prev_costsofar + transition_cost + emission_cost; }

}
}
