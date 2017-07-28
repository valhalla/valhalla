#include "midgard/logging.h"
#include "baldr/pathlocation.h"

#include "meili/candidate_search.h"
#include "meili/routing.h"
#include "meili/match_result.h"
#include "meili/map_matching.h"

namespace {

inline float
GreatCircleDistance(const valhalla::meili::Measurement& left,
                    const valhalla::meili::Measurement& right)
{ return left.lnglat().Distance(right.lnglat()); }

inline float ClockDistance(const valhalla::meili::Measurement& left,
                    const valhalla::meili::Measurement& right)
{ return right.epoch_time() - left.epoch_time(); }

}


namespace valhalla {
namespace meili {

void
State::route(const std::vector<State>& states,
             baldr::GraphReader& graphreader,
             float max_route_distance,
             float max_route_time,
             const midgard::DistanceApproximator& approximator,
             const float search_radius,
             sif::cost_ptr_t costing,
             const Label* edgelabel,
             const float turn_cost_table[181]) const
{
  // Prepare locations
  std::vector<baldr::PathLocation> locations;
  locations.reserve(1 + states.size());
  locations.push_back(candidate_);
  for (const auto state : states) {
    locations.push_back(state.candidate());
  }

  // Route
  labelset_ = std::make_shared<LabelSet>(std::ceil(max_route_distance));
  const auto& results = find_shortest_path(
      graphreader, locations, 0, labelset_,
      approximator, search_radius,
      costing, edgelabel, turn_cost_table,
      std::ceil(max_route_distance), std::ceil(max_route_time));

  // Cache results
  label_idx_.clear();
  uint16_t dest = 1;  // dest at 0 is remained for the origin
  for (const auto state : states) {
    const auto it = results.find(dest);
    if (it != results.end()) {
      label_idx_[state.stateid()] = it->second;
    }
    dest++;
  }
}


const Label*
State::last_label(const State& state) const
{
  const auto it = label_idx_.find(state.stateid());
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
                         float max_route_time_factor,
                         float turn_penalty_factor)
    : graphreader_(graphreader),
      mode_costing_(mode_costing),
      mode_(mode),
      measurements_(),
      columns_(),
      sigma_z_(sigma_z),
      inv_double_sq_sigma_z_(1.f / (sigma_z_ * sigma_z_ * 2.f)),
      beta_(beta),
      inv_beta_(1.f / beta_),
      breakage_distance_(breakage_distance),
      max_route_distance_factor_(max_route_distance_factor),
      max_route_time_factor_(max_route_time_factor),
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
                  config.get<float>("max_route_time_factor"),
                  config.get<float>("turn_penalty_factor")) {}


MapMatching::~MapMatching()
{ Clear(); }


void
MapMatching::Clear()
{
  measurements_.clear();
  columns_.clear();
  ViterbiSearch::Clear();
}

float
MapMatching::TransitionCost(const StateId& lhs, const StateId& rhs) const
{
  // Get some basic info about difference between the two measurements
  const auto& left = state(lhs);
  const auto& right = state(rhs);
  const auto& left_measurement = measurement(left);
  const auto& right_measurement = measurement(right);
  const auto gc_dist = GreatCircleDistance(left_measurement, right_measurement);
  const auto clk_dist = ClockDistance(left_measurement, right_measurement);

  // If we need to actually compute the route
  if (!left.routed()) {
    const Label* edgelabel = nullptr;
    const auto prev_stateid = Predecessor(lhs);
    if (prev_stateid.IsValid()) {
      const auto& prev_state = state(prev_stateid);
      if (!prev_state.routed()) {
        // When ViterbiSearch calls this method, the left state is
        // guaranteed to be optimal, its predecessor is therefore
        // guaranteed to be expanded (and routed). When
        // NaiveViterbiSearch calls this method, the previous column,
        // where the predecessor of the left state stays, are
        // guaranteed to be all expanded (and routed).
        throw std::logic_error("The predecessor of current state must have been routed."
                               " Check if you have misused the TransitionCost method");
      }
      edgelabel = prev_state.last_label(left);
    }

    // TODO: inefficient; should move State::route to MapMatching
    std::vector<State> unreached_states;
    for (const auto& stateid : unreached_states_[right.stateid().time()]) {
      unreached_states.push_back(state(stateid));
    }

    // NOTE TransitionCost is a mutable method and will change
    // cached routes of a state. We should be careful with it and
    // do not use it for purposes like getting transition cost of
    // two *arbitrary* states.
    const midgard::DistanceApproximator approximator(measurement(right).lnglat());
    left.route(unreached_states, graphreader_,
               std::min(gc_dist * max_route_distance_factor_, breakage_distance_),
               clk_dist * max_route_time_factor_,
               approximator, measurement(right).search_radius(),
               costing(), edgelabel, turn_cost_table_);
  }
  // TODO: test it state.route(...); assert(state.routed());

  // Compute the transition cost if we found a path
  const auto label = left.last_label(right);
  if (label) {
    return CalculateTransitionCost(label->turn_cost(), label->cost().cost, gc_dist, label->cost().secs, clk_dist);
  }

  // No path found
  return -1.f;
}


inline float
MapMatching::EmissionCost(const StateId& stateid) const
{ return CalculateEmissionCost(state(stateid).candidate().edges.front().score); }


inline double
MapMatching::CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const
{ return prev_costsofar + transition_cost + emission_cost; }

}
}
