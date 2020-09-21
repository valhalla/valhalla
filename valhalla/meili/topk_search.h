#ifndef MMP_TOPK_SEAECH_H_
#define MMP_TOPK_SEAECH_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <valhalla/meili/stateid.h>
#include <valhalla/meili/viterbi_search.h>

namespace valhalla {
namespace meili {

class EnlargedViterbiSearch;

class EnlargedEmissionCostModel {
public:
  EnlargedEmissionCostModel(const EnlargedViterbiSearch& evs) : evs_(evs) {
  }

  float operator()(const StateId& stateid);

private:
  float calculate_cost(const StateId& stateid, const StateId& original_stateid) const;

  const EnlargedViterbiSearch& evs_;
  std::unordered_map<StateId, float> cached_costs_;
};

class EnlargedTransitionCostModel {
public:
  EnlargedTransitionCostModel(const EnlargedViterbiSearch& evs) : evs_(evs) {
  }

  float operator()(const StateId& lhs, const StateId& rhs);

private:
  float calculate_cost(const StateId& lhs, const StateId& rhs) const;

  const EnlargedViterbiSearch& evs_;
  std::unordered_map<std::pair<StateId, StateId>, float> cached_costs_;
};

class EnlargedViterbiSearch {
public:
  EnlargedViterbiSearch(IViterbiSearch& vs,
                        std::function<StateId(const StateId::Time&)> claim_stateid,
                        std::unordered_map<StateId, StateId>& initial_origins,
                        std::unordered_set<StateId>& removed_origins)
      : vs_(vs), claim_stateid_(claim_stateid),
        original_emission_cost_model_(vs.emission_cost_model()),
        original_transition_cost_model_(vs.transition_cost_model()), origin_(), clone_(),
        initial_origins_(initial_origins), removed_origins_(removed_origins),
        clone_start_time_(kInvalidTime), clone_end_time_(kInvalidTime) {
    vs_.set_emission_cost_model(EnlargedEmissionCostModel(*this));
    vs_.set_transition_cost_model(EnlargedTransitionCostModel(*this));
  }

  const IEmissionCostModel& original_emission_cost_model() const {
    return original_emission_cost_model_;
  }

  const ITransitionCostModel& original_transition_cost_model() const {
    return original_transition_cost_model_;
  }

  StateId GetOrigin(const StateId& stateid) const {
    const auto it = origin_.find(stateid);
    if (it == origin_.end()) {
      return {};
    } else {
      return it->second;
    }
  }

  StateId GetClone(const StateId& stateid) const {
    const auto it = clone_.find(stateid);
    if (it == clone_.end()) {
      return {};
    } else {
      return it->second;
    }
  }

  bool IsRemoved(const StateId& stateid) const {
    return removed_origins_.find(stateid) != removed_origins_.cend();
  }

  void ClonePath(const std::vector<StateId>& path);

  const StateId::Time& clone_start_time() const {
    return clone_start_time_;
  }

  const StateId::Time& clone_end_time() const {
    return clone_end_time_;
  }

private:
  IViterbiSearch& vs_;

  // a function that guarantees to generate a NEW stateid at the specified time
  std::function<StateId(const StateId::Time& time)> claim_stateid_;

  // vs_'s original emission cost model before it's been enlarged
  IEmissionCostModel original_emission_cost_model_;

  // vs_'s original transition cost model before it's been enlarged
  ITransitionCostModel original_transition_cost_model_;

  // clone -> origin
  std::unordered_map<StateId, StateId> origin_;

  // origin -> clone
  std::unordered_map<StateId, StateId> clone_;

  // clone -> root origin
  std::unordered_map<StateId, StateId>& initial_origins_;

  // origins that were removed
  std::unordered_set<StateId>& removed_origins_;

  StateId::Time clone_start_time_, clone_end_time_;
};

class TopKSearch {
public:
  TopKSearch(IViterbiSearch& vs) : vs_(vs) {
  }

  void Clear() {
    last_claimed_stateids_.clear();
    evss_.clear();
    initial_origins_.clear();
    removed_origins_.clear();
  }

  // remove path from first state to last state
  void RemovePath(const std::vector<StateId>& path);

  // find corresponding origin stateid recursively
  StateId GetOrigin(const StateId& stateid, const StateId& not_found = {}) const;

  void RemoveStateId(const StateId& stateid) {
    removed_origins_.emplace(stateid);
  }

  bool IsRemoved(const StateId& stateid) const {
    return removed_origins_.find(stateid) != removed_origins_.cend();
  }

private:
  IViterbiSearch& vs_;

  std::unordered_map<StateId::Time, uint32_t> last_claimed_stateids_;

  // to not invalidate references of evs we use pointers
  std::vector<std::unique_ptr<EnlargedViterbiSearch>> evss_;

  std::unordered_map<StateId, StateId> initial_origins_;

  std::unordered_set<StateId> removed_origins_;
};

} // namespace meili
} // namespace valhalla

#endif /* MMP_TOPK_SEAECH_H_ */
