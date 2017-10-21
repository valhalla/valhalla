#ifndef MMP_TOPK_SEAECH_H_
#define MMP_TOPK_SEAECH_H_

#include <unordered_map>
#include <functional>
#include <list>

#include <valhalla/meili/viterbi_search.h>
#include <valhalla/meili/stateid.h>

namespace valhalla {
namespace meili {

class EnlargedViterbiSearch;

class EnlargedEmissionCostModel
{
 public:
  EnlargedEmissionCostModel(const EnlargedViterbiSearch& evs)
      :evs_(evs) {}

  float operator() (const StateId& stateid) const;

 private:
  const EnlargedViterbiSearch& evs_;
};


class EnlargedTransitionCostModel
{
 public:
  EnlargedTransitionCostModel(const EnlargedViterbiSearch& evs)
      :evs_(evs) {}

  float operator() (const StateId& lhs, const StateId& rhs) const;

 private:
  const EnlargedViterbiSearch& evs_;
};

class EnlargedViterbiSearch
{
 public:
  EnlargedViterbiSearch(IViterbiSearch& vs, std::function<StateId(const StateId::Time&)> claim_stateid)
      : vs_(vs),
        claim_stateid_(claim_stateid),
        original_emission_cost_model_(vs.emission_cost_model()),
        original_transition_cost_model_(vs.transition_cost_model()),
        origin_(),
        clone_(),
        clone_start_time_(kInvalidTime),
        clone_end_time_(kInvalidTime)
  {
    vs_.set_emission_cost_model(EnlargedEmissionCostModel(*this));
    vs_.set_transition_cost_model(EnlargedTransitionCostModel(*this));
  }

  const IEmissionCostModel& original_emission_cost_model() const
  { return original_emission_cost_model_; }

  const ITransitionCostModel& original_transition_cost_model() const
  { return original_transition_cost_model_; }

  StateId GetOrigin(const StateId& stateid) const
  {
    const auto it = origin_.find(stateid);
    if (it == origin_.end()) {
      return {};
    } else {
      return it->second;
    }
  }

  StateId GetClone(const StateId& stateid) const
  {
    const auto it = clone_.find(stateid);
    if (it == clone_.end()) {
      return {};
    } else {
      return it->second;
    }
  }

  void ClonePath(const StateId::Time& time);

  const StateId::Time& clone_start_time() const
  { return clone_start_time_; }

  const StateId::Time& clone_end_time() const
  { return clone_end_time_; }

 private:
  IViterbiSearch& vs_;

  // a function that gurantees to generate a NEW stateid at the specified time
  std::function<StateId(const StateId::Time& time)> claim_stateid_;

  // vs_'s orignal emission cost model before it's been enlarged
  IEmissionCostModel original_emission_cost_model_;

  // vs_'s original transition cost model before it's been enlarged
  ITransitionCostModel original_transition_cost_model_;

  // clone -> origin
  std::unordered_map<StateId, StateId> origin_;

  // origin -> clone
  std::unordered_map<StateId, StateId> clone_;

  StateId::Time clone_start_time_, clone_end_time_;
};

class TopKSearch
{
 public:
  TopKSearch(IViterbiSearch& vs)
      : vs_(vs),
        last_claimed_stateids_(),
        evss_() {}

  void Clear()
  {
    last_claimed_stateids_.clear();
    evss_.clear();
  }

  // remove path from 0 to time
  void RemovePath(const StateId::Time& time);

  // find corresponding origin stateid recursively
  StateId GetOrigin(const StateId& stateid) const;

 private:
  IViterbiSearch& vs_;

  std::unordered_map<StateId::Time, uint32_t> last_claimed_stateids_;

  // to not invalidate references of evs we use list
  std::list<EnlargedViterbiSearch> evss_;
};

}
}

#endif /* MMP_TOPK_SEAECH_H_ */
