#ifndef MMP_TOPK_SEAECH_H_
#define MMP_TOPK_SEAECH_H_

#include <valhalla/meili/viterbi_search.h>

namespace valhalla {
namespace meili {

class TopKSearch;

class EnlargedEmissionCostModel
{
 public:
  EnlargedEmissionCostModel(const TopKSearch& ts)
      :ts_(ts) {}

  float operator() (const StateId& stateid) const;

 private:
  const TopKSearch& ts_;
};


class EnlargedTransitionCostModel
{
 public:
  EnlargedTransitionCostModel(const TopKSearch& ts)
      :ts_(ts) {}

  float operator() (const StateId& lhs, const StateId& rhs) const;

 private:
  const TopKSearch& ts_;
};

class TopKSearch
{
  TopKSearch(IViterbiSearch& vs)
      : vs_(vs),
        original_emission_cost_model_(vs_.emission_cost_model()),
        original_transition_cost_model_(vs_.transition_cost_model()),
        cloned_stateid_(),
        original_stateid_() {}

  const IEmissionCostModel& original_emission_cost_model() const
  { return original_emission_cost_model_; }

  const ITransitionCostModel& original_transition_cost_model() const
  { return original_transition_cost_model_; }

  StateId GetOriginalStateId(const StateId& stateid) const;

  StateId GetClonedStateId(const StateId& stateid) const;

  void RemovePath(const StateId::Time& time);

 private:
  IViterbiSearch& vs_;

  const IEmissionCostModel& original_emission_cost_model_;

  const ITransitionCostModel& original_transition_cost_model_;

  std::unordered_map<StateId, StateId> cloned_stateid_;

  std::unordered_map<StateId, StateId> original_stateid_;
};

}
}

#endif /* MMP_TOPK_SEAECH_H_ */
