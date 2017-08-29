#ifndef MMP_TOPK_SEAECH_H_
#define MMP_TOPK_SEAECH_H_

#include <valhalla/meili/viterbi_search.h>

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
  EnlargedViterbiSearch(IViterbiSearch& vs)
      : vs_(vs),
        original_emission_cost_model_(vs.emission_cost_model()),
        original_transition_cost_model_(vs.transition_cost_model()),
        origin_(),
        clone_()
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

 private:
  IViterbiSearch& vs_;

  const IEmissionCostModel& original_emission_cost_model_;

  const ITransitionCostModel& original_transition_cost_model_;

  // clone -> origin
  std::unordered_map<StateId, StateId> origin_;

  // origin -> clone
  std::unordered_map<StateId, StateId> clone_;
};

class TopKSearch
{
 public:
  TopKSearch(IViterbiSearch& vs)
      : vs_(vs),
        evss_() {}

  void RemovePath(const StateId::Time& time);

 private:
  IViterbiSearch& vs_;

  std::vector<EnlargedViterbiSearch> evss_;
};

}
}

#endif /* MMP_TOPK_SEAECH_H_ */
