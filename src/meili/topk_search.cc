#include "meili/topk_search.h"

namespace valhalla {
namespace meili {

EnlargedEmissionCostModel::operator()(const StateId& stateid) const
{
  const auto& cloned_stateid = ts_.GetClonedStateId(stateid);
  // TODO remove stateid by returning invalid cost here
  if (cloned_stateid.IsValid()) {
    if (cloned_stateid.time() == 0) {
      return -1.0;
    } else {
      return vs_.original_emission_cost_model()(ts_.GetOriginalStateId(cloned_stateid));
    }
  } else {
    return vs_.original_emission_cost_model()(ts_.GetOriginalStateId(cloned_stateid));
  }
}

EnlargedTransitionCostModel::operator()(const StateId& lhs, const StateId& rhs) const
{
  const auto& original_lhs = ts_.GetOriginalStateId(lhs);
  const auto& original_rhs = ts_.GetOriginalStateId(rhs);
  // TODO build edges between stateids
  return vs_.original_transition_cost_model()(original_lhs, original_rhs);
}

StateId TopKSearch::GetOriginalStateId(const StateId& stateid) const
{
  const auto it = original_stateid_.find(stateid);
  if (it == original_stateid_.end()) {
    return stateid;
  } else {
    return it->second;
  }
}

StateId TopKSearch::GetClonedStateId(const StateId& stateid) const
{
  const auto it = cloned_stateid_.find(stateid);
  if (it == cloned_stateid_.end()) {
    return StateId();
  } else {
    return it->second;
  }
}

void TopKSearch::RemovePath(const StateId::Time& time)
{
  for (auto it = vs_.SearchPath(time); it != vs_.PathEnd(); it++) {
    // TODO: cloned_stateid(it->time(), UUID())
    StateId cloned_stateid;
    cloned_stateids_[cloned_stateid] = *it;
  }
  // Add the cloned stateids to vs_
  for (const auto& pair: cloned_stateids_) {
    vs_.AddStateId(pair.first);
  }
}

}
}
