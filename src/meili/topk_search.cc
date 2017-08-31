#include "meili/topk_search.h"

namespace valhalla {
namespace meili {

// a state has three status in the enlarged graph model:
// 1. clone (evs_.GetOrigin(stateid).IsValid())
// 2. origin that is been cloned (evs_.GetClone(stateid).IsValid())
// 3. origin that is not been cloned (otherwise)
float EnlargedEmissionCostModel::operator()(const StateId& stateid) const
{
  const auto& model = evs_.original_emission_cost_model();
  const auto& original_stateid = evs_.GetOrigin(stateid);
  if (original_stateid.IsValid()) {
    return model(original_stateid);
  }
  // stateid that is been cloned at the intial time should be removed
  if (stateid.time() == 0 && evs_.GetClone(stateid).IsValid()) {
    return -1.0;
  } else {
    return model(stateid);
  }
}

float EnlargedTransitionCostModel::operator()(const StateId& lhs, const StateId& rhs) const
{
  const auto& model = evs_.original_transition_cost_model();
  const auto& original_lhs = evs_.GetOrigin(lhs);
  const auto& original_rhs = evs_.GetOrigin(rhs);
  if (original_lhs.IsValid()) {
    if (original_rhs.IsValid()) {
      return model(original_lhs, original_rhs);
    } else {
      if (evs_.GetClone(rhs).IsValid()) {
        return -1.0;
      } else {
        return model(original_rhs, rhs);
      }
    }
  } else {
    if (original_rhs.IsValid()) {
      return -1.0;
    } else {
      return model(lhs, rhs);
    }
  }
}

void EnlargedViterbiSearch::ClonePath(const StateId::Time& time)
{
  for (auto it = vs_.SearchPath(time); it != vs_.PathEnd(); it++) {
    const auto& origin = *it;
    if (origin.IsValid()) {
      StateId clone = claim_stateid_(time);
      origin_[clone] = origin;
      clone_[origin] = clone;
    }
  }

  // Add the clones to vs_
  for (const auto& pair: origin_) {
    vs_.AddStateId(pair.first);
  }
}

void TopKSearch::RemovePath(const StateId::Time& time)
{
  vs_.ClearSearch();
  evss_.emplace_back(vs_, [this](const StateId::Time& time) {
      const auto it = last_claimed_stateids_.emplace(
          time,
          std::numeric_limits<float>::max());
      if (!it.second) {
        it.first->second --;
      }
      return StateId(time, it.first->second);
    });
  evss_.back().ClonePath(time);
}

}
}
