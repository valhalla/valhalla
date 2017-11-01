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
    // remove the last clone
    if (stateid.time() == evs_.clone_end_time()) {
      return -1.0;
    } else {
      return model(original_stateid);
    }
  }
  // remove the first cloned origin
  if (stateid.time() == evs_.clone_start_time() && evs_.GetClone(stateid).IsValid()) {
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
        return model(original_lhs, rhs);
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
      clone_[origin] = claim_stateid_(origin.time());
      if (!clone_[origin].IsValid()) {
        throw std::logic_error("generate invalid stateid?");
      }
      origin_[clone_[origin]] = origin;

      // remember when the cloning starts and ends
      if (clone_start_time_ == kInvalidTime || origin.time() < clone_start_time_) {
        clone_start_time_ = origin.time();
      }
      if (clone_end_time_ == kInvalidTime || clone_end_time_ < origin.time()) {
        clone_end_time_ = origin.time();
      }
    }
  }
  vs_.ClearSearch();

  // Add the clones to vs_
  for (const auto& pair: clone_) {
    const auto added = vs_.AddStateId(pair.second);
    if (!added) {
      std::runtime_error("generated clone state IDs must be unique");
    }
  }
}

void TopKSearch::RemovePath(const StateId::Time& time)
{
  evss_.emplace_back(vs_, [this](const StateId::Time& time) {
      const auto it = last_claimed_stateids_.emplace(
          time,
          std::numeric_limits<StateId::Id>::max());
      if (!it.second) {
        it.first->second --;
      }
      return StateId(time, it.first->second);
    });
  evss_.back().ClonePath(time);
}

StateId TopKSearch::GetOrigin(const StateId& stateid) const
{
  StateId last_valid_origin, current = stateid;

  // we are not sure stateid was cloned in which graph, so we try recursively
  for (auto it = evss_.rbegin(); it != evss_.rend(); it++) {
    const auto& origin = it->GetOrigin(current);
    if (origin.IsValid()) {
      current = origin;
      last_valid_origin = origin;
    } // otherwise current is a clone in the other graphs
  }

  return last_valid_origin;
}

}
}
