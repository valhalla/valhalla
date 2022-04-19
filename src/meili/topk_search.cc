#include "meili/topk_search.h"

namespace valhalla {
namespace meili {

float EnlargedEmissionCostModel::operator()(const StateId& stateid) {
  // If this was a removed state bail
  const auto& original_stateid = evs_.GetOrigin(stateid);
  if (evs_.IsRemoved(original_stateid)) {
    return -1.0;
  }

  // Check for cache and compute if its not there
  auto c = cached_costs_.find(stateid);
  if (c == cached_costs_.cend()) {
    c = cached_costs_.emplace(stateid, calculate_cost(stateid, original_stateid)).first;
  }
  return c->second;
}

// a state has three status in the enlarged graph model:
// 1. clone (evs_.GetOrigin(stateid).IsValid())
// 2. origin that is been cloned (evs_.GetClone(stateid).IsValid())
// 3. origin that is not been cloned (otherwise)
float EnlargedEmissionCostModel::calculate_cost(const StateId& stateid,
                                                const StateId& original_stateid) const {
  const auto& model = evs_.original_emission_cost_model();
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

float EnlargedTransitionCostModel::operator()(const StateId& lhs, const StateId& rhs) {
  auto couple = std::make_pair(lhs, rhs);
  auto c = cached_costs_.find(couple);
  if (c == cached_costs_.cend()) {
    c = cached_costs_.emplace(couple, calculate_cost(lhs, rhs)).first;
  }
  return c->second;
}

float EnlargedTransitionCostModel::calculate_cost(const StateId& lhs, const StateId& rhs) const {
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

void EnlargedViterbiSearch::ClonePath(const std::vector<StateId>& path) {
  for (const auto& origin : path) {
    // Did we actually get to this state
    if (origin.IsValid()) {
      // Get a new id to map to and from this paths use of this candidate
      auto clone = claim_stateid_(origin.time());
      clone_[origin] = clone;
      if (!clone.IsValid()) {
        throw std::logic_error("generate invalid stateid?");
      }
      // Keep inverse mapping
      origin_[clone] = origin;

      // This candidate was not a clone this is the first use of it
      auto found = initial_origins_.find(origin);
      if (found == initial_origins_.end()) {
        initial_origins_[clone] = origin; // this use of clone to original candidate
      } // This was a use of a clone so we already had the original candidate
      else {
        initial_origins_[clone] = found->second; // this use of clone to original candidate
      }

      // remember when the cloning starts and ends
      if (clone_start_time_ == kInvalidTime || origin.time() < clone_start_time_) {
        clone_start_time_ = origin.time();
      }
      if (clone_end_time_ == kInvalidTime || clone_end_time_ < origin.time()) {
        clone_end_time_ = origin.time();
      }
    }
  }

  // Add the clones to vs_
  for (const auto& pair : clone_) {
    const auto added = vs_.AddStateId(pair.second);
    if (!added) {
      std::runtime_error("generated clone state IDs must be unique");
    }
  }
}

void TopKSearch::RemovePath(const std::vector<StateId>& path) {
  // A lambda for generating new claimed ids to use as a mapping
  auto claim = [this](const StateId::Time& time) {
    const auto it = last_claimed_stateids_.emplace(time, std::numeric_limits<StateId::Id>::max());
    if (!it.second) {
      it.first->second--;
    }
    return StateId(time, it.first->second);
  };

  // Create a new enlarged viterbi search
  evss_.emplace_back(new EnlargedViterbiSearch(vs_, claim, initial_origins_, removed_origins_));

  // Have it clone the current path
  evss_.back()->ClonePath(path);
}

StateId TopKSearch::GetOrigin(const StateId& stateid, const StateId& not_found) const {
  auto found = initial_origins_.find(stateid);
  if (found == initial_origins_.end()) {
    return not_found;
  }
  return found->second;
}

} // namespace meili
} // namespace valhalla
