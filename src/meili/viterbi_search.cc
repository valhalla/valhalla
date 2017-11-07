#include <algorithm>
#include <string>

#include "meili/viterbi_search.h"

namespace valhalla {
namespace meili {

void StateIdIterator::Next()
{
  ValidateStateId(time_, stateid_);

  // We're done searching between states if time == 0 meaning we found the last one or
  // we are at a state without a path to it but aren't allowing breaks in the path
  if(0 == time_ ||
      (stateid_.IsValid() && !(stateid_ = vs_.Predecessor(stateid_)).IsValid() && !allow_breaks_)) {
    time_ = kInvalidTime;
    stateid_ = StateId();
    return;
  }

  // Search at previous time directly
  // used in cloning path so no need to enforce a continuous path
  --time_;
  if (!stateid_.IsValid())
    stateid_ = vs_.SearchWinner(time_);
}

template <bool Maximize>
void NaiveViterbiSearch<Maximize>::Clear()
{
  IViterbiSearch::Clear();
  states_.clear();
  ClearSearch();
}

template <bool Maximize>
void NaiveViterbiSearch<Maximize>::ClearSearch()
{
  history_.clear();
  winner_.clear();
}

template <bool Maximize>
bool NaiveViterbiSearch<Maximize>::AddStateId(const StateId& stateid)
{
  if (!IViterbiSearch::AddStateId(stateid)) {
    return false;
  }

  while (states_.size() <= stateid.time()) {
    states_.emplace_back();
  }
  states_[stateid.time()].push_back(stateid);

  return true;
}

template <bool Maximize>
inline double
NaiveViterbiSearch<Maximize>::AccumulatedCost(const StateId& stateid) const
{ return stateid.IsValid() ? GetLabel(stateid).costsofar() : kInvalidCost; }

template <bool Maximize>
StateId NaiveViterbiSearch<Maximize>::SearchWinner(StateId::Time target)
{
  if (states_.size() <= target) {
    return {};
  }

  // Use the cache
  if (target < winner_.size()) {
    return winner_[target];
  }

  for (StateId::Time time = winner_.size(); time <= target; ++time) {
    const auto& column = states_[time];
    std::vector<StateLabel> labels;

    // Update labels
    if (time == 0) {
      labels = InitLabels(column, true);
    } else {
      labels = InitLabels(column, false);
      UpdateLabels(labels, history_.back());
    }

    auto winner = FindWinner(labels);
    if (!winner.IsValid() && 0 < time) {
      // If it's not reachable by prevous column, we find the winner
      // with the best emission cost only
      labels = InitLabels(column, true);
      winner = FindWinner(labels);
    }
    winner_.push_back(winner);
    history_.push_back(labels);
  }

  return winner_[target];
}

template <bool Maximize>
inline StateId
NaiveViterbiSearch<Maximize>::Predecessor(const StateId& stateid) const
{ return stateid.IsValid() ? GetLabel(stateid).predecessor() : StateId(); }

template <bool Maximize>
void NaiveViterbiSearch<Maximize>::UpdateLabels(
    std::vector<StateLabel>& labels,
    const std::vector<StateLabel>& prev_labels) const
{
  for (const auto& prev_label : prev_labels) {
    const auto& prev_stateid = prev_label.stateid();

    const auto prev_costsofar = prev_label.costsofar();
    if (kInvalidCost == prev_costsofar) {
      continue;
    }

    for (auto& label : labels) {
      const auto stateid = label.stateid();

      const auto emission_cost = EmissionCost(stateid);
      if (kInvalidCost == emission_cost) {
        continue;
      }

      const auto transition_cost = TransitionCost(prev_stateid, stateid);
      if (kInvalidCost == transition_cost) {
        continue;
      }

      const auto costsofar = CostSofar(prev_costsofar, transition_cost, emission_cost);
      if (kInvalidCost == costsofar) {
        continue;
      }

      if (Maximize) {
        label = std::max(StateLabel(costsofar, stateid, prev_stateid), label);
      } else {
        label = std::min(StateLabel(costsofar, stateid, prev_stateid), label);
      }
    }
  }
}

template <bool Maximize>
std::vector<StateLabel>
NaiveViterbiSearch<Maximize>::InitLabels(
    const std::vector<StateId>& column,
    bool use_emission_cost) const
{
  std::vector<StateLabel> labels;
  for (const auto& stateid: column) {
    labels.emplace_back(
        use_emission_cost ? EmissionCost(stateid) : kInvalidCost,
        stateid,
        StateId());
  }
  return labels;
}

template <bool Maximize>
StateId
NaiveViterbiSearch<Maximize>::FindWinner(const std::vector<StateLabel>& labels) const
{
  const auto cmp = [](const StateLabel& lhs, const StateLabel& rhs) {
    return lhs.costsofar() < rhs.costsofar();
  };

  auto it = labels.cend();
  if (Maximize) {
    it = std::max_element(labels.cbegin(), labels.cend(), cmp);
  } else {
    it = std::min_element(labels.cbegin(), labels.cend(), cmp);
  }

  // The max label's costsofar is invalid (-infinity), that means all
  // labels are invalid
  if (it == labels.cend() || it->costsofar() == kInvalidCost) {
    return {};
  }

  return it->stateid();
}

// Linear search a state's label
template <bool Maximize>
const StateLabel&
NaiveViterbiSearch<Maximize>::GetLabel(const StateId& stateid) const
{
  const auto& labels = history_[stateid.time()];
  const auto it = std::find_if(
      labels.cbegin(),
      labels.cend(),
      [&stateid](const StateLabel& label) {
        return label.stateid() == stateid;
      });
  if (it == labels.end()) {
    throw std::runtime_error("impossible that label not found; if it happened, check SearchWinner");
  }
  return *it;
}

template class NaiveViterbiSearch<true>;
template class NaiveViterbiSearch<false>;

bool ViterbiSearch::AddStateId(const StateId& stateid)
{
  if (!IViterbiSearch::AddStateId(stateid)) {
    return false;
  }

  while (states_.size() <= stateid.time()) {
    states_.emplace_back();
  }
  states_[stateid.time()].push_back(stateid);

  while (unreached_states_.size() <= stateid.time()) {
    unreached_states_.emplace_back();
  }
  unreached_states_[stateid.time()].push_back(stateid);

  return true;
}

StateId ViterbiSearch::SearchWinner(StateId::Time time)
{
  // Use the cache
  if (time < winner_.size()) {
    return winner_[time];
  }

  if (unreached_states_.empty()) {
    return {};
  }

  const StateId::Time max_allowed_time = unreached_states_.size() - 1;
  const auto target = std::min(time, max_allowed_time);

  // Continue last search if possible
  StateId::Time searched_time = IterativeSearch(target, false);

  while (searched_time < target) {
    // searched_time < target implies that there was a breakage during
    // last search, so we request a new start
    searched_time = IterativeSearch(target, true);
    // Guarantee that that winner_.size() is increasing and searched_time == winner_.size() - 1
  }

  if (time < winner_.size()) {
    return winner_[time];
  }

  return {};
}

StateId
ViterbiSearch::Predecessor(const StateId& stateid) const
{
  const auto it = scanned_labels_.find(stateid);
  if (it == scanned_labels_.end()) {
    return {};
  } else {
    return (it->second).predecessor();
  }
}

double ViterbiSearch::AccumulatedCost(const StateId& stateid) const
{
  const auto it = scanned_labels_.find(stateid);
  if (it == scanned_labels_.end()) {
    return -1.f;
  } else {
    return it->second.costsofar();
  }
}

void ViterbiSearch::Clear()
{
  IViterbiSearch::Clear();
  states_.clear();
  ClearSearch();
}

void ViterbiSearch::ClearSearch()
{
  earliest_time_ = 0;
  queue_.clear();
  scanned_labels_.clear();
  winner_.clear();
  unreached_states_ = states_;
}

void ViterbiSearch::InitQueue(const std::vector<StateId>& column)
{
  queue_.clear();
  for (const auto stateid : column) {
    const auto emission_cost = EmissionCost(stateid);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }
    queue_.push(StateLabel(emission_cost, stateid, {}));
  }
}

void ViterbiSearch::AddSuccessorsToQueue(const StateId& stateid)
{
  if (!(stateid.time() + 1 < unreached_states_.size())) {
    throw std::logic_error("the state at time " + std::to_string(stateid.time()) + " is impossible to have successors");
  }

  const auto it = scanned_labels_.find(stateid);
  if (it == scanned_labels_.end()) {
    throw std::logic_error("the state must be scanned");
  }
  const auto costsofar = it->second.costsofar();
  if (IsInvalidCost(costsofar)) {
    // All invalid ones should be filtered out before pushing labels
    // into the queue
    throw std::logic_error("impossible to get invalid cost from scanned labels");
  }

  // Optimal states have been removed from unreached_states_ so no
  // worry about optimality
  for (const auto& next_stateid : unreached_states_[stateid.time() + 1]) {
    const auto emission_cost = EmissionCost(next_stateid);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }

    const auto transition_cost = TransitionCost(stateid, next_stateid);
    if (IsInvalidCost(transition_cost)) {
      continue;
    }

    const auto next_costsofar = CostSofar(costsofar, transition_cost,  emission_cost);
    if (IsInvalidCost(next_costsofar)) {
      continue;
    }

    queue_.push(StateLabel(next_costsofar, next_stateid, stateid));
  }
}

StateId::Time ViterbiSearch::IterativeSearch(StateId::Time target, bool request_new_start)
{
  if (unreached_states_.size() <= target) {
    if (unreached_states_.empty()) {
      throw std::runtime_error(
          "empty states: add some states at least before searching");
    } else {
      throw std::runtime_error(
          "the target time is beyond the maximum allowed time "
          + std::to_string(unreached_states_.size()-1));
    }
  }

  // Do nothing since the winner at the target time is already known
  if (target < winner_.size()) {
    return target;
  }

  // Clearly here we have precondition: winner_.size() <= target < unreached_states_.size()

  StateId::Time source;

  // Either continue last search, or start a new search
  if (!request_new_start && !winner_.empty() && winner_.back().IsValid()) {
    source = winner_.size() - 1;
    AddSuccessorsToQueue(winner_[source]);
  } else {
    source = winner_.size();
    InitQueue(unreached_states_[source]);
  }

  // Start with the source time, which will be searched anyhow
  auto searched_time = source;

  while (!queue_.empty()) {
    // Pop up the state with the optimal cost. Note it is not
    // necessarily to be the winner at its time yet, unless it is the
    // first one found at the time
    const auto label = queue_.top();
    const auto& stateid = label.stateid();
    queue_.pop();

    // Skip labels that are earlier than the earliest time, since they
    // are impossible to be part of the path to future winners
    if (stateid.time() < earliest_time_) {
      continue;
    }

    // Mark it as scanned and remember its cost and predecessor
    const auto& inserted = scanned_labels_.emplace(stateid, label);
    if (!inserted.second) {
      throw std::logic_error(
          "the principle of optimality is violated in the viterbi search,"
          " probably negative costs occurred");
    }

    // Remove it from its column
    auto& column = unreached_states_[stateid.time()];
    const auto it = std::find(column.begin(), column.end(), stateid);
    if (it == column.end()) {
      throw std::logic_error("the state must exist in the column");
    }
    column.erase(it);

    // Since current column is empty now, earlier labels can't reach
    // future winners in a optimal way any more, so we mark time + 1
    // as the earliest time to skip all earlier labels
    if (column.empty()) {
      earliest_time_ = stateid.time() + 1;
    }

    // If it's the first state that arrives at this column, mark it as
    // the winner at this time
    if (winner_.size() <= stateid.time()) {
      if (!(stateid.time() == winner_.size())) {
        // Should check if states at unreached_states_[time] are all
        // at the same TIME
        throw std::logic_error("found a state from the future time " + std::to_string(stateid.time()));
      }
      winner_.push_back(stateid);
    }

    // Update searched time
    searched_time = std::max(stateid.time(), searched_time);

    // Break immediately when the winner at the target time is found.
    // We will add its successors to queue at next search
    if (target <= searched_time) {
      break;
    }

    AddSuccessorsToQueue(stateid);
  }

  // Guarantee that either winner (if found) or invalid stateid (not
  // found) is saved at searched_time
  while (winner_.size() <= searched_time) {
    winner_.emplace_back();
  }

  // Postcondition: searched_time == winner_.size() - 1 && search_time <= target

  // If search_time < target it implies that there is a breakage,
  // i.e. unable to find any connection from the column at search_time
  // to the column at search_time + 1

  return searched_time;
}

}
}
