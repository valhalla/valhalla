#include "meili/viterbi_search.h"

#include <algorithm>
#include <string>

namespace valhalla {
namespace meili {

StateLabel::StateLabel(double costsofar, const StateId& stateid, const StateId& predecessor)
    : costsofar_(costsofar), stateid_(stateid), predecessor_(predecessor) {
  if (!stateid.IsValid()) {
    throw std::invalid_argument("expect valid stateid");
  }
}

StateId StateLabel::id() const {
  return stateid_;
}

double StateLabel::costsofar() const {
  return costsofar_;
}

StateId StateLabel::stateid() const {
  return stateid_;
}

StateId StateLabel::predecessor() const {
  return predecessor_;
}
// Required by SPQueue
bool StateLabel::operator<(const StateLabel& rhs) const {
  return costsofar_ < rhs.costsofar_;
}
// Required by SPQueue
bool StateLabel::operator>(const StateLabel& rhs) const {
  return costsofar_ > rhs.costsofar_;
}
// Required by SPQueue
bool StateLabel::operator==(const StateLabel& rhs) const {
  return costsofar_ == rhs.costsofar_;
}

StateIdIterator::StateIdIterator(IViterbiSearch& vs,
                                 StateId::Time time,
                                 const StateId& stateid,
                                 bool allow_breaks)
    : vs_(vs), time_(time), stateid_(stateid), allow_breaks_(allow_breaks) {
  ValidateStateId(time, stateid);
}

StateIdIterator::StateIdIterator(IViterbiSearch& vs) : StateIdIterator(vs, kInvalidTime, StateId()) {
}

void StateIdIterator::Next() {
  ValidateStateId(time_, stateid_);

  // We're done searching between states if time == 0 meaning we found the last one or
  // we are at a state without a path to it but aren't allowing breaks in the path
  if (time_ == 0 ||
      (stateid_.IsValid() && !(stateid_ = vs_.Predecessor(stateid_)).IsValid() && !allow_breaks_)) {
    time_ = kInvalidTime;
    stateid_ = StateId();
    return;
  }

  // Search at previous time directly
  // used in cloning path so no need to enforce a continuous path
  --time_;
  if (!stateid_.IsValid()) {
    stateid_ = vs_.SearchWinner(time_);
  }
}

StateIdIterator StateIdIterator::operator++(int) {
  auto copy = *this;
  Next();
  return copy;
}
// Prefix increment
StateIdIterator StateIdIterator::operator++() {
  Next();
  return *this;
}

bool StateIdIterator::operator==(const StateIdIterator& other) const {
  return &vs_ == &(other.vs_) && time_ == other.time_ && stateid_ == other.stateid_;
}

bool StateIdIterator::operator!=(const StateIdIterator& other) const {
  return !(*this == other);
}

const StateId& StateIdIterator::operator*() const {
  return stateid_;
}

void StateIdIterator::ValidateStateId(const StateId::Time time, const StateId& stateid) {
  if (!stateid.IsValid()) {
    return;
  } else if (time == kInvalidTime) {
    throw std::runtime_error("expect invalid stateid");
  } else if (stateid.time() != time) {
    throw std::runtime_error("time is not matched");
  }
}

IViterbiSearch::IViterbiSearch(const IEmissionCostModel& emission_cost_model,
                               const ITransitionCostModel& transition_cost_model)
    : emission_cost_model_(emission_cost_model), transition_cost_model_(transition_cost_model),
      path_end_(*this) {
}

IViterbiSearch::IViterbiSearch()
    : IViterbiSearch(DefaultEmissionCostModel, DefaultTransitionCostModel) {
}

IViterbiSearch::~IViterbiSearch() {
  Clear();
};

void IViterbiSearch::Clear() {
  added_states_.clear();
}

bool IViterbiSearch::AddStateId(const StateId& stateid) {
  return added_states_.insert(stateid).second;
}

bool IViterbiSearch::RemoveStateId(const StateId& stateid) {
  return added_states_.erase(stateid) > 0;
}

bool IViterbiSearch::HasStateId(const StateId& stateid) const {
  return added_states_.find(stateid) != added_states_.end();
}

StateIdIterator IViterbiSearch::SearchPath(StateId::Time time, bool allow_breaks) {
  return stateid_iterator(*this, time, SearchWinner(time), allow_breaks);
}

StateIdIterator IViterbiSearch::PathEnd() const {
  return path_end_;
}

const IEmissionCostModel& IViterbiSearch::emission_cost_model() const {
  return emission_cost_model_;
}

void IViterbiSearch::set_emission_cost_model(const IEmissionCostModel cost_model) {
  emission_cost_model_ = cost_model;
}

const ITransitionCostModel& IViterbiSearch::transition_cost_model() const {
  return transition_cost_model_;
}

void IViterbiSearch::set_transition_cost_model(const ITransitionCostModel cost_model) {
  transition_cost_model_ = cost_model;
}

float IViterbiSearch::TransitionCost(const StateId& lhs, const StateId& rhs) const {
  return transition_cost_model_(lhs, rhs);
}

float IViterbiSearch::EmissionCost(const StateId& stateid) const {
  return emission_cost_model_(stateid);
}

constexpr double
IViterbiSearch::CostSofar(double prev_costsofar, float transition_cost, float emission_cost) {
  return prev_costsofar + transition_cost + emission_cost;
}

template <bool Maximize> NaiveViterbiSearch<Maximize>::~NaiveViterbiSearch() {
  Clear();
}

template <bool Maximize> void NaiveViterbiSearch<Maximize>::Clear() {
  IViterbiSearch::Clear();
  states_by_time.clear();
  ClearSearch();
}

template <bool Maximize> void NaiveViterbiSearch<Maximize>::ClearSearch() {
  history_.clear();
  winner_by_time.clear();
}

template <bool Maximize> bool NaiveViterbiSearch<Maximize>::AddStateId(const StateId& stateid) {
  if (!IViterbiSearch::AddStateId(stateid)) {
    return false;
  }

  if (states_by_time.size() <= stateid.time()) {
    states_by_time.resize(stateid.time() + 1);
  }

  states_by_time[stateid.time()].push_back(stateid);

  return true;
}

template <bool Maximize> bool NaiveViterbiSearch<Maximize>::RemoveStateId(const StateId& stateid) {
  const bool removed = IViterbiSearch::RemoveStateId(stateid);
  if (!removed) {
    return false;
  }
  // remove it from columns
  auto& column = states_by_time[stateid.time()];
  const auto it = std::find(column.begin(), column.end(), stateid);
  if (it == column.end()) {
    throw std::logic_error("the state must exist in the column");
  }
  column.erase(it);

  return true;
}

template <bool Maximize>
double NaiveViterbiSearch<Maximize>::AccumulatedCost(const StateId& stateid) const {
  return stateid.IsValid() ? GetLabel(stateid).costsofar() : kInvalidCost;
}

template <bool Maximize> StateId NaiveViterbiSearch<Maximize>::SearchWinner(StateId::Time target) {
  if (states_by_time.size() <= target) {
    return {};
  }

  // Use the cache
  if (target < winner_by_time.size()) {
    return winner_by_time[target];
  }

  for (StateId::Time time = winner_by_time.size(); time <= target; ++time) {
    const auto& column = states_by_time[time];
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
      // If it's not reachable by previous column, we find the winner
      // with the best emission cost only
      labels = InitLabels(column, true);
      winner = FindWinner(labels);
    }
    winner_by_time.push_back(winner);
    history_.push_back(move(labels));
  }

  return winner_by_time[target];
}

template <bool Maximize>
StateId NaiveViterbiSearch<Maximize>::Predecessor(const StateId& stateid) const {
  return stateid.IsValid() ? GetLabel(stateid).predecessor() : StateId();
}

template <bool Maximize>
void NaiveViterbiSearch<Maximize>::UpdateLabels(std::vector<StateLabel>& labels,
                                                const std::vector<StateLabel>& prev_labels) const {
  for (const auto& prev_label : prev_labels) {
    const auto& prev_stateid = prev_label.stateid();

    const auto prev_costsofar = prev_label.costsofar();
    if (prev_costsofar == kInvalidCost) {
      continue;
    }

    for (auto& label : labels) {
      const auto stateid = label.stateid();

      const auto emission_cost = EmissionCost(stateid);
      if (emission_cost == kInvalidCost) {
        continue;
      }

      const auto transition_cost = TransitionCost(prev_stateid, stateid);
      if (transition_cost == kInvalidCost) {
        continue;
      }

      const auto costsofar = CostSofar(prev_costsofar, transition_cost, emission_cost);
      if (costsofar == kInvalidCost) {
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
std::vector<StateLabel> NaiveViterbiSearch<Maximize>::InitLabels(const std::vector<StateId>& column,
                                                                 bool use_emission_cost) const {
  std::vector<StateLabel> labels;
  labels.reserve(column.size());
  for (const auto& stateid : column) {
    labels.emplace_back(use_emission_cost ? EmissionCost(stateid) : kInvalidCost, stateid, StateId());
  }
  return labels;
}

template <bool Maximize>
StateId NaiveViterbiSearch<Maximize>::FindWinner(const std::vector<StateLabel>& labels) const {
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
const StateLabel& NaiveViterbiSearch<Maximize>::GetLabel(const StateId& stateid) const {
  const auto& labels = history_[stateid.time()];
  const auto it = std::find_if(labels.cbegin(), labels.cend(), [&stateid](const StateLabel& label) {
    return label.stateid() == stateid;
  });
  if (it == labels.end()) {
    throw std::runtime_error("impossible that label not found; if it happened, check SearchWinner");
  }
  return *it;
}

template class NaiveViterbiSearch<true>;
template class NaiveViterbiSearch<false>;

ViterbiSearch::ViterbiSearch(const IEmissionCostModel& emission_cost_model,
                             const ITransitionCostModel& transition_cost_model)
    : IViterbiSearch(emission_cost_model, transition_cost_model) {
}

ViterbiSearch::ViterbiSearch() : ViterbiSearch(DefaultEmissionCostModel, DefaultTransitionCostModel) {
}

ViterbiSearch::~ViterbiSearch() {
  Clear();
}

bool ViterbiSearch::AddStateId(const StateId& stateid) {
  if (!IViterbiSearch::AddStateId(stateid)) {
    return false;
  }

  if (states_by_time.size() <= stateid.time()) {
    states_by_time.resize(stateid.time() + 1);
  }
  states_by_time[stateid.time()].push_back(stateid);

  if (unreached_states_by_time.size() <= stateid.time()) {
    unreached_states_by_time.resize(stateid.time() + 1);
  }
  unreached_states_by_time[stateid.time()].push_back(stateid);

  return true;
}

bool ViterbiSearch::RemoveStateId(const StateId& stateid) {
  const auto removed = IViterbiSearch::RemoveStateId(stateid);
  if (!removed) {
    return false;
  }
  // remove it from columns
  auto& column = states_by_time[stateid.time()];
  const auto it = std::find(column.begin(), column.end(), stateid);
  if (it == column.end()) {
    throw std::logic_error("the state must exist in the column");
  }
  column.erase(it);
  return true;
}

StateId ViterbiSearch::SearchWinner(StateId::Time time) {
  // Use the cache
  if (time < winner_by_time.size()) {
    return winner_by_time[time];
  }

  if (unreached_states_by_time.empty()) {
    return {};
  }

  const StateId::Time max_allowed_time = unreached_states_by_time.size() - 1;
  const auto target = std::min(time, max_allowed_time);

  // Continue last search if possible
  StateId::Time searched_time = IterativeSearch(target, false);

  while (searched_time < target) {
    // searched_time < target implies that there was a breakage during
    // last search, so we request a new start
    searched_time = IterativeSearch(target, true);
    // Guarantee that that winner_by_time.size() is increasing and searched_time ==
    // winner_by_time.size() - 1
  }

  if (time < winner_by_time.size()) {
    return winner_by_time[time];
  }

  return {};
}

StateId ViterbiSearch::Predecessor(const StateId& stateid) const {
  const auto it = scanned_labels_.find(stateid);
  if (it == scanned_labels_.end()) {
    return {};
  } else {
    return (it->second).predecessor();
  }
}

double ViterbiSearch::AccumulatedCost(const StateId& stateid) const {
  const auto it = scanned_labels_.find(stateid);
  if (it == scanned_labels_.end()) {
    return -1.f;
  } else {
    return it->second.costsofar();
  }
}

void ViterbiSearch::Clear() {
  IViterbiSearch::Clear();
  states_by_time.clear();
  ClearSearch();
}

void ViterbiSearch::ClearSearch() {
  earliest_time_ = 0;
  queue_.clear();
  scanned_labels_.clear();
  winner_by_time.clear();
  unreached_states_by_time = states_by_time;
}

void ViterbiSearch::InitQueue(const std::vector<StateId>& column) {
  queue_.clear();
  for (const auto stateid : column) {
    const auto emission_cost = EmissionCost(stateid);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }
    queue_.push(StateLabel(emission_cost, stateid, {}));
  }
}

void ViterbiSearch::AddSuccessorsToQueue(const StateId& stateid) {
  if (!(stateid.time() + 1 < unreached_states_by_time.size())) {
    throw std::logic_error("the state at time " + std::to_string(stateid.time()) +
                           " is impossible to have successors");
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

  // Optimal states have been removed from unreached_states_by_time so no
  // worry about optimality
  for (const auto& next_stateid : unreached_states_by_time[stateid.time() + 1]) {
    const auto emission_cost = EmissionCost(next_stateid);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }

    const auto transition_cost = TransitionCost(stateid, next_stateid);
    if (IsInvalidCost(transition_cost)) {
      continue;
    }

    const auto next_costsofar = CostSofar(costsofar, transition_cost, emission_cost);
    if (IsInvalidCost(next_costsofar)) {
      continue;
    }

    queue_.push(StateLabel(next_costsofar, next_stateid, stateid));
  }
}

StateId::Time ViterbiSearch::IterativeSearch(StateId::Time target, bool request_new_start) {
  if (unreached_states_by_time.size() <= target) {
    if (unreached_states_by_time.empty()) {
      throw std::runtime_error("empty states: add some states at least before searching");
    } else {
      throw std::runtime_error("the target time is beyond the maximum allowed time " +
                               std::to_string(unreached_states_by_time.size() - 1));
    }
  }

  // Do nothing since the winner at the target time is already known
  if (target < winner_by_time.size()) {
    return target;
  }

  // Clearly here we have precondition: winner_by_time.size() <= target <
  // unreached_states_by_time.size()

  StateId::Time source;
  // Either continue last search, or start a new search
  if (!request_new_start && !winner_by_time.empty() && winner_by_time.back().IsValid()) {
    source = winner_by_time.size() - 1;
    AddSuccessorsToQueue(winner_by_time[source]);
  } else {
    source = winner_by_time.size();
    InitQueue(unreached_states_by_time[source]);
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
      throw std::logic_error("the principle of optimality is violated in the viterbi search,"
                             " probably negative costs occurred");
    }

    // Remove it from its column
    auto& column = unreached_states_by_time[stateid.time()];
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
    if (winner_by_time.size() <= stateid.time()) {
      if (!(stateid.time() == winner_by_time.size())) {
        // Should check if states at unreached_states_by_time[time] are all
        // at the same TIME
        throw std::logic_error("found a state from the future time " +
                               std::to_string(stateid.time()));
      }
      winner_by_time.push_back(stateid);
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
  if (winner_by_time.size() <= searched_time) {
    winner_by_time.resize(searched_time + 1);
  }

  // Postcondition: searched_time == winner_by_time.size() - 1 && search_time <= target
  // If search_time < target it implies that there is a breakage,
  // i.e. unable to find any connection from the column at search_time
  // to the column at search_time + 1

  return searched_time;
}

constexpr bool ViterbiSearch::IsInvalidCost(double cost) {
  return cost < 0.f;
}

} // namespace meili
} // namespace valhalla
