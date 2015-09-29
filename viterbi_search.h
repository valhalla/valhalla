// -*- mode: c++ -*-

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cassert>

#include "queue.h"


using Time = uint32_t;
using CandidateId = uint32_t;
using CandidatePairId = uint64_t;


inline CandidatePairId candidateid_make_pair(CandidateId left, CandidateId right)
{
  return (static_cast<CandidatePairId>(left) << 32) + right;
}


inline CandidateId candidateid_left(CandidatePairId pair)
{
  return static_cast<CandidateId>(pair >> 32);
}


inline CandidateId candidateid_right(CandidatePairId pair)
{
  return static_cast<CandidateId>((pair << 32) >> 32);
}


template <typename T>
class CandidateWrapper final
{
 public:
  CandidateWrapper(const CandidateId id, const Time time, const T& candidate)
      : id_(id), time_(time), candidate_(candidate) {}
  inline const Time time() const {return time_;}
  inline const CandidateId id() const {return id_;}
  inline const T& candidate() const {return candidate_;}

 private:
  Time time_;
  CandidateId id_;
  T candidate_;
};


#define CANDIDATE_TYPE CandidateWrapper<T>


template <typename T>
struct LabelTemplate: public LabelInterface<CandidateId>
{
  LabelTemplate(double c,
                const CANDIDATE_TYPE* a,
                const CANDIDATE_TYPE* p)
      : costsofar(c),
        candidate(a),
        predecessor(p) {
  }

  inline CandidateId id() const override {
    return candidate->id();
  }

  inline double sortcost() const override {
    return costsofar;
  }

  // Accumulated cost since time = 0
  double costsofar;

  // Invariant: candidate->time() == (predecessor==null_ptr? 0 : predecessor->time() + 1)
  const CANDIDATE_TYPE* candidate;
  const CANDIDATE_TYPE* predecessor;
};


// TODO interfaces to confidence of a path and costsofar of a candidate??
template <typename T>
class ViterbiSearchInterface
{
 public:
  // Search the winner at a specific time
  virtual const T* SearchWinner(Time target) = 0;

  // Search the optimal path from the initial time to the target time
  virtual std::vector<const T*> SearchPath(Time target) = 0;

  virtual ~ViterbiSearchInterface() {
  }
 protected:
  // Calculate transition cost from left candidate to right candidate
  virtual float TransitionCost(const T& left, const T& right) const = 0;

  // Calculate emission cost of a candidate
  virtual float EmissionCost(const T& candidate) const = 0;

  // Calculate the a candidate's costsofar based on its predecessor's
  // costsofar, transition cost from predecessor to this candidate,
  // and emission cost of this candidate
  virtual double CostSofar(double prev_costsofar,
                           float transition_cost,
                           float emission_cost) const = 0;
};


template <typename T, bool Maximize>
class NaiveViterbiSearch: public ViterbiSearchInterface<CANDIDATE_TYPE>
{
 public:
  ~NaiveViterbiSearch();
  Time AppendState(typename std::vector<T>::const_iterator begin,
                   typename std::vector<T>::const_iterator end);
  const CANDIDATE_TYPE* SearchWinner(Time target) override;
  std::vector<const CANDIDATE_TYPE*> SearchPath(Time target) override;

  // TODO in-class initilization well supported?
  // An invalid costsofar indicates that candiadte is unreachable from
  // previous candidates
  const double kInvalidCost = Maximize? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();

  // Clear everything
  void Clear();

  inline double costsofar(const CANDIDATE_TYPE& candidate) const
  {
    return label(candidate).costsofar;
  }

  inline const CANDIDATE_TYPE* predecessor(const CANDIDATE_TYPE& candidate) const
  {
    return label(candidate).predecessor;
  }

 protected:
  virtual float TransitionCost(const CANDIDATE_TYPE& left, const CANDIDATE_TYPE& right) const override = 0;
  virtual float EmissionCost(const CANDIDATE_TYPE& candidate) const override = 0;
  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  using Label = LabelTemplate<T>;
  std::vector<std::vector<Label>> history_;

  using State = std::vector<const CANDIDATE_TYPE*>;
  std::vector<State> states_;

  std::vector<const CANDIDATE_TYPE*> candidates_;
  std::vector<const CANDIDATE_TYPE*> winners_;

  // TODO move it outside?
  void UpdateLabels(std::vector<Label>& labels,
                    const std::vector<Label>& prev_labels) const
  {
    for (const auto& prev_label : prev_labels) {
      auto prev_candidate_ptr = prev_label.candidate;

      auto prev_costsofar = prev_label.costsofar;
      if (kInvalidCost == prev_costsofar) {
        continue;
      }

      for (auto& label : labels) {
        auto candidate_ptr = label.candidate;

        auto emission_cost = EmissionCost(*candidate_ptr);
        if (kInvalidCost == emission_cost) {
          continue;
        };

        auto transition_cost = TransitionCost(*prev_candidate_ptr, *candidate_ptr);
        if (kInvalidCost == transition_cost) {
          continue;
        }

        auto costsofar = CostSofar(prev_costsofar, transition_cost, emission_cost);
        if (kInvalidCost == costsofar) {
          continue;
        }

        if (Maximize) {
          label = std::max(Label(costsofar, candidate_ptr, prev_candidate_ptr), label);
        } else {
          label = std::min(Label(costsofar, candidate_ptr, prev_candidate_ptr), label);
        }
      }
    }
  }

  // TODO move it outside?
  std::vector<Label> InitLabels(const State& state,
                                bool use_emission_cost) const
  {
    std::vector<Label> labels;
    for (const auto& candidate_ptr : state) {
      auto initial_cost = use_emission_cost? EmissionCost(*candidate_ptr) : kInvalidCost;
      labels.emplace_back(initial_cost, candidate_ptr, nullptr);
    }
    return labels;
  }

  template <typename U>
  U FindExtremeElement(U begin, U end) const
  {
    U extreme_itr = end;

    for (U itr = begin; itr != end; itr++) {
      // Filter out invalid costs
      if (kInvalidCost == itr->costsofar) {
        continue;
      }

      // Find the first extreme element
      if (Maximize) {
        if (extreme_itr == end || extreme_itr->costsofar < itr->costsofar) {
          extreme_itr = itr;
        }
      } else {
        if (extreme_itr == end || itr->costsofar < extreme_itr->costsofar) {
          extreme_itr = itr;
        }
      }
    }

    return extreme_itr;
  }

  const CANDIDATE_TYPE* FindWinner(const std::vector<Label>& labels) const
  {
    if (labels.empty()) {
      return nullptr;
    }
    auto itr = FindExtremeElement(labels.cbegin(), labels.cend());
    if (itr == labels.cend()) {
      return nullptr;
    }
    assert(kInvalidCost != itr->costsofar);
    return itr->candidate;
  }

  // Linear search the label of a candidate
  Label label(const CANDIDATE_TYPE& candidate) const
  {
    auto time = candidate.time();

    for (const auto& label : history_[time]) {
      if (label.candidate->id() == candidate.id()) {
        return label;
      }
    }

    assert(false);
    throw std::runtime_error("impossible that label not found; if it happened, check SearchWinner");
  }

  const std::vector<const CANDIDATE_TYPE*>
  FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const;
};


template <typename T, bool Maximize>
NaiveViterbiSearch<T, Maximize>::~NaiveViterbiSearch()
{
  Clear();
}


template <typename T, bool Maximize>
Time NaiveViterbiSearch<T, Maximize>::AppendState(typename std::vector<T>::const_iterator begin,
                                                  typename std::vector<T>::const_iterator end)
{
  State state;
  Time time = states_.size();
  for (auto candidate = begin; candidate != end; candidate++) {
    auto candidate_id = candidates_.size();
    candidates_.push_back(new CANDIDATE_TYPE(candidate_id, time, *candidate));
    state.push_back(candidates_.back());
  }
  states_.push_back(state);
  return time;
}


template <typename T, bool Maximize>
void NaiveViterbiSearch<T, Maximize>::Clear()
{
  history_.clear();
  states_.clear();
  winners_.clear();
  for (auto candidate_ptr : candidates_) {
    delete candidate_ptr;
  }
  candidates_.clear();
}


template <typename T, bool Maximize>
const CANDIDATE_TYPE*
NaiveViterbiSearch<T, Maximize>::SearchWinner(Time target)
{
  if (states_.size() <= target) {
    return nullptr;
  }

  // Use the cache
  if (target < winners_.size()) {
    return winners_[target];
  }

  for (Time time = winners_.size(); time <= target; ++time) {
    const auto& state = states_[time];
    std::vector<Label> labels;

    // Update labels
    if (time == 0) {
      labels = InitLabels(state, true);
    } else {
      labels = InitLabels(state, false);
      UpdateLabels(labels, history_.back());
    }
    assert(labels.size() == state.size());

    auto winner = FindWinner(labels);
    if (!winner && time > 0) {
      // If it's not reachable by prevous state, we find the winner
      // with the best emission cost only
      labels = InitLabels(state, true);
      winner = FindWinner(labels);
    }
    winners_.push_back(winner);
    history_.push_back(labels);
  }

  return winners_[target];
}


template <typename T, bool Maximize>
std::vector<const CANDIDATE_TYPE*>
NaiveViterbiSearch<T, Maximize>::SearchPath(Time target)
{
  std::vector<const CANDIDATE_TYPE*> path;
  // Need add this number of candidates to the path
  auto count = target + 1;

  while (path.size() < count) {
    Time time = target - path.size();
    if (states_.size() <= time) {
      path.push_back(nullptr);
      continue;
    }

    auto winner_ptr = time < winners_.size()? winners_[time] : SearchWinner(time);
    if (!winner_ptr) {
      path.push_back(nullptr);
      continue;
    }

    auto segment = FormPathSegment(winner_ptr);
    assert(segment.front() == winner_ptr);
    for (const auto& candidate_ptr : segment) {
      path.push_back(candidate_ptr);
    }
  }

  assert(path.size() == count);

  return path;
}


template <typename T, bool Maximize>
const std::vector<const CANDIDATE_TYPE*>
NaiveViterbiSearch<T, Maximize>::FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const
{
  Time target = candidate_ptr->time();
  const auto count = target + 1;

  std::vector<const CANDIDATE_TYPE*> path;
  while (candidate_ptr && path.size() < count) {
    path.push_back(candidate_ptr);
    candidate_ptr = predecessor(*candidate_ptr);
  }

  return path;
}


template <typename T>
class ViterbiSearch: public ViterbiSearchInterface<CANDIDATE_TYPE>
{
 public:
  ~ViterbiSearch();

  virtual Time AppendState(typename std::vector<T>::const_iterator begin,
                           typename std::vector<T>::const_iterator end);
  const CANDIDATE_TYPE* SearchWinner(Time time) override;
  const CANDIDATE_TYPE* SearchWinner();
  const CANDIDATE_TYPE* SearchLastWinner(Time time);
  const CANDIDATE_TYPE* SearchLastWinner();

  std::vector<const CANDIDATE_TYPE*> SearchPath(Time target) override;

  const CANDIDATE_TYPE* previous(const CANDIDATE_TYPE& candidate) const;

  const CANDIDATE_TYPE* predecessor(const CANDIDATE_TYPE& candidate) const;

  void Clear();

  virtual bool IsInvalidCost(double cost) {
    return cost < 0.f;
  }

  double costsofar(const CANDIDATE_TYPE& candidate)
  {
    auto itr = scanned_labels_.find(candidate.id());
    if (itr==scanned_labels_.end()) {
      return -1.f;
    } else {
      return itr->second.costsofar;
    }
  }

 protected:
  virtual float TransitionCost(const CANDIDATE_TYPE& left, const CANDIDATE_TYPE& right) const override = 0;
  virtual float EmissionCost(const CANDIDATE_TYPE& candidate) const override = 0;
  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  using State = std::vector<const CANDIDATE_TYPE*>;

  struct Label: public LabelTemplate<T> {
    Label()
        : LabelTemplate<T>(-1.f, nullptr, nullptr) {
    }
    Label(double c, const CANDIDATE_TYPE* a, const CANDIDATE_TYPE* p)
        : LabelTemplate<T>(c, a, p) {
    }
  };

  SPQueue<Label> queue_;

  // candidate id => label
  std::unordered_map<CandidateId, Label> scanned_labels_;

  // candidate id => candidate
  // Invariant: for each candidate: assert(candidate == candidates_[candidate.id]);
  std::vector<const CANDIDATE_TYPE*> candidates_;

  // time => state
  std::vector<State> states_;

  // time => winner
  // Invariant: for each winner: assert(winner in states_[winner->time])
  std::vector<const CANDIDATE_TYPE*> winners_;

  // Initialize labels from state and push them into priority queue
  void InitQueue(const State& state);

  void AddSuccessorsToQueue(const CANDIDATE_TYPE* candidate_ptr);

  Time IterativeSearch(Time target, bool new_start);

  const CANDIDATE_TYPE* FindLastWinner(Time time) const;

  const std::vector<const CANDIDATE_TYPE*>
  FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const;
};


template <typename T>
ViterbiSearch<T>::~ViterbiSearch()
{
  Clear();
}


template <typename T>
Time ViterbiSearch<T>::AppendState(typename std::vector<T>::const_iterator begin,
                                   typename std::vector<T>::const_iterator end)
{
  State state;
  Time time = states_.size();
  for (auto candidate = begin; candidate != end; candidate++) {
    auto candidate_id = candidates_.size();
    candidates_.push_back(new CANDIDATE_TYPE(candidate_id, time, *candidate));
    state.push_back(candidates_.back());
  }
  states_.push_back(state);
  return time;
}


template <typename T>
const CANDIDATE_TYPE* ViterbiSearch<T>::SearchWinner(Time time)
{
  // Use the cache
  if (time < winners_.size()) {
    return winners_[time];
  }

  if (states_.empty()) {
    return nullptr;
  }

  Time target = std::min(time, static_cast<Time>(states_.size()) - 1);
  Time searched_time = IterativeSearch(target, false);
  while (searched_time < target) {
    searched_time = IterativeSearch(target, true);
  }

  return time < winners_.size()? winners_[time] : nullptr;
}


// Search the winner at the latest time
template <typename T>
inline const CANDIDATE_TYPE*
ViterbiSearch<T>::SearchWinner()
{
  if (states_.empty()) {
    return nullptr;
  }
  return SearchWinner(states_.size() - 1);
}


// Find the last winner during [0, time)
template <typename T>
const CANDIDATE_TYPE*
ViterbiSearch<T>::FindLastWinner(Time time) const
{
  assert(0 <= time && time < winners_.size());
  if (time == 0) {
    return nullptr;
  }
  time -= 1;
  while (!winners_[time] && time > 0) {
    time -= 1;
  }
  return winners_[time]? winners_[time] : nullptr;
}


template <typename T>
inline const CANDIDATE_TYPE*
ViterbiSearch<T>::SearchLastWinner(Time time)
{
  if (states_.empty()) {
    return nullptr;
  }
  time = std::min(time, states_.size() - 1);
  auto candidate = SearchWinner(time);
  return candidate? candidate : FindLastWinner(time);
}


template <typename T>
inline const CANDIDATE_TYPE*
ViterbiSearch<T>::SearchLastWinner()
{
  if (states_.empty()) {
    return nullptr;
  }
  return SearchLastWinner(states_.size() - 1);
}


template <typename T>
const CANDIDATE_TYPE*
ViterbiSearch<T>::previous(const CANDIDATE_TYPE& candidate) const
{
  auto time = candidate.time();
  auto predecessor_ptr = predecessor(candidate);
  if (predecessor_ptr) {
    assert(predecessor_ptr->time() + 1 == time);
    return predecessor_ptr;
  }
  return FindLastWinner(time);
}


template <typename T>
inline const CANDIDATE_TYPE*
ViterbiSearch<T>::predecessor(const CANDIDATE_TYPE& candidate) const
{
  auto itr = scanned_labels_.find(candidate.id());
  if (itr == scanned_labels_.end()) {
    return nullptr;
  } else {
    return itr->second.predecessor;
  }
}


template <typename T>
void ViterbiSearch<T>::Clear()
{
  queue_.clear();
  scanned_labels_.clear();
  states_.clear();
  winners_.clear();
  for (auto candidate_ptr : candidates_) {
    delete candidate_ptr;
  }
  candidates_.clear();
}


template <typename T>
void ViterbiSearch<T>::InitQueue(const ViterbiSearch<T>::State& state)
{
  queue_.clear();
  for (const auto& candidate_ptr : state) {
    auto emission_cost = EmissionCost(*candidate_ptr);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }
    queue_.push(Label(emission_cost, candidate_ptr, nullptr));
  }
}


template <typename T>
void ViterbiSearch<T>::AddSuccessorsToQueue(const CANDIDATE_TYPE* candidate_ptr)
{
  assert(candidate_ptr->time() + 1 < states_.size());
  if (states_.size() <= candidate_ptr->time() + 1) {
    return;
  }

  auto label_itr = scanned_labels_.find(candidate_ptr->id());
  assert(label_itr != scanned_labels_.end());
  if (label_itr == scanned_labels_.end()) {
    return;
  }
  assert(label_itr->second.candidate == candidate_ptr);  // TODO remove
  auto costsofar = label_itr->second.costsofar;
  assert(!IsInvalidCost(costsofar));

  auto next_state = states_[candidate_ptr->time() + 1];
  for (const auto& next_candidate_ptr : next_state) {
    auto emission_cost = EmissionCost(*next_candidate_ptr);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }

    auto transition_cost = TransitionCost(*candidate_ptr, *next_candidate_ptr);
    if (IsInvalidCost(transition_cost)) {
      continue;
    }

    auto next_costsofar = CostSofar(costsofar, transition_cost,  emission_cost);
    if (IsInvalidCost(next_costsofar)) {
      continue;
    }

    queue_.push(Label(next_costsofar, next_candidate_ptr, candidate_ptr));
  }
}


template<typename T, typename U>
bool state_remove(T* state, const U& candidate)
{
  CandidateId id = candidate.id();
  auto itr = std::find_if(state->begin(), state->end(),
                          [id] (const U* candidate) {
                            return id == candidate->id();
                          });
  if (itr == state->end()) {
    return false;
  }
  state->erase(itr);
  return true;
}


template <typename T>
Time ViterbiSearch<T>::IterativeSearch(Time target, bool new_start)
{
  assert(!states_.empty() && target < states_.size());
  if (states_.empty()) {
    throw std::runtime_error("empty states");
  }

  if (target < winners_.size()) {
    return target;
  }

  // So here we have: assert(winners_.size() <= target && target < states_.size());

  Time source;

  // Initialize queue
  if (!new_start && !winners_.empty() && winners_.back()) {
    source = winners_.size() - 1;
    AddSuccessorsToQueue(winners_[source]);
  } else {
    source = winners_.size();
    InitQueue(states_[source]);
  }

  // Start with the source time, which will be searched anyhow
  auto searched_time = source;

  while (!queue_.empty()) {
    // Candidate with the lowest cost
    auto label = queue_.top();
    queue_.pop();
    auto candidate_ptr = label.candidate;
    auto time = candidate_ptr->time();

    // Mark it as scanned and remember its cost and predecessor
    assert(scanned_labels_.find(candidate_ptr->id())==scanned_labels_.end());
    scanned_labels_[candidate_ptr->id()] = label;

    // Remove it from its state
    auto& state = states_[time];
    bool removed = state_remove(&state, *candidate_ptr);
    assert(removed);

    // If it's the first candidate that arrives at this state, mark it
    // as the winner of this state
    if (winners_.size() <= time) {
      assert(time == winners_.size());
      winners_.push_back(candidate_ptr);
    }

    searched_time = std::max(time, searched_time);

    if (target <= time) {
      break;
    }

    AddSuccessorsToQueue(candidate_ptr);
  }

  // Guarantee that either winner (if found) or nullptr is saved at
  // searched time
  while (winners_.size() <= searched_time) {
    winners_.push_back(nullptr);
  }

  assert(searched_time < winners_.size());

  return searched_time;
}


// Copy from NaiveViterbiSearch
template <typename T>
std::vector<const CANDIDATE_TYPE*>
ViterbiSearch<T>::SearchPath(Time target)
{
  std::vector<const CANDIDATE_TYPE*> path;

  // We need add this number of candidates to the path
  auto count = target + 1;

  while (path.size() < count) {
    Time time = target - path.size();
    if (states_.size() <= time) {
      path.push_back(nullptr);
      continue;
    }

    auto winner_ptr = time < winners_.size()? winners_[time] : SearchWinner(time);
    if (!winner_ptr) {
      path.push_back(nullptr);
      continue;
    }

    auto segment = FormPathSegment(winner_ptr);
    assert(segment.front() == winner_ptr);
    for (const auto& candidate_ptr : segment) {
      path.push_back(candidate_ptr);
    }
  }

  assert(path.size() == count);

  return path;
}


template <typename T>
const std::vector<const CANDIDATE_TYPE*>
ViterbiSearch<T>::FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const
{
  Time target = candidate_ptr->time();
  const auto count = target + 1;

  std::vector<const CANDIDATE_TYPE*> path;
  while (candidate_ptr && path.size() < count) {
    path.push_back(candidate_ptr);
    candidate_ptr = predecessor(*candidate_ptr);
  }

  return path;
}
