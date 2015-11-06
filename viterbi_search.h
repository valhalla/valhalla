// -*- mode: c++ -*-

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cassert>

#include "queue.h"


using Time = uint32_t;
using CandidateId = uint32_t;
using CandidatePairId = uint64_t;


constexpr Time kInvalidTime = std::numeric_limits<Time>::max();
constexpr CandidateId kInvalidStateId = std::numeric_limits<CandidateId>::max();


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


#define CANDIDATE_TYPE T


template <typename T>
struct LabelTemplate: public LabelInterface<CandidateId>
{
  LabelTemplate(double c,
                const CANDIDATE_TYPE* a,
                const CANDIDATE_TYPE* p)
      : costsofar(c),
        candidate(a),
        predecessor(p) {}

  CandidateId id() const override
  { return candidate->id(); }

  double sortcost() const override
  { return costsofar; }

  // Accumulated cost since time = 0
  double costsofar;

  // Invariant: candidate->time() == (predecessor==null_ptr? 0 : predecessor->time() + 1)
  const CANDIDATE_TYPE* candidate;
  const CANDIDATE_TYPE* predecessor;
};


template <typename T>
class IViterbiSearch;


template <typename state_t>
class ViterbiPathIterator:
    public std::iterator<std::forward_iterator_tag, state_t>
{
 public:
  ViterbiPathIterator(IViterbiSearch<state_t>* vs, CandidateId id)
      : vs_(vs),
        id_(id),
        time_(vs->state(id).time()) {}

  ViterbiPathIterator(IViterbiSearch<state_t>* vs): vs_(vs)
  { set_end(); }

  // Postfix increment
  ViterbiPathIterator operator++(int)
  {
    if (!is_end()) {
      auto copy = *this;
      goback();
      return copy;
    }
    return *this;
  }

  // Prefix increment
  ViterbiPathIterator<state_t> operator++()
  {
    if (!is_end()) {
      goback();
    }
    return *this;
  }

  bool operator==(const ViterbiPathIterator<state_t>& other) const
  { return id_ == other.id_ && time_ == other.time_ && vs_ == other.vs_; }

  bool operator!=(const ViterbiPathIterator<state_t>& other) const
  { return !(*this == other); }

  // Derefrencnce
  const state_t& operator*() const
  { return vs_->state(id_); }

  // Pointer dereference
  const state_t* operator->() const
  { return &(vs_->state(id_)); }

  Time time() const
  { return time_; }

  // Invalid iterator can't be dereferenced
  bool IsValid() const
  { return id_ == kInvalidStateId; }

 private:
  IViterbiSearch<state_t>* vs_;
  CandidateId id_;
  Time time_;

  bool is_end() const
  { return id_ == kInvalidStateId && time_ == kInvalidTime; }

  void set_end()
  {
    id_ = kInvalidStateId;
    time_ = kInvalidTime;
  }

  void goback()
  {
    if (time_ > 0) {
      id_ = vs_->predecessor(id_);
      time_ --;
      if (id_ == kInvalidStateId) {
        id_ = vs_->SearchWinner(time_);
      }
    } else {
      set_end();
    }
  }
};


template <typename T>
class IViterbiSearch
{
  friend class ViterbiPathIterator<T>;

 public:
  using iterator = ViterbiPathIterator<T>;

  IViterbiSearch(): path_end_(iterator(this)) {};

  virtual ~IViterbiSearch() {};

  virtual CandidateId SearchWinner(Time time) = 0;

  virtual CandidateId predecessor(CandidateId id) const = 0;

  // Get the state reference given its ID
  virtual const T& state(CandidateId id) const = 0;

  iterator SearchPath(Time time)
  { return iterator(this, SearchWinner(time)); }

  iterator PathEnd() const
  { return path_end_; }

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

 private:
  const iterator path_end_;
};


template <typename T, bool Maximize>
class NaiveViterbiSearch: public IViterbiSearch<CANDIDATE_TYPE>
{
 public:
  // An invalid costsofar indicates that candiadte is unreachable from
  // previous candidates
  static constexpr double
  kInvalidCost = Maximize? -std::numeric_limits<double>::infinity()
      : std::numeric_limits<double>::infinity();

  ~NaiveViterbiSearch();

  void Clear();

  CandidateId SearchWinner(Time time) override;

  CandidateId predecessor(CandidateId id) const override;

  const T& state(CandidateId id) const override;

  double costsofar(const CANDIDATE_TYPE& candidate) const;

 protected:
  std::vector<std::vector<const CANDIDATE_TYPE*>> states_;
  std::vector<const CANDIDATE_TYPE*> candidates_;
  std::vector<const CANDIDATE_TYPE*> winners_;

  virtual float TransitionCost(const CANDIDATE_TYPE& left, const CANDIDATE_TYPE& right) const override = 0;
  virtual float EmissionCost(const CANDIDATE_TYPE& candidate) const override = 0;
  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  using label_type = LabelTemplate<T>;

  std::vector<std::vector<label_type>> history_;

  void UpdateLabels(std::vector<label_type>& labels,
                    const std::vector<label_type>& prev_labels) const;

  std::vector<label_type> InitLabels(const std::vector<const CANDIDATE_TYPE*>& state,
                                     bool use_emission_cost) const;

  const CANDIDATE_TYPE* FindWinner(const std::vector<label_type>& labels) const;

  // Linear search the label of a candidate
  label_type label(const CANDIDATE_TYPE& candidate) const;

  const std::vector<const CANDIDATE_TYPE*>
  FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const;
};


template <typename T, bool Maximize>
inline NaiveViterbiSearch<T, Maximize>::~NaiveViterbiSearch()
{
  Clear();
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
inline double NaiveViterbiSearch<T, Maximize>::costsofar(const CANDIDATE_TYPE& candidate) const
{
  return label(candidate).costsofar;
}


template <typename T, bool Maximize>
CandidateId NaiveViterbiSearch<T, Maximize>::SearchWinner(Time target)
{
  if (states_.size() <= target) {
    return kInvalidStateId;
  }

  // Use the cache
  if (target < winners_.size()) {
    return winners_[target]? winners_[target]->id() : kInvalidStateId;
  }

  for (Time time = winners_.size(); time <= target; ++time) {
    const auto& state = states_[time];
    std::vector<label_type> labels;

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

  return winners_[target]? winners_[target]->id() : kInvalidStateId;
}


template <typename T, bool Maximize>
inline CandidateId
NaiveViterbiSearch<T, Maximize>::predecessor(CandidateId id) const
{
  if (id != kInvalidStateId) {
    const auto& predecessor = label(state(id)).predecessor;
    return predecessor? predecessor->id() : kInvalidStateId;
  }
  return kInvalidStateId;
}


template <typename T, bool Maximize>
inline const T&
NaiveViterbiSearch<T, Maximize>::state(CandidateId id) const
{
  return *candidates_[id];
}


template <typename T, bool Maximize>
void NaiveViterbiSearch<T, Maximize>::UpdateLabels(
    std::vector<label_type>& labels,
    const std::vector<label_type>& prev_labels) const
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
        label = std::max(label_type(costsofar, candidate_ptr, prev_candidate_ptr), label);
      } else {
        label = std::min(label_type(costsofar, candidate_ptr, prev_candidate_ptr), label);
      }
    }
  }
}


template <typename T, bool Maximize>
std::vector<typename NaiveViterbiSearch<T, Maximize>::label_type>
NaiveViterbiSearch<T, Maximize>::InitLabels(
    const std::vector<const CANDIDATE_TYPE*>& state,
    bool use_emission_cost) const
{
  std::vector<label_type> labels;
  for (const auto& candidate_ptr : state) {
    auto initial_cost = use_emission_cost? EmissionCost(*candidate_ptr) : kInvalidCost;
    labels.emplace_back(initial_cost, candidate_ptr, nullptr);
  }
  return labels;
}


template <typename U, bool Maximize>
U FindExtremeElement(U begin, U end, double invalid_cost)
{
  U extreme_itr = end;

  for (U itr = begin; itr != end; itr++) {
    // Filter out invalid costs
    if (invalid_cost == itr->costsofar) {
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


template <typename T, bool Maximize>
const CANDIDATE_TYPE*
NaiveViterbiSearch<T, Maximize>::FindWinner(const std::vector<label_type>& labels) const
{
  if (labels.empty()) {
    return nullptr;
  }
  auto itr = FindExtremeElement<decltype(labels.cbegin()), Maximize>(labels.cbegin(), labels.cend(), kInvalidCost);
  if (itr == labels.cend()) {
    return nullptr;
  }
  assert(kInvalidCost != itr->costsofar);
  return itr->candidate;
}


// Linear search a candidate's label
template <typename T, bool Maximize>
typename NaiveViterbiSearch<T, Maximize>::label_type
NaiveViterbiSearch<T, Maximize>::label(const CANDIDATE_TYPE& candidate) const
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


template <typename T>
class ViterbiSearch: public IViterbiSearch<CANDIDATE_TYPE>
{
 public:
  ~ViterbiSearch();

  void Clear();

  CandidateId SearchWinner(Time time) override;

  const T& state(CandidateId id) const override;

  CandidateId predecessor(CandidateId id) const override;

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
  // candidate id => candidate
  // Invariant: for each candidate: assert(candidate == candidates_[candidate.id]);
  std::vector<const CANDIDATE_TYPE*> candidates_;

  // time => winner
  std::vector<const CANDIDATE_TYPE*> winners_;

  // time => state
  std::vector<std::vector<const CANDIDATE_TYPE*>> unreached_states_;

  // TODO remove it
  const std::vector<const CANDIDATE_TYPE*>& unreached_states(Time time) const;

  virtual float TransitionCost(const CANDIDATE_TYPE& left, const CANDIDATE_TYPE& right) const override = 0;
  virtual float EmissionCost(const CANDIDATE_TYPE& candidate) const override = 0;
  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
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

  // Initialize labels from state and push them into priority queue
  void InitQueue(const std::vector<const CANDIDATE_TYPE*>& state);

  void AddSuccessorsToQueue(const CANDIDATE_TYPE* candidate_ptr);

  Time IterativeSearch(Time target, bool request_new_start);

  const CANDIDATE_TYPE* FindLastWinner(Time time) const;

  const std::vector<const CANDIDATE_TYPE*>
  FormPathSegment(const CANDIDATE_TYPE* candidate_ptr) const;
};


template <typename T>
inline ViterbiSearch<T>::~ViterbiSearch()
{
  Clear();
}


template <typename T>
CandidateId ViterbiSearch<T>::SearchWinner(Time time)
{
  // Use the cache
  if (time < winners_.size()) {
    return winners_[time]? winners_[time]->id() : kInvalidStateId;
  }

  if (unreached_states_.empty()) {
    return kInvalidStateId;
  }

  Time target = std::min(time, static_cast<Time>(unreached_states_.size()) - 1);
  Time searched_time = IterativeSearch(target, false);
  while (searched_time < target) {
    searched_time = IterativeSearch(target, true);
  }

  if (time < winners_.size() && winners_[time]) {
    return winners_[time]->id();
  }
  return kInvalidStateId;
}


template <typename T>
inline CandidateId
ViterbiSearch<T>::predecessor(CandidateId id) const
{
  auto it = scanned_labels_.find(id);
  if (it != scanned_labels_.end()) {
    return (it->second).predecessor->id();
  } else {
    return kInvalidStateId;
  }
}


template <typename T>
inline const T& ViterbiSearch<T>::state(CandidateId id) const
{
  return *candidates_[id];
}


template <typename T>
void ViterbiSearch<T>::Clear()
{
  queue_.clear();
  scanned_labels_.clear();
  unreached_states_.clear();
  winners_.clear();
  for (auto candidate_ptr : candidates_) {
    delete candidate_ptr;
  }
  candidates_.clear();
}


template <typename T>
void ViterbiSearch<T>::InitQueue(const std::vector<const CANDIDATE_TYPE*>& state)
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
const std::vector<const CANDIDATE_TYPE*>&
ViterbiSearch<T>::unreached_states(Time time) const {
  return unreached_states_[time];
}


template <typename T>
void ViterbiSearch<T>::AddSuccessorsToQueue(const CANDIDATE_TYPE* candidate_ptr)
{
  assert(candidate_ptr->time() + 1 < unreached_states_.size());
  if (unreached_states_.size() <= candidate_ptr->time() + 1) {
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

  auto next_state = unreached_states_[candidate_ptr->time() + 1];
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
Time ViterbiSearch<T>::IterativeSearch(Time target, bool request_new_start)
{
  assert(!unreached_states_.empty() && target < unreached_states_.size());
  if (unreached_states_.empty()) {
    throw std::runtime_error("empty states");
  }

  if (target < winners_.size()) {
    return target;
  }

  // So here we have: assert(winners_.size() <= target && target < unreached_states_.size());

  Time source;

  // Initialize queue
  if (!request_new_start && !winners_.empty() && winners_.back()) {
    source = winners_.size() - 1;
    AddSuccessorsToQueue(winners_[source]);
  } else {
    source = winners_.size();
    InitQueue(unreached_states_[source]);
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
    auto& state = unreached_states_[time];
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
