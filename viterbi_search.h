// -*- mode: c++ -*-

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cassert>

#include "queue.h"


using Time = uint32_t;
using StateId = uint32_t;
using StatePairId = uint64_t;


constexpr Time kInvalidTime = std::numeric_limits<Time>::max();
constexpr StateId kInvalidStateId = std::numeric_limits<StateId>::max();


inline StatePairId stateid_make_pair(StateId left, StateId right)
{ return (static_cast<StatePairId>(left) << 32) + right; }


inline StateId stateid_left(StatePairId pair)
{ return static_cast<StateId>(pair >> 32); }


inline StateId stateid_right(StatePairId pair)
{ return static_cast<StateId>((pair << 32) >> 32); }


template <typename T>
struct LabelTemplate: public LabelInterface<StateId>
{
  LabelTemplate(double c, const T* a, const T* p)
      : costsofar(c), state(a), predecessor(p) {}

  StateId id() const override
  { return state->id(); }

  double sortcost() const override
  { return costsofar; }

  // Accumulated cost since time = 0
  double costsofar;

  const T* state;

  const T* predecessor;
};


template <typename T>
class IViterbiSearch;


template <typename T>
class ViterbiPathIterator:
    public std::iterator<std::forward_iterator_tag, T>
{
 public:
  ViterbiPathIterator(IViterbiSearch<T>* vs, StateId id)
      : vs_(vs),
        id_(id),
        time_(vs->state(id).time()) {}

  ViterbiPathIterator(IViterbiSearch<T>* vs): vs_(vs)
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
  ViterbiPathIterator<T> operator++()
  {
    if (!is_end()) {
      goback();
    }
    return *this;
  }

  bool operator==(const ViterbiPathIterator<T>& other) const
  { return id_ == other.id_ && time_ == other.time_ && vs_ == other.vs_; }

  bool operator!=(const ViterbiPathIterator<T>& other) const
  { return !(*this == other); }

  // Derefrencnce
  const T& operator*() const
  { return vs_->state(id_); }

  // Pointer dereference
  const T* operator->() const
  { return &(vs_->state(id_)); }

  Time time() const
  { return time_; }

  // Invalid iterator can't be dereferenced
  bool IsValid() const
  { return id_ == kInvalidStateId; }

 private:
  IViterbiSearch<T>* vs_;

  StateId id_;

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

  virtual StateId SearchWinner(Time time) = 0;

  virtual StateId predecessor(StateId id) const = 0;

  // Get the state reference given its ID
  virtual const T& state(StateId id) const = 0;

  iterator SearchPath(Time time)
  { return iterator(this, SearchWinner(time)); }

  iterator PathEnd() const
  { return path_end_; }

 protected:
  // Calculate transition cost from left state to right state
  virtual float TransitionCost(const T& left, const T& right) const = 0;

  // Calculate emission cost of a state
  virtual float EmissionCost(const T& state) const = 0;

  // Calculate the a state's costsofar based on its predecessor's
  // costsofar, transition cost from predecessor to this state,
  // and emission cost of this state
  virtual double CostSofar(double prev_costsofar,
                           float transition_cost,
                           float emission_cost) const = 0;

 private:
  const iterator path_end_;
};


template <typename T, bool Maximize>
class NaiveViterbiSearch: public IViterbiSearch<T>
{
 public:
  // An invalid costsofar indicates that a state is unreachable
  static constexpr double
  kInvalidCost = Maximize? -std::numeric_limits<double>::infinity()
      : std::numeric_limits<double>::infinity();

  ~NaiveViterbiSearch();

  void Clear();

  StateId SearchWinner(Time time) override;

  StateId predecessor(StateId id) const override;

  const T& state(StateId id) const override;

  double costsofar(const T& state) const;

 protected:
  std::vector<std::vector<const T*>> states_;

  std::vector<const T*> state_;

  std::vector<const T*> winner_;

  virtual float TransitionCost(const T& left, const T& right) const override = 0;

  virtual float EmissionCost(const T& state) const override = 0;

  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  using label_type = LabelTemplate<T>;

  std::vector<std::vector<label_type>> history_;

  void UpdateLabels(std::vector<label_type>& labels,
                    const std::vector<label_type>& prev_labels) const;

  std::vector<label_type> InitLabels(const std::vector<const T*>& column,
                                     bool use_emission_cost) const;

  const T* FindWinner(const std::vector<label_type>& labels) const;

  const label_type& label(const T& state) const;
};


template <typename T, bool Maximize>
inline NaiveViterbiSearch<T, Maximize>::~NaiveViterbiSearch()
{ Clear(); }


template <typename T, bool Maximize>
void NaiveViterbiSearch<T, Maximize>::Clear()
{
  history_.clear();
  states_.clear();
  winner_.clear();
  for (auto state : state_) {
    delete state;
  }
  state_.clear();
}


template <typename T, bool Maximize>
inline double
NaiveViterbiSearch<T, Maximize>::costsofar(const T& state) const
{ return label(state).costsofar; }


template <typename T, bool Maximize>
StateId NaiveViterbiSearch<T, Maximize>::SearchWinner(Time target)
{
  if (states_.size() <= target) {
    return kInvalidStateId;
  }

  // Use the cache
  if (target < winner_.size()) {
    return winner_[target]? winner_[target]->id() : kInvalidStateId;
  }

  for (Time time = winner_.size(); time <= target; ++time) {
    const auto& column = states_[time];
    std::vector<label_type> labels;

    // Update labels
    if (time == 0) {
      labels = InitLabels(column, true);
    } else {
      labels = InitLabels(column, false);
      UpdateLabels(labels, history_.back());
    }
    assert(labels.size() == column.size());

    auto winner = FindWinner(labels);
    if (!winner && time > 0) {
      // If it's not reachable by prevous column, we find the winner
      // with the best emission cost only
      labels = InitLabels(column, true);
      winner = FindWinner(labels);
    }
    winner_.push_back(winner);
    history_.push_back(labels);
  }

  return winner_[target]? winner_[target]->id() : kInvalidStateId;
}


template <typename T, bool Maximize>
inline StateId
NaiveViterbiSearch<T, Maximize>::predecessor(StateId id) const
{
  if (id != kInvalidStateId) {
    const auto predecessor = label(state(id)).predecessor;
    return predecessor? predecessor->id() : kInvalidStateId;
  }
  return kInvalidStateId;
}


template <typename T, bool Maximize>
inline const T&
NaiveViterbiSearch<T, Maximize>::state(StateId id) const
{ return *state_[id]; }


template <typename T, bool Maximize>
void NaiveViterbiSearch<T, Maximize>::UpdateLabels(
    std::vector<label_type>& labels,
    const std::vector<label_type>& prev_labels) const
{
  for (const auto& prev_label : prev_labels) {
    auto prev_state = prev_label.state;

    auto prev_costsofar = prev_label.costsofar;
    if (kInvalidCost == prev_costsofar) {
      continue;
    }

    for (auto& label : labels) {
      auto state = label.state;

      auto emission_cost = EmissionCost(*state);
      if (kInvalidCost == emission_cost) {
        continue;
      };

      auto transition_cost = TransitionCost(*prev_state, *state);
      if (kInvalidCost == transition_cost) {
        continue;
      }

      auto costsofar = CostSofar(prev_costsofar, transition_cost, emission_cost);
      if (kInvalidCost == costsofar) {
        continue;
      }

      if (Maximize) {
        label = std::max(label_type(costsofar, state, prev_state), label);
      } else {
        label = std::min(label_type(costsofar, state, prev_state), label);
      }
    }
  }
}


template <typename T, bool Maximize>
std::vector<typename NaiveViterbiSearch<T, Maximize>::label_type>
NaiveViterbiSearch<T, Maximize>::InitLabels(
    const std::vector<const T*>& column,
    bool use_emission_cost) const
{
  std::vector<label_type> labels;
  for (const auto state : column) {
    auto initial_cost = use_emission_cost? EmissionCost(*state) : kInvalidCost;
    labels.emplace_back(initial_cost, state, nullptr);
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
const T*
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
  return itr->state;
}


// Linear search a state's label
template <typename T, bool Maximize>
const typename NaiveViterbiSearch<T, Maximize>::label_type&
NaiveViterbiSearch<T, Maximize>::label(const T& state) const
{
  auto time = state.time();

  for (auto& label : history_[time]) {
    if (label.state->id() == state.id()) {
      return label;
    }
  }

  assert(false);
  throw std::runtime_error("impossible that label not found; if it happened, check SearchWinner");
}


template <typename T>
class ViterbiSearch: public IViterbiSearch<T>
{
 public:
  ~ViterbiSearch();

  void Clear();

  StateId SearchWinner(Time time) override;

  const T& state(StateId id) const override;

  StateId predecessor(StateId id) const override;

  virtual bool IsInvalidCost(double cost) const;

  double costsofar(const T& state) const;

 protected:
  std::vector<const T*> state_;

  std::vector<const T*> winner_;

  std::vector<std::vector<const T*>> unreached_states_;

  virtual float TransitionCost(const T& left, const T& right) const override = 0;

  virtual float EmissionCost(const T& state) const override = 0;

  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  struct Label: public LabelTemplate<T> {
    Label()
        : LabelTemplate<T>(-1.f, nullptr, nullptr) {
    }
    Label(double c, const T* a, const T* p)
        : LabelTemplate<T>(c, a, p) {
    }
  };

  SPQueue<Label> queue_;

  std::unordered_map<StateId, Label> scanned_labels_;

  // Initialize labels from a column and push them into priority queue
  void InitQueue(const std::vector<const T*>& column);

  void AddSuccessorsToQueue(const T* state);

  Time IterativeSearch(Time target, bool request_new_start);
};


template <typename T>
inline ViterbiSearch<T>::~ViterbiSearch()
{ Clear(); }


template <typename T>
StateId ViterbiSearch<T>::SearchWinner(Time time)
{
  // Use the cache
  if (time < winner_.size()) {
    return winner_[time]? winner_[time]->id() : kInvalidStateId;
  }

  if (unreached_states_.empty()) {
    return kInvalidStateId;
  }

  Time target = std::min(time, static_cast<Time>(unreached_states_.size()) - 1);
  Time searched_time = IterativeSearch(target, false);
  while (searched_time < target) {
    searched_time = IterativeSearch(target, true);
  }

  if (time < winner_.size() && winner_[time]) {
    return winner_[time]->id();
  }
  return kInvalidStateId;
}


template <typename T>
inline StateId
ViterbiSearch<T>::predecessor(StateId id) const
{
  auto it = scanned_labels_.find(id);
  if (it != scanned_labels_.end()) {
    return (it->second).predecessor->id();
  } else {
    return kInvalidStateId;
  }
}


template <typename T>
inline bool
ViterbiSearch<T>::IsInvalidCost(double cost) const
{ return cost < 0.f; }


template <typename T>
double ViterbiSearch<T>::costsofar(const T& state) const
{
  auto itr = scanned_labels_.find(state.id());
  if (itr==scanned_labels_.end()) {
    return -1.f;
  } else {
    return itr->second.costsofar;
  }
}


template <typename T>
inline const T& ViterbiSearch<T>::state(StateId id) const
{ return *state_[id]; }


template <typename T>
void ViterbiSearch<T>::Clear()
{
  queue_.clear();
  scanned_labels_.clear();
  unreached_states_.clear();
  winner_.clear();
  for (auto state : state_) {
    delete state;
  }
  state_.clear();
}


template <typename T>
void ViterbiSearch<T>::InitQueue(const std::vector<const T*>& column)
{
  queue_.clear();
  for (const auto state : column) {
    auto emission_cost = EmissionCost(*state);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }
    queue_.push(Label(emission_cost, state, nullptr));
  }
}


template <typename T>
void ViterbiSearch<T>::AddSuccessorsToQueue(const T* state)
{
  assert(state->time() + 1 < unreached_states_.size());
  if (unreached_states_.size() <= state->time() + 1) {
    return;
  }

  auto label_itr = scanned_labels_.find(state->id());
  assert(label_itr != scanned_labels_.end());
  if (label_itr == scanned_labels_.end()) {
    return;
  }
  assert(label_itr->second.state == state);  // TODO remove
  auto costsofar = label_itr->second.costsofar;
  assert(!IsInvalidCost(costsofar));

  auto next_column = unreached_states_[state->time() + 1];
  for (const auto& next_state : next_column) {
    auto emission_cost = EmissionCost(*next_state);
    if (IsInvalidCost(emission_cost)) {
      continue;
    }

    auto transition_cost = TransitionCost(*state, *next_state);
    if (IsInvalidCost(transition_cost)) {
      continue;
    }

    auto next_costsofar = CostSofar(costsofar, transition_cost,  emission_cost);
    if (IsInvalidCost(next_costsofar)) {
      continue;
    }

    queue_.push(Label(next_costsofar, next_state, state));
  }
}


// Remove a state from its column
template<typename T, typename U>
bool remove_state(const T& state, U& column)
{
  StateId id = state.id();
  auto itr = std::find_if(column.begin(), column.end(),
                          [id] (const T* state) {
                            return id == state->id();
                          });
  if (itr == column.end()) {
    return false;
  }
  column.erase(itr);
  return true;
}


template <typename T>
Time ViterbiSearch<T>::IterativeSearch(Time target, bool request_new_start)
{
  assert(!unreached_states_.empty() && target < unreached_states_.size());
  if (unreached_states_.empty()) {
    throw std::runtime_error("empty states");
  }

  if (target < winner_.size()) {
    return target;
  }

  // So here we have: assert(winner_.size() <= target && target < unreached_states_.size());

  Time source;

  // Initialize queue
  if (!request_new_start && !winner_.empty() && winner_.back()) {
    source = winner_.size() - 1;
    AddSuccessorsToQueue(winner_[source]);
  } else {
    source = winner_.size();
    InitQueue(unreached_states_[source]);
  }

  // Start with the source time, which will be searched anyhow
  auto searched_time = source;

  while (!queue_.empty()) {
    // Pop up the state with the lowest cost
    auto label = queue_.top();
    queue_.pop();
    auto state = label.state;
    auto time = state->time();

    // Mark it as scanned and remember its cost and predecessor
    assert(scanned_labels_.find(state->id())==scanned_labels_.end());
    scanned_labels_[state->id()] = label;

    // Remove it from its column
    bool removed = remove_state(*state, unreached_states_[time]);
    assert(removed);

    // If it's the first state that arrives at this column, mark it as
    // the winner at this time
    if (winner_.size() <= time) {
      assert(time == winner_.size());
      winner_.push_back(state);
    }

    searched_time = std::max(time, searched_time);

    if (target <= time) {
      break;
    }

    AddSuccessorsToQueue(state);
  }

  // Guarantee that either winner (if found) or nullptr is saved at
  // searched time
  while (winner_.size() <= searched_time) {
    winner_.push_back(nullptr);
  }

  assert(searched_time < winner_.size());

  return searched_time;
}
