// -*- mode: c++ -*-
#ifndef MMP_VITERBI_SEARCH_H_
#define MMP_VITERBI_SEARCH_H_

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <valhalla/meili/priority_queue.h>
#include <valhalla/meili/stateid.h>

namespace {

struct StateLabel
{
 public:
  // Required by SPQueue
  using id_type = valhalla::meili::StateId;

  StateLabel(double costsofar,
             const id_type& stateid,
             const id_type& predecessor)
      : costsofar_(costsofar),
        stateid_(stateid),
        predecessor_(predecessor) {}

  // Required by SPQueue
  const id_type& id() const
  { return stateid_; }

  double costsofar() const
  { return costsofar_; }

  const id_type& stateid() const
  { return stateid_; }

  const id_type& predecessor() const
  { return predecessor_; }

  // Required by SPQueue
  bool operator<(const StateLabel& rhs) const
  { return costsofar_ < rhs.costsofar_; }

  // Required by SPQueue
  bool operator>(const StateLabel& rhs) const
  { return costsofar_ > rhs.costsofar_; }

  // Required by SPQueue
  bool operator==(const StateLabel& rhs) const
  { return costsofar_ == rhs.costsofar_; }

 private:
  // Accumulated cost since time = 0
  double costsofar_;

  id_type stateid_;

  id_type predecessor_;
};

}

namespace valhalla{
namespace meili {

class IViterbiSearch;

// TODO test it
class StateIterator: public std::iterator<std::forward_iterator_tag, StateId>
{
 public:
  StateIterator(IViterbiSearch& vs, StateId::Time time, const StateId& stateid)
      : vs_(vs),
        time_(time),
        stateid_(stateid)
  {
    // Hold the invariant in the beginning (see below about the
    // invariant)
    if (time_ == StateId::kInvalidTime) {
      if (stateid_.IsValid()) {
        throw std::runtime_error("expect invalid stateid");
      }
    } else {
      if (stateid_.IsValid()) {
        if (stateid_.time() != time_) {
          throw std::runtime_error("time is not matched");
        }
      }
    }
  }

  StateIterator(IViterbiSearch& vs)
      : StateIterator(vs, StateId::kInvalidTime, StateId())
  {}

  // Postfix increment
  StateIterator operator++(int)
  {
    auto copy = *this;
    Next();
    return copy;
  }

  // Prefix increment
  StateIterator operator++()
  {
    Next();
    return *this;
  }

  bool operator==(const StateIterator& other) const
  { return &vs_ == &(other.vs_) && time_ == other.time_ && stateid_ == other.stateid_; }

  bool operator!=(const StateIterator& other) const
  { return !(*this == other); }

  const StateId& operator*() const
  { return stateid_; }

 private:
  IViterbiSearch& vs_;

  // invariant:

  // if (time_ is invalid) {
  //   stateid_ must be invalid;
  // } else {
  //   if (stateid_ is valid) {
  //     assert stateid_.time() == time_;
  //   }
  // }
  StateId::Time time_;

  StateId stateid_;

  void Next();
};

class IViterbiSearch
{
 public:
  using state_iterator = StateIterator;

  IViterbiSearch()
      : path_end_(state_iterator(*this))
  {}

  virtual ~IViterbiSearch() {};

  virtual bool AddStateId(const StateId& stateid)
  { return added_states_.insert(stateid).second; }

  virtual bool HasStateId(const StateId& stateid) const
  { return added_states_.find(stateid) != added_states_.end(); }

  virtual StateId SearchWinner(StateId::Time time) = 0;

  state_iterator SearchPath(StateId::Time time)
  { return state_iterator(*this, time, SearchWinner(time)); }

  state_iterator PathEnd() const
  { return path_end_; }

  virtual StateId Predecessor(const StateId& stateid) const = 0;

  virtual double AccumulatedCost(const StateId& stateid) const = 0;

 protected:
  // Calculate transition cost from left state to right state
  virtual float TransitionCost(const StateId& lhs, const StateId& rhs) const = 0;

  // Calculate emission cost of a state
  virtual float EmissionCost(const StateId& state) const = 0;

  // Calculate the a state's costsofar based on its predecessor's
  // costsofar, transition cost from predecessor to this state,
  // and emission cost of this state
  virtual double CostSofar(double prev_costsofar,
                           float transition_cost,
                           float emission_cost) const = 0;

  std::unordered_set<StateId> added_states_;

 private:
  const state_iterator path_end_;
};

template <bool Maximize>
class NaiveViterbiSearch: public IViterbiSearch
{
 public:
  // An invalid costsofar indicates that a state is unreachable
  static constexpr double
  kInvalidCost = Maximize? -std::numeric_limits<double>::infinity()
      : std::numeric_limits<double>::infinity();

  ~NaiveViterbiSearch()
  { Clear(); }

  void Clear();

  bool AddStateId(const StateId& stateid) override;

  StateId SearchWinner(StateId::Time time) override;

  StateId Predecessor(const StateId& stateid) const override;

  double AccumulatedCost(const StateId& stateid) const override;

 protected:
  std::vector<std::vector<StateId>> states_;

  std::vector<StateId> winner_;

  virtual float TransitionCost(const StateId& lhs, const StateId& rhs) const override = 0;

  virtual float EmissionCost(const StateId& stateid) const override = 0;

  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  std::vector<std::vector<StateLabel>> history_;

  void UpdateLabels(
      std::vector<StateLabel>& labels,
      const std::vector<StateLabel>& prev_labels) const;

  std::vector<StateLabel> InitLabels(
      const std::vector<StateId>& column,
      bool use_emission_cost) const;

  StateId FindWinner(const std::vector<StateLabel>& labels) const;

  const StateLabel& GetLabel(const StateId& stateid) const;
};

class ViterbiSearch: public IViterbiSearch
{
 public:
  ViterbiSearch(): earliest_time_(0) {}

  ~ViterbiSearch()
  { Clear(); }

  void Clear();

  bool AddStateId(const StateId& stateid) override;

  StateId SearchWinner(StateId::Time time) override;

  StateId Predecessor(const StateId& stateid) const override;

  virtual bool IsInvalidCost(double cost) const
  { return cost < 0.f; }

  using IViterbiSearch::AccumulatedCost;

  virtual double AccumulatedCost(const StateId& stateid) const override;

 protected:
  std::vector<StateId> winner_;

  std::vector<std::vector<StateId>> unreached_states_;

  virtual float TransitionCost(const StateId& lhs, const StateId& rhs) const override = 0;

  virtual float EmissionCost(const StateId& state) const override = 0;

  virtual double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override = 0;

 private:
  SPQueue<StateLabel> queue_;

  std::unordered_map<StateId, StateLabel> scanned_labels_;

  // Initialize labels from a column and push them into priority queue
  void InitQueue(const std::vector<StateId>& column);

  void AddSuccessorsToQueue(const StateId& stateid);

  StateId::Time IterativeSearch(StateId::Time target, bool request_new_start);

  StateId::Time earliest_time_;
};

}
}
#endif // MMP_VITERBI_SEARCH_H_
