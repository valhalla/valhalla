// -*- mode: c++ -*-
#ifndef MMP_VITERBI_SEARCH_H_
#define MMP_VITERBI_SEARCH_H_

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>

#include <valhalla/meili/priority_queue.h>
#include <valhalla/meili/stateid.h>

namespace valhalla{
namespace meili {

struct StateLabel
{
 public:
  // Required by SPQueue
  using id_type = StateId;

  StateLabel(double costsofar,
             const id_type& stateid,
             const id_type& predecessor)
      : costsofar_(costsofar),
        stateid_(stateid),
        predecessor_(predecessor)
  {
    if (!stateid.IsValid()) {
      throw std::invalid_argument("expect valid stateid");
    }
  }

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

class IViterbiSearch;

// TODO test it
class StateIdIterator: public std::iterator<std::forward_iterator_tag, StateId>
{
 public:
  StateIdIterator(IViterbiSearch& vs, StateId::Time time, const StateId& stateid, bool allow_breaks = true)
      : vs_(vs),
        time_(time),
        stateid_(stateid),
        allow_breaks_(allow_breaks)
  { ValidateStateId(time, stateid); }

  StateIdIterator(IViterbiSearch& vs)
      : StateIdIterator(vs, kInvalidTime, StateId())
  {}

  // Postfix increment
  StateIdIterator operator++(int)
  {
    auto copy = *this;
    Next();
    return copy;
  }

  // Prefix increment
  StateIdIterator operator++()
  {
    Next();
    return *this;
  }

  bool operator==(const StateIdIterator& other) const
  { return &vs_ == &(other.vs_) && time_ == other.time_ && stateid_ == other.stateid_; }

  bool operator!=(const StateIdIterator& other) const
  { return !(*this == other); }

  const StateId& operator*() const
  { return stateid_; }

 private:
  // Invariant
  void ValidateStateId(const StateId::Time time, const StateId& stateid)
  {
    if (time == kInvalidTime) {
      if (stateid.IsValid()) {
        throw std::runtime_error("expect invalid stateid");
      }
    } else {
      if (stateid.IsValid()) {
        if (stateid.time() != time) {
          throw std::runtime_error("time is not matched");
        }
      }
    }
  }

  IViterbiSearch& vs_;

  StateId::Time time_;

  StateId stateid_;

  bool allow_breaks_;

  void Next();
};

using IEmissionCostModel = std::function<float(const StateId& stateid)>;

inline float DefaultEmissionCostModel(const StateId&)
{ return 0; }

using ITransitionCostModel = std::function<float(const StateId& lhs, const StateId& rhs)>;

inline float DefaultTransitionCostModel(const StateId&, const StateId&)
{ return 1; }

class IViterbiSearch
{
 public:
  using stateid_iterator = StateIdIterator;

  IViterbiSearch(
      const IEmissionCostModel& emission_cost_model,
      const ITransitionCostModel& transition_cost_model)
      : emission_cost_model_(emission_cost_model),
        transition_cost_model_(transition_cost_model),
        path_end_(stateid_iterator(*this))
  {}

  IViterbiSearch()
      : IViterbiSearch(DefaultEmissionCostModel, DefaultTransitionCostModel) {}

  virtual ~IViterbiSearch()
  { Clear(); };

  virtual void Clear()
  { added_states_.clear(); }

  virtual void ClearSearch() {};

  virtual bool AddStateId(const StateId& stateid)
  { return added_states_.insert(stateid).second; }

  /**
   * Remove a state ID. Note that if an ID is removed, client must call ClearSearch before new search.
   *
   * @return true if it's removed
   */
  virtual bool RemoveStateId(const StateId& stateid)
  {
    return 0 < added_states_.erase(stateid);
  }

  virtual bool HasStateId(const StateId& stateid) const
  { return added_states_.find(stateid) != added_states_.end(); }

  virtual StateId SearchWinner(StateId::Time time) = 0;

  stateid_iterator SearchPath(StateId::Time time, bool allow_breaks = true)
  { return stateid_iterator(*this, time, SearchWinner(time), allow_breaks); }

  stateid_iterator PathEnd() const
  { return path_end_; }

  const IEmissionCostModel& emission_cost_model() const
  { return emission_cost_model_; }

  void set_emission_cost_model(const IEmissionCostModel cost_model)
  { emission_cost_model_ = cost_model; }

  const ITransitionCostModel& transition_cost_model() const
  { return transition_cost_model_; }

  void set_transition_cost_model(const ITransitionCostModel cost_model)
  { transition_cost_model_ = cost_model; }

  virtual StateId Predecessor(const StateId& stateid) const = 0;

  virtual double AccumulatedCost(const StateId& stateid) const = 0;

 protected:
  // Calculate transition cost from left state to right state
  virtual float TransitionCost(const StateId& lhs, const StateId& rhs) const
  { return transition_cost_model_(lhs, rhs); }

  // Calculate emission cost of a state
  virtual float EmissionCost(const StateId& stateid) const
  { return emission_cost_model_(stateid); }

  // Calculate the a state's costsofar based on its predecessor's
  // costsofar, transition cost from predecessor to this state,
  // and emission cost of this state
  virtual double CostSofar(
      double prev_costsofar,
      float transition_cost,
      float emission_cost) const
  { return prev_costsofar + transition_cost + emission_cost; }

 private:
  std::unordered_set<StateId> added_states_;

  IEmissionCostModel emission_cost_model_;

  ITransitionCostModel transition_cost_model_;

  const stateid_iterator path_end_;
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

  void Clear() override;

  void ClearSearch() override;

  bool AddStateId(const StateId& stateid) override;

  bool RemoveStateId(const StateId& stateid) override
  {
    const auto removed = IViterbiSearch::RemoveStateId(stateid);
    if (!removed) {
      return false;
    }
    // remove it from columns
    auto& column = states_[stateid.time()];
    const auto it = std::find(column.begin(), column.end(), stateid);
    column.erase(it);
    return true;
  }

  StateId SearchWinner(StateId::Time time) override;

  StateId Predecessor(const StateId& stateid) const override;

  double AccumulatedCost(const StateId& stateid) const override;

 private:
  std::vector<std::vector<StateId>> states_;

  std::vector<StateId> winner_;

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
  ViterbiSearch(
      const IEmissionCostModel& emission_cost_model,
      const ITransitionCostModel& transition_cost_model)
      : IViterbiSearch(emission_cost_model, transition_cost_model),
        earliest_time_(0) {}

  ViterbiSearch()
      : ViterbiSearch(DefaultEmissionCostModel, DefaultTransitionCostModel) {}

  ~ViterbiSearch()
  { Clear(); }

  void Clear() override;

  void ClearSearch() override;

  bool AddStateId(const StateId& stateid) override;

  bool RemoveStateId(const StateId& stateid) override
  {
    const auto removed = IViterbiSearch::RemoveStateId(stateid);
    if (!removed) {
      return false;
    }
    // remove it from columns
    auto& column = states_[stateid.time()];
    const auto it = std::find(column.begin(), column.end(), stateid);
    column.erase(it);
    return true;
  }

  StateId SearchWinner(StateId::Time time) override;

  StateId Predecessor(const StateId& stateid) const override;

  virtual bool IsInvalidCost(double cost) const
  { return cost < 0.f; }

  using IViterbiSearch::AccumulatedCost;

  virtual double AccumulatedCost(const StateId& stateid) const override;

 private:
  std::vector<std::vector<StateId>> states_;

  std::vector<StateId> winner_;

  std::vector<std::vector<StateId>> unreached_states_;

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
