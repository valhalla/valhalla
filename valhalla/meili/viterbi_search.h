// -*- mode: c++ -*-
#ifndef MMP_VITERBI_SEARCH_H_
#define MMP_VITERBI_SEARCH_H_

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <valhalla/meili/priority_queue.h>
#include <valhalla/meili/stateid.h>

namespace valhalla {
namespace meili {

using IEmissionCostModel = std::function<float(const StateId& stateid)>;
using ITransitionCostModel = std::function<float(const StateId& lhs, const StateId& rhs)>;
constexpr float DefaultEmissionCostModel(const StateId&) {
  return 0.0;
}
constexpr float DefaultTransitionCostModel(const StateId&, const StateId&) {
  return 1.0;
}

class StateLabel {
public:
  using id_type = StateId;
  // Required by SPQueue
  StateLabel(double costsofar, const StateId& stateid, const StateId& predecessor);

  double costsofar() const;
  StateId stateid() const;
  StateId predecessor() const;
  /* The following operator overloading and member functions are Required by SPQueue */
  StateId id() const;
  bool operator<(const StateLabel& rhs) const;
  bool operator>(const StateLabel& rhs) const;
  bool operator==(const StateLabel& rhs) const;

private:
  StateId stateid_{};
  StateId predecessor_{};
  double costsofar_{0.0}; // Accumulated cost since time = 0
};

class IViterbiSearch;

// TODO test it
class StateIdIterator : public std::iterator<std::forward_iterator_tag, StateId> {
public:
  StateIdIterator(IViterbiSearch& vs,
                  StateId::Time time,
                  const StateId& stateid,
                  bool allow_breaks = true);
  StateIdIterator(IViterbiSearch& vs);

  StateIdIterator operator++(int);
  StateIdIterator operator++();
  bool operator==(const StateIdIterator& other) const;
  bool operator!=(const StateIdIterator& other) const;
  const StateId& operator*() const;

private:
  void static ValidateStateId(const StateId::Time time, const StateId& stateid); // Invariant
  void Next();

  IViterbiSearch& vs_;
  StateId::Time time_{kInvalidTime};
  StateId stateid_{};
  bool allow_breaks_{true};
};

class IViterbiSearch {
public:
  using stateid_iterator = StateIdIterator;

  IViterbiSearch(const IEmissionCostModel& emission_cost_model,
                 const ITransitionCostModel& transition_cost_model);
  IViterbiSearch();
  virtual ~IViterbiSearch();

  virtual void Clear();
  virtual void ClearSearch() = 0;
  virtual bool AddStateId(const StateId& stateid);
  /**
   * Remove a state ID. Note that if an ID is removed, client must call ClearSearch before new
   * search.
   *
   * @return true if it's removed
   */
  virtual bool RemoveStateId(const StateId& stateid);
  virtual StateId SearchWinner(StateId::Time time) = 0;
  virtual StateId Predecessor(const StateId& stateid) const = 0;
  virtual double AccumulatedCost(const StateId& stateid) const = 0;

  bool HasStateId(const StateId& stateid) const;
  StateIdIterator SearchPath(StateId::Time time, bool allow_breaks = true);
  StateIdIterator PathEnd() const;
  const IEmissionCostModel& emission_cost_model() const;
  void set_emission_cost_model(const IEmissionCostModel cost_model);
  const ITransitionCostModel& transition_cost_model() const;
  void set_transition_cost_model(const ITransitionCostModel cost_model);

protected:
  // Calculate transition cost from left state to right state
  float TransitionCost(const StateId& lhs, const StateId& rhs) const;
  // Calculate emission cost of a state
  float EmissionCost(const StateId& stateid) const;
  /* Calculate the a state's costsofar based on its predecessor's
     costsofar, transition cost from predecessor to this state,
     and emission cost of this state */
  constexpr static double
  CostSofar(double prev_costsofar, float transition_cost, float emission_cost);

  std::vector<std::vector<StateId>> states_by_time;
  std::vector<StateId> winner_by_time;

private:
  std::unordered_set<StateId> added_states_;
  IEmissionCostModel emission_cost_model_;
  ITransitionCostModel transition_cost_model_;
  const stateid_iterator path_end_;
};

template <bool Maximize> class NaiveViterbiSearch : public IViterbiSearch {
public:
  static constexpr double kInvalidCost =
      Maximize ? -std::numeric_limits<double>::infinity()
               : std::numeric_limits<double>::infinity(); // An invalid costsofar indicates that a
                                                          // state is unreachable

  ~NaiveViterbiSearch();

  void Clear() override;
  void ClearSearch() override;
  bool AddStateId(const StateId& stateid) override;
  bool RemoveStateId(const StateId& stateid) override;
  StateId SearchWinner(StateId::Time time) override;
  StateId Predecessor(const StateId& stateid) const override;
  double AccumulatedCost(const StateId& stateid) const override;

private:
  void UpdateLabels(std::vector<StateLabel>& labels,
                    const std::vector<StateLabel>& prev_labels) const;
  std::vector<StateLabel> InitLabels(const std::vector<StateId>& column,
                                     bool use_emission_cost) const;
  StateId FindWinner(const std::vector<StateLabel>& labels) const;
  const StateLabel& GetLabel(const StateId& stateid) const;

  std::vector<std::vector<StateLabel>> history_;
};

class ViterbiSearch : public IViterbiSearch {
public:
  ViterbiSearch(const IEmissionCostModel& emission_cost_model,
                const ITransitionCostModel& transition_cost_model);
  ViterbiSearch();
  ~ViterbiSearch();

  void Clear() override;
  void ClearSearch() override;
  bool AddStateId(const StateId& stateid) override;
  bool RemoveStateId(const StateId& stateid) override;
  StateId SearchWinner(StateId::Time time) override;
  StateId Predecessor(const StateId& stateid) const override;
  double AccumulatedCost(const StateId& stateid) const override;

private:
  // Initialize labels from a column and push them into priority queue
  void InitQueue(const std::vector<StateId>& column);
  void AddSuccessorsToQueue(const StateId& stateid);
  StateId::Time IterativeSearch(StateId::Time target, bool request_new_start);
  constexpr static bool IsInvalidCost(double cost);

  std::vector<std::vector<StateId>> unreached_states_by_time;
  std::unordered_map<StateId, StateLabel> scanned_labels_;
  SPQueue<StateLabel> queue_;
  StateId::Time earliest_time_{0};
};
} // namespace meili
} // namespace valhalla
#endif // MMP_VITERBI_SEARCH_H_
