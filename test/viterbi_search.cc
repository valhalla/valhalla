#include <cstdint>
// -*- mode: c++ -*-
#include <iostream>
#include <chrono>
#include <random>

#include "test.h"
#include "meili/viterbi_search.h"

using namespace valhalla::meili;


using ObjectId = uint32_t;

class Candidate
{
 public:
  Candidate(ObjectId id)
      : id_(id), emission_cost_(-1.f) {}

  Candidate(ObjectId id, float emission_cost)
      : id_(id), emission_cost_(emission_cost) {}

  ObjectId id() const
  { return id_; }

  float emission_cost() const
  { return emission_cost_; }

  float transition_cost(ObjectId id) const
  {
    auto iter = transition_cost_.find(id);
    if (iter == transition_cost_.end()) {
      return -1.f;
    }
    return iter->second;
  }

  void set_transition_cost(ObjectId id, float cost)
  { transition_cost_[id] = cost; }

 protected:
  std::unordered_map<ObjectId, float> transition_cost_;

 private:
  ObjectId id_;
  float emission_cost_;
};


bool operator==(const Candidate& lhs, const Candidate& rhs)
{ return lhs.id() == rhs.id() && lhs.emission_cost() == rhs.emission_cost(); }


class State
{
 public:
  State(StateId id, Time time, const Candidate& candidate)
      : id_(id), time_(time), candidate_(candidate) {}

  const Time time() const
  { return time_; }

  const StateId id() const
  { return id_; }

  const Candidate& candidate() const
  { return candidate_; }

 private:
  const StateId id_;
  const Time time_;
  const Candidate candidate_;
};


class SimpleViterbiSearch: public ViterbiSearch<State>
{
 public:
  template <typename candidate_iterator_t>
  Time AppendState(candidate_iterator_t begin, candidate_iterator_t end)
  {
    std::vector<const State*> column;
    Time time = unreached_states_.size();
    for (auto candidate = begin; candidate != end; candidate++) {
      auto candidate_id = state_.size();
      state_.push_back(new State(candidate_id, time, *candidate));
      column.push_back(state_.back());
    }
    unreached_states_.push_back(column);
    return time;
  }

 protected:
  float TransitionCost(const State& left, const State& right) const
  {
    test::assert_bool(left.time() + 1 == right.time(),
                      "left state should be earlier than right time");
    const auto right_id = right.candidate().id();
    return left.candidate().transition_cost(right_id);
  }

  float EmissionCost(const State& candidate) const
  { return candidate.candidate().emission_cost(); }

  double CostSofar(double prev_cost_sofar,
                   float transition_cost,
                   float emission_cost) const
  { return prev_cost_sofar + transition_cost + emission_cost; }
};


class SimpleNaiveViterbiSearch: public NaiveViterbiSearch<State, false>
{
 public:
  template <typename candidate_iterator_t>
  Time AppendState(candidate_iterator_t begin, candidate_iterator_t end)
  {
    std::vector<const State*> column;
    Time time = states_.size();
    for (auto candidate = begin; candidate != end; candidate++) {
      auto candidate_id = state_.size();
      state_.push_back(new State(candidate_id, time, *candidate));
      column.push_back(state_.back());
    }
    states_.push_back(column);
    return time;
  }

 protected:
  float TransitionCost(const State& left, const State& right) const
  {
    test::assert_bool(left.time() + 1 == right.time(),
                      "left time should be eariler than right time");
    auto right_id = right.candidate().id();
    auto cost = left.candidate().transition_cost(right_id);
    return cost < 0.f? kInvalidCost : cost;
  }

  float EmissionCost(const State& candidate) const
  {
    auto cost = candidate.candidate().emission_cost();
    return cost < 0.f? kInvalidCost : cost;
  }

  double CostSofar(double prev_cost_sofar,
                   float transition_cost,
                   float emission_cost) const
  {
    test::assert_bool(0.f <= prev_cost_sofar
                      && 0.f <= transition_cost
                      && 0.f <= emission_cost,
                      "all costs should be non-negative");
    return prev_cost_sofar + transition_cost + emission_cost;
  }
};


unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine TRANSITION_COST_GENERATOR(seed),
  EMISSION_COST_GENERATOR(seed),
  COUNT_GENERATOR(seed);


std::vector<Candidate>
generate_candidates(ObjectId *start_id,
                    size_t num_candidates,
                    std::uniform_int_distribution<int> transition_cost_distribution,
                    std::uniform_int_distribution<int> emission_cost_distribution,
                    // Generate transition costs to next candidates
                    const std::vector<Candidate>& next_candidates)
{
  std::vector<Candidate> candidates;

  for (size_t i = 0; i < num_candidates; i++) {
    auto emission_cost = static_cast<float>(emission_cost_distribution(EMISSION_COST_GENERATOR));
    candidates.emplace_back((*start_id)++, emission_cost);
    for (const auto& candidate : next_candidates) {
      auto transition_cost = static_cast<float>(transition_cost_distribution(TRANSITION_COST_GENERATOR));
      candidates.back().set_transition_cost(candidate.id(), transition_cost);
    }
  }

  return candidates;
}


void print_state(const std::vector<Candidate>& state,
                 const std::vector<Candidate>& next_state)
{
  for (const auto& candidate : state) {
    std::cout << candidate.id() << "(" << candidate.emission_cost() << "): ";
    for (const auto& next_candidate : next_state) {
      if (candidate.transition_cost(next_candidate.id()) >= 0.f) {
        std::cout << next_candidate.id() << "(" << candidate.transition_cost(next_candidate.id()) << ") ";
      }
    }
    std::cout << std::endl;
  }
}


void print_state(const std::vector<Candidate>& state)
{
  for (const auto& candidate : state) {
    std::cout << candidate.id() << "(" << candidate.emission_cost() << ")" << std::endl;
  }
}


void print_trellis_diagram_vertically(const std::vector<std::vector<Candidate>>& states)
{
  for (auto cursor=states.begin(); cursor < states.end(); cursor++) {
    if (cursor+1 != states.end()) {
      print_state(*cursor, *(cursor+1));
    } else {
      print_state(*cursor);
    }
    std::cout << std::endl;
  }
}


void print_path(const std::vector<const State*>& path)
{
  for (const auto candidate_ptr : path) {
    if (candidate_ptr) {
      std::cout << candidate_ptr->candidate().id() << " ";
    }
  }
  std::cout << std::endl;
}


void test_viterbi_search(std::uniform_int_distribution<int> transition_cost_distribution,
                         std::uniform_int_distribution<int> emission_cost_distribution,
                         std::vector<size_t> candidate_counts)
{
  // Generate candidates for testing
  ObjectId start_id = 0;
  std::vector<Candidate> prev_candidates;
  std::vector<std::vector<Candidate>> candidate_lists;
  for (const auto count : candidate_counts) {
    const auto& candidates = generate_candidates(&start_id, count,
                                                 transition_cost_distribution,
                                                 emission_cost_distribution,
                                                 prev_candidates);
    prev_candidates = candidates;
    candidate_lists.push_back(candidates);
  }
  std::reverse(candidate_lists.begin(), candidate_lists.end());

  // print_trellis_diagram_vertically(states);

  // Test viterbi search
  SimpleNaiveViterbiSearch snvs;
  SimpleViterbiSearch svs;
  for (const auto& candidate_list : candidate_lists) {
    auto svs_time = svs.AppendState(candidate_list.cbegin(), candidate_list.cend()),
        snvs_time = snvs.AppendState(candidate_list.cbegin(), candidate_list.cend());
    test::assert_bool(svs_time == snvs_time,
                      "time should be matched");

    auto snvs_id = snvs.SearchWinner(snvs_time);
    const State* snvs_winner = snvs_id == kInvalidStateId? nullptr : &snvs.state(snvs_id);
    auto svs_id = svs.SearchWinner(svs_time);
    const State* svs_winner = svs_id == kInvalidStateId? nullptr : &svs.state(svs_id);

    if (svs_winner) {
      test::assert_bool(svs_winner->time() == svs_time && snvs_winner->time() == snvs_time,
                        "time should be matched");
      // Gurantee that both costs are the same
      test::assert_bool(svs.AccumulatedCost(*svs_winner) == snvs.AccumulatedCost(*snvs_winner),
                        "costs should be both same");
    } else {
      test::assert_bool(!snvs_winner,
                        "both winner should not be found");
    }
  }
}


std::vector<size_t>
generate_candidate_counts(size_t length,
                          std::uniform_int_distribution<size_t> count_distribution)
{
  std::vector<size_t> counts;
  for (size_t i = 0; i < length; i++) {
    counts.push_back(count_distribution(COUNT_GENERATOR));
  }
  return counts;
}


void TestViterbiSearch()
{
  std::uniform_int_distribution<int> transition_cost_distribution(0, 50);
  std::uniform_int_distribution<int> emission_cost_distribution(0, 100);
  std::uniform_int_distribution<size_t> count_distribution(1, 100);
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution,
                      generate_candidate_counts(1000, count_distribution));

  transition_cost_distribution = std::uniform_int_distribution<int>(-50, 10);
  emission_cost_distribution = std::uniform_int_distribution<int>(-100, 10);
  count_distribution = std::uniform_int_distribution<size_t>(0, 100);
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution,
                      generate_candidate_counts(1000, count_distribution));

  transition_cost_distribution = std::uniform_int_distribution<int>(-30, -3);
  emission_cost_distribution = std::uniform_int_distribution<int>(3, 30);
  count_distribution = std::uniform_int_distribution<size_t>(0, 100);
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution,
                      generate_candidate_counts(1000, count_distribution));

  transition_cost_distribution = std::uniform_int_distribution<int>(3, 30);
  emission_cost_distribution = std::uniform_int_distribution<int>(-30, -3);
  count_distribution = std::uniform_int_distribution<size_t>(0, 100);
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution,
                      generate_candidate_counts(1000, count_distribution));

  transition_cost_distribution = std::uniform_int_distribution<int>(0, 1000);
  emission_cost_distribution = std::uniform_int_distribution<int>(0, 3000);
  count_distribution = std::uniform_int_distribution<size_t>(1, 100);
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution,
                      generate_candidate_counts(1000, count_distribution));
}


int main(int argc, char *argv[])
{
  test::suite suite("viterbi search");

  suite.test(TEST_CASE(TestViterbiSearch));

  return suite.tear_down();
}
