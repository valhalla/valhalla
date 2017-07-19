#include <cstdint>
// -*- mode: c++ -*-
#include <iostream>
#include <chrono>
#include <random>

#include "test.h"
#include "meili/viterbi_search.h"

using namespace valhalla::meili;

struct State
{
  float emission_cost;
  std::unordered_map<uint32_t, float> transition_costs;
};

using Column = std::vector<State>;

void print_state(const StateId& stateid, const State& state)
{
  // [state_time state_id emission_cost (next_stateid transition_cost)...]
  std::cout << "[(";
  std::cout << stateid.time();
  std::cout << "," << stateid.id() << ")";
  std::cout << " $" << state.emission_cost;
  for (const auto& transition_cost : state.transition_costs) {
    std::cout << " (" << transition_cost.first << " $" << transition_cost.second << ")";
  }
  std::cout << "]";
}

void print_trellis_diagram_vertically(const std::vector<Column>& columns)
{
  StateId::Time time = 0;
  for (const auto& column : columns) {
    uint32_t idx = 0;
    for (const auto& state : column) {
      print_state(StateId(time, idx), state);
      std::cout << std::endl;
      idx++;
    }
    time++;
    std::cout << std::endl;
  }
}

void print_path_reversely(
    const std::vector<Column>& columns,
    const IViterbiSearch::stateid_iterator rbegin,
    const IViterbiSearch::stateid_iterator rend)
{
  for (auto stateid = rbegin; stateid != rend; stateid++) {
    if ((*stateid).IsValid()) {
      print_state(*stateid, columns[(*stateid).time()][(*stateid).id()]);
      std::cout << std::endl;
    } else {
      std::cout << "[NOT FOUND]" << std::endl;
    }
  }
}

void AddColumns(IViterbiSearch* ivs, const std::vector<Column>& columns)
{
  StateId::Time time = 0;
  for (const auto& column : columns) {
    uint32_t idx = 0;
    for (const auto& state : column) {
      StateId stateid(time, idx);
      const auto added = ivs->AddStateId(stateid);
      test::assert_bool(added, "must be added");
      test::assert_bool(ivs->HasStateId(stateid), "must contain it");
      idx++;
    }
    time++;
  }
}

class SimpleNaiveViterbiSearch: public NaiveViterbiSearch<false>
{
 public:
  SimpleNaiveViterbiSearch(const std::vector<Column>& columns)
      : columns_(columns)
  { AddColumns(this, columns); }

 protected:
  const State& GetState(const StateId& stateid) const
  { return columns_[stateid.time()][stateid.id()]; }

  static float NormalizeCost(float cost)
  { return cost < 0.0 ? kInvalidCost : cost; }

  float EmissionCost(const StateId& stateid) const
  { return NormalizeCost(GetState(stateid).emission_cost); }

  float TransitionCost(const StateId& lhs, const StateId& rhs) const
  {
    test::assert_bool(
        lhs.time() + 1 == rhs.time(),
        "left state should be earlier than right time");
    const auto& state = GetState(lhs);
    const auto it = state.transition_costs.find(rhs.id());
    return it == state.transition_costs.end() ? kInvalidCost : NormalizeCost(it->second);
  }

  double CostSofar(
      double prev_cost_sofar,
      float transition_cost,
      float emission_cost) const
  { return prev_cost_sofar + transition_cost + emission_cost; }

 private:
  std::vector<Column> columns_;
};

class SimpleViterbiSearch: public ViterbiSearch
{
 public:
  SimpleViterbiSearch(const std::vector<Column>& columns)
      : columns_(columns)
  { AddColumns(this, columns); }

 protected:
  const State& GetState(const StateId& stateid) const
  { return columns_[stateid.time()][stateid.id()]; }

  float EmissionCost(const StateId& stateid) const
  { return GetState(stateid).emission_cost; }

  float TransitionCost(const StateId& lhs, const StateId& rhs) const
  {
    test::assert_bool(
        lhs.time() + 1 == rhs.time(),
        "left state should be earlier than right time");
    const auto& state = GetState(lhs);
    const auto it = state.transition_costs.find(rhs.id());
    return it == state.transition_costs.end() ? -1.f : it->second;
  }

  double CostSofar(
      double prev_cost_sofar,
      float transition_cost,
      float emission_cost) const
  {
    test::assert_bool(
        0.f <= prev_cost_sofar
        && 0.f <= transition_cost
        && 0.f <= emission_cost,
        "all costs should be non-negative");
    return prev_cost_sofar + transition_cost + emission_cost;
  }

 private:
  std::vector<Column> columns_;
};

unsigned SEED = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine TRANSITION_COST_GENERATOR(SEED),
  EMISSION_COST_GENERATOR(SEED),
  COUNT_GENERATOR(SEED);

Column
generate_column(
    size_t num_states,
    std::uniform_int_distribution<int> transition_cost_distribution,
    std::uniform_int_distribution<int> emission_cost_distribution,
    // Generate transition costs to next column
    const Column& next_column)
{
  Column states;

  for (size_t _ = 0; _ < num_states; _++) {
    const auto emission_cost = static_cast<float>(emission_cost_distribution(EMISSION_COST_GENERATOR));
    std::unordered_map<uint32_t, float> transition_costs;
    for (uint32_t idx = 0; idx < next_column.size(); idx++) {
      const auto transition_cost = static_cast<float>(transition_cost_distribution(TRANSITION_COST_GENERATOR));
      transition_costs[idx] = transition_cost;
    }
    states.push_back({emission_cost, transition_costs});
  }

  return states;
}

std::vector<size_t>
generate_column_counts(
    size_t column_length,
    std::uniform_int_distribution<size_t> count_distribution)
{
  std::vector<size_t> counts;

  for (size_t i = 0; i < column_length; i++) {
    counts.push_back(count_distribution(COUNT_GENERATOR));
  }

  return counts;
}

std::vector<Column>
generate_columns(
    std::uniform_int_distribution<int> transition_cost_distribution,
    std::uniform_int_distribution<int> emission_cost_distribution,
    std::vector<size_t> column_counts)
{
  std::vector<Column> columns;

  for (const auto count : column_counts) {
    const auto& column = generate_column(
        count,
        transition_cost_distribution,
        emission_cost_distribution,
        columns.empty() ? Column() : columns.back());
    columns.push_back(column);
  }
  std::reverse(columns.begin(), columns.end());

  return columns;
}

void test_viterbi_search(const std::vector<Column>& columns)
{
  SimpleNaiveViterbiSearch na(columns);
  SimpleViterbiSearch vs(columns);

  for (StateId::Time time = 0; time < columns.size(); time++) {
    const auto& na_winner = na.SearchWinner(time);
    const auto& vs_winner = vs.SearchWinner(time);

    if (na_winner.IsValid()) {
      test::assert_bool(
          vs_winner.IsValid(),
          "both winners should be valid");

      test::assert_bool(
          na_winner.time() == time,
          "time should be matched");

      test::assert_bool(
          vs_winner.time() == time,
          "time should be matched");

      if (na.AccumulatedCost(na_winner) != vs.AccumulatedCost(vs_winner)) {
        std::cout << "GRAPH" << std::endl;
        print_trellis_diagram_vertically(columns);

        std::cout << "PATH OF NA" << std::endl;
        print_path_reversely(columns, na.SearchPath(time), na.PathEnd());

        std::cout << "PATH OF VS" << std::endl;
        print_path_reversely(columns, vs.SearchPath(time), vs.PathEnd());
      }

      test::assert_bool(
          na.AccumulatedCost(na_winner) == vs.AccumulatedCost(vs_winner),
          "costs should be both optimal"
          "but got na = " + std::to_string(na.AccumulatedCost(na_winner)) +
          " and vs = " + std::to_string(vs.AccumulatedCost(vs_winner)));
    } else {
      test::assert_bool(
          !vs_winner.IsValid(),
          "both winners should not be found");
    }
  }
}

void TestViterbiSearch()
{
  // small case for visualization
  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(-10, 30),
        // emission costs
        std::uniform_int_distribution<int>(-10, 30),
        generate_column_counts(
            5,
            // column sizes
            std::uniform_int_distribution<size_t>(1, 3)));
    // std::cout << std::endl;
    // print_trellis_diagram_vertically(columns);
    test_viterbi_search(columns);
  }

  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(0, 50),
        // emission costs
        std::uniform_int_distribution<int>(0, 100),
        generate_column_counts(
            1000,
            // column sizes
            std::uniform_int_distribution<size_t>(1, 100)));
    test_viterbi_search(columns);
  }

  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(-50, 10),
        // emission costs
        std::uniform_int_distribution<int>(-100, 10),
        generate_column_counts(
            1000,
            // column sizes
            std::uniform_int_distribution<size_t>(0, 100)));
    test_viterbi_search(columns);
  }

  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(-30, -3),
        // emission costs
        std::uniform_int_distribution<int>(3, 30),
        generate_column_counts(
            1000,
            // column sizes
            std::uniform_int_distribution<size_t>(0, 100)));
    test_viterbi_search(columns);
  }

  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(3, 30),
        // emission costs
        std::uniform_int_distribution<int>(-30, -3),
        generate_column_counts(
            1000,
            // column sizes
            std::uniform_int_distribution<size_t>(0, 100)));
    test_viterbi_search(columns);
  }

  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(0, 1000),
        // emission costs
        std::uniform_int_distribution<int>(0, 3000),
        generate_column_counts(
            1000,
            // column sizes
            std::uniform_int_distribution<size_t>(1, 100)));
    test_viterbi_search(columns);
  }
}

int main(int argc, char *argv[])
{
  test::suite suite("viterbi search");

  suite.test(TEST_CASE(TestViterbiSearch));

  return suite.tear_down();
}
