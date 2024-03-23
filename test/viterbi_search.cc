#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>

#include "meili/topk_search.h"
#include "meili/viterbi_search.h"

#include "test.h"

// Viterbi and topK search tests

using namespace valhalla::meili;

struct State {
  float emission_cost;
  std::unordered_map<uint32_t, float> transition_costs;
};

using Column = std::vector<State>;

void print_state(const StateId& stateid, const State& state) {
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

void print_trellis_diagram_vertically(const std::vector<Column>& columns) {
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

template <typename iterator_t>
void print_path_reversely(const std::vector<Column>& columns, iterator_t rbegin, iterator_t rend) {
  for (auto stateid = rbegin; stateid != rend; stateid++) {
    if ((*stateid).IsValid()) {
      print_state(*stateid, columns[(*stateid).time()][(*stateid).id()]);
      std::cout << std::endl;
    } else {
      std::cout << "[NOT FOUND]" << std::endl;
    }
  }
}

template <typename iterator_t> void print_path(iterator_t rbegin, iterator_t rend) {
  for (auto it = rbegin; it != rend; it++) {
    std::cout << it->time() << "/" << it->id() << " ";
  }
  std::cout << std::endl;
}

void AddColumns(IViterbiSearch& vs, const std::vector<Column>& columns) {
  StateId::Time time = 0;
  for (const auto& column : columns) {
    for (uint32_t idx = 0; idx < column.size(); ++idx) {
      StateId stateid(time, idx);
      const auto added = vs.AddStateId(stateid);

      ASSERT_TRUE(added) << std::to_string(stateid.time()) << "/" + std::to_string(stateid.id())
                         << " must be added";

      ASSERT_TRUE(vs.HasStateId(stateid)) << "must contain it";
    }
    time++;
  }
}

class SimpleNaiveViterbiSearch : public NaiveViterbiSearch<false> {
public:
  SimpleNaiveViterbiSearch(const std::vector<Column>& columns) : columns_(columns) {
    AddColumns(*this, columns);
  }

protected:
  const State& GetState(const StateId& stateid) const {
    return columns_[stateid.time()][stateid.id()];
  }

  static float NormalizeCost(float cost) {
    return cost < 0.0 ? kInvalidCost : cost;
  }

  float EmissionCost(const StateId& stateid) const {
    return NormalizeCost(GetState(stateid).emission_cost);
  }

  float TransitionCost(const StateId& lhs, const StateId& rhs) const {
    EXPECT_EQ(lhs.time() + 1, rhs.time()) << "left state should be earlier than right time";
    const auto& state = GetState(lhs);
    const auto it = state.transition_costs.find(rhs.id());
    return it == state.transition_costs.end() ? kInvalidCost : NormalizeCost(it->second);
  }

  double CostSofar(double prev_cost_sofar, float transition_cost, float emission_cost) const {
    return prev_cost_sofar + transition_cost + emission_cost;
  }

private:
  std::vector<Column> columns_;
};

class SimpleViterbiSearch : public ViterbiSearch {
public:
  SimpleViterbiSearch(const std::vector<Column>& columns) : columns_(columns) {
    AddColumns(*this, columns);
  }

protected:
  const State& GetState(const StateId& stateid) const {
    return columns_[stateid.time()][stateid.id()];
  }

  float EmissionCost(const StateId& stateid) const {
    return GetState(stateid).emission_cost;
  }

  float TransitionCost(const StateId& lhs, const StateId& rhs) const {
    EXPECT_EQ(lhs.time() + 1, rhs.time()) << "left state should be earlier than right time";
    const auto& state = GetState(lhs);
    const auto it = state.transition_costs.find(rhs.id());
    return it == state.transition_costs.end() ? -1.f : it->second;
  }

  double CostSofar(double prev_cost_sofar, float transition_cost, float emission_cost) const {
    // all costs should be non-negative
    EXPECT_GE(prev_cost_sofar, 0.f);
    EXPECT_GE(transition_cost, 0.f);
    EXPECT_GE(emission_cost, 0.f);

    return prev_cost_sofar + transition_cost + emission_cost;
  }

private:
  std::vector<Column> columns_;
};

unsigned SEED = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 TRANSITION_COST_GENERATOR(SEED), EMISSION_COST_GENERATOR(SEED), COUNT_GENERATOR(SEED);

Column generate_column(size_t num_states,
                       std::uniform_int_distribution<int> transition_cost_distribution,
                       std::uniform_int_distribution<int> emission_cost_distribution,
                       // Generate transition costs to next column
                       const Column& next_column) {
  Column states;

  for (size_t _ = 0; _ < num_states; _++) {
    const auto emission_cost =
        static_cast<float>(emission_cost_distribution(EMISSION_COST_GENERATOR));
    std::unordered_map<uint32_t, float> transition_costs;
    for (uint32_t idx = 0; idx < next_column.size(); idx++) {
      const auto transition_cost =
          static_cast<float>(transition_cost_distribution(TRANSITION_COST_GENERATOR));
      transition_costs[idx] = transition_cost;
    }
    states.push_back({emission_cost, transition_costs});
  }

  return states;
}

std::vector<size_t> generate_column_counts(size_t column_length,
                                           std::uniform_int_distribution<size_t> count_distribution) {
  std::vector<size_t> counts;

  for (size_t i = 0; i < column_length; i++) {
    counts.push_back(count_distribution(COUNT_GENERATOR));
  }

  return counts;
}

std::vector<Column> generate_columns(std::uniform_int_distribution<int> transition_cost_distribution,
                                     std::uniform_int_distribution<int> emission_cost_distribution,
                                     std::vector<size_t> column_counts) {
  std::vector<Column> columns;

  for (const auto count : column_counts) {
    const auto& column =
        generate_column(count, transition_cost_distribution, emission_cost_distribution,
                        columns.empty() ? Column() : columns.back());
    columns.push_back(column);
  }
  std::reverse(columns.begin(), columns.end());

  return columns;
}

void test_viterbi_search(const std::vector<Column>& columns) {
  SimpleNaiveViterbiSearch na(columns);
  SimpleViterbiSearch vs(columns);

  for (StateId::Time time = 0; time < columns.size(); time++) {
    const auto& na_winner = na.SearchWinner(time);
    const auto& vs_winner = vs.SearchWinner(time);

    if (na_winner.IsValid()) {
      EXPECT_TRUE(vs_winner.IsValid()) << "both winners should be valid";

      EXPECT_EQ(na_winner.time(), time) << "time should be matched";

      EXPECT_EQ(vs_winner.time(), time) << "time should be matched";

      if (na.AccumulatedCost(na_winner) != vs.AccumulatedCost(vs_winner)) {
        std::cout << "GRAPH" << std::endl;
        print_trellis_diagram_vertically(columns);

        std::cout << "PATH OF NA" << std::endl;
        print_path_reversely(columns, na.SearchPathVS(time), na.PathEnd());

        std::cout << "PATH OF VS" << std::endl;
        print_path_reversely(columns, vs.SearchPathVS(time), vs.PathEnd());
      }
      EXPECT_EQ(na.AccumulatedCost(na_winner), vs.AccumulatedCost(vs_winner))
          << "costs should be both optimal";
    } else {
      EXPECT_FALSE(vs_winner.IsValid()) << "both winners should not be found";
    }
  }
}

TEST(ViterbiSearch, TestViterbiSearch) {
  // small case for visualization
  {
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(-10, 30),
        // emission costs
        std::uniform_int_distribution<int>(-10, 30),
        generate_column_counts(5,
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
        generate_column_counts(1000,
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
        generate_column_counts(1000,
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
        generate_column_counts(1000,
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
        generate_column_counts(1000,
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
        generate_column_counts(1000,
                               // column sizes
                               std::uniform_int_distribution<size_t>(1, 100)));
    test_viterbi_search(columns);
  }
}

inline const State& get_state(const std::vector<Column>& columns, const StateId& stateid) {
  return columns[stateid.time()][stateid.id()];
}

class EmissionCostModel {
public:
  EmissionCostModel(const std::vector<Column>& columns) : columns_(columns) {
  }

  float operator()(const StateId& stateid) const {
    return get_state(columns_, stateid).emission_cost;
  }

private:
  std::vector<Column> columns_;
};

class TransitionCostModel {
public:
  TransitionCostModel(const std::vector<Column>& columns) : columns_(columns) {
  }

  float operator()(const StateId& lhs, const StateId& rhs) const {
    const auto& left = get_state(columns_, lhs);
    const auto it = left.transition_costs.find(rhs.id());
    if (it == left.transition_costs.end()) {
      return -1.0;
    } else {
      return it->second;
    }
  }

private:
  std::vector<Column> columns_;
};

struct PathWithCost : std::pair<std::vector<StateId>, float> {
  using std::pair<std::vector<StateId>, float>::pair;
  const std::vector<StateId>& path() const {
    return first;
  }
  float cost() const {
    return second;
  }
};

std::vector<PathWithCost> sort_all_paths(const std::vector<Column>& columns,
                                         const StateId::Time& since_time = 0) {
  if (columns.size() <= since_time) {
    return {PathWithCost({}, 0.0)};
  }

  const auto& sub_pcs = sort_all_paths(columns, since_time + 1);
  std::vector<PathWithCost> pcs;
  for (size_t id = 0; id < columns[since_time].size(); id++) {
    const StateId stateid(since_time, id);
    const auto& state = get_state(columns, stateid);
    for (const auto& sub_pc : sub_pcs) {
      if (sub_pc.path().empty()) {
        pcs.emplace_back(std::vector<StateId>{stateid}, state.emission_cost);
      } else {
        const auto it = state.transition_costs.find(sub_pc.path().front().id());
        if (it != state.transition_costs.end()) {
          const float cost = state.emission_cost + it->second + sub_pc.cost();
          std::vector<StateId> path;
          path.reserve(1 + sub_pc.path().size());
          path.push_back(stateid);
          std::copy(sub_pc.path().begin(), sub_pc.path().end(), std::back_inserter(path));
          pcs.emplace_back(path, cost);
        }
      }
    }
  }

  std::sort(pcs.begin(), pcs.end(),
            [](const PathWithCost& lhs, const PathWithCost& rhs) { return lhs.cost() < rhs.cost(); });

  return pcs;
}

void validate_path(const std::vector<Column>& columns, const std::vector<StateId>& path) {
  EXPECT_EQ(columns.size(), path.size()) << "path size and columns size must be same";

  for (StateId::Time time = 0; time < path.size(); time++) {
    EXPECT_TRUE(path[time].IsValid())
        << "stateid in path must be valid at time " + std::to_string(time);

    EXPECT_EQ(time, path[time].time())
        << "path[time].time() != time " + std::to_string(path[time].time());

    EXPECT_LT(path[time].id(), columns[time].size());

    if (0 < time) {
      const auto& prev_state = get_state(columns, path[time - 1]);
      const auto it = prev_state.transition_costs.find(path[time].id());
      ASSERT_NE(it, prev_state.transition_costs.end()) << "must be connected";
    }
  }
}

float total_cost(const std::vector<Column>& columns, const std::vector<StateId>& path) {
  float cost = 0;
  for (StateId::Time time = 0; time < path.size(); time++) {
    if (0 < time) {
      const auto& prev_state = get_state(columns, path[time - 1]);
      const auto it = prev_state.transition_costs.find(path[time].id());
      if (it != prev_state.transition_costs.end()) {
        cost += it->second;
      }
    }
    cost += get_state(columns, path[time]).emission_cost;
  }
  return cost;
}

void test_viterbisearch_brute_force(const std::vector<Column>& columns, IViterbiSearch& vs) {
  const auto& pcs = sort_all_paths(columns);
  if (columns.empty()) {
    EXPECT_EQ(pcs, std::vector<PathWithCost>{PathWithCost({}, 0.0)})
        << "expect empty set from empty columns";
    return;
  }

  TopKSearch ts(vs);
  AddColumns(vs, columns);
  const StateId::Time time = columns.size() - 1;

  for (const auto& pc : pcs) {
    const auto c = total_cost(columns, pc.path());
    validate_path(columns, pc.path());

    std::vector<StateId> vs_path, original_state_ids;
    std::copy(vs.SearchPathVS(time), vs.PathEnd(), std::back_inserter(vs_path));
    for (auto s_itr = vs_path.rbegin(); s_itr != vs_path.rend(); ++s_itr)
      original_state_ids.push_back(ts.GetOrigin(*s_itr, *s_itr));
    validate_path(columns, original_state_ids);

    // std::cout << "top total cost     : " << c << std::endl;
    // std::cout << "top brute force    : " << pc.cost() << std::endl;
    // std::cout << "top viterbi search : " << total_cost(columns, vs_path) << std::endl;
    // std::cout << std::endl;

    EXPECT_EQ(c, pc.cost()) << "total cost by brute force mush be correct";
    EXPECT_EQ(c, total_cost(columns, original_state_ids)) << "Wrong total cost by viterbisearch";

    ts.RemovePath(vs_path);
    vs.ClearSearch();
  }
}

TEST(ViterbiSearch, TestTopKSearch) {

  {
    ViterbiSearch vs;
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(1, 10),
        // emission costs
        std::uniform_int_distribution<int>(1, 10),
        generate_column_counts(3,
                               // column sizes
                               std::uniform_int_distribution<size_t>(4, 5)));

    vs.set_emission_cost_model(EmissionCostModel(columns));
    vs.set_transition_cost_model(TransitionCostModel(columns));

    test_viterbisearch_brute_force(columns, vs);
  }

  {
    ViterbiSearch vs;
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(1, 10),
        // emission costs
        std::uniform_int_distribution<int>(1, 10),
        generate_column_counts(1,
                               // column sizes
                               std::uniform_int_distribution<size_t>(1, 10)));

    vs.set_emission_cost_model(EmissionCostModel(columns));
    vs.set_transition_cost_model(TransitionCostModel(columns));

    test_viterbisearch_brute_force(columns, vs);
  }

  {
    ViterbiSearch vs;
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(1, 10),
        // emission costs
        std::uniform_int_distribution<int>(1, 10),
        generate_column_counts(1,
                               // column sizes
                               std::uniform_int_distribution<size_t>(0, 0)));

    vs.set_emission_cost_model(EmissionCostModel(columns));
    vs.set_transition_cost_model(TransitionCostModel(columns));

    test_viterbisearch_brute_force(columns, vs);
  }

  {
    ViterbiSearch vs;
    const auto& columns = generate_columns(
        // transition costs
        std::uniform_int_distribution<int>(1, 10),
        // emission costs
        std::uniform_int_distribution<int>(1, 10),
        generate_column_counts(0,
                               // column sizes
                               std::uniform_int_distribution<size_t>(10, 10)));

    vs.set_emission_cost_model(EmissionCostModel(columns));
    vs.set_transition_cost_model(TransitionCostModel(columns));

    test_viterbisearch_brute_force(columns, vs);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
