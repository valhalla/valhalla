#include <iostream>
#include <chrono>
#include <random>

#include "viterbi_search.h"


using ObjectId = uint32_t;

class Candidate
{
 public:
  Candidate(ObjectId id)
      : id_(id), emission_cost_(-1.f) {
  }

  Candidate(ObjectId id, float emission_cost)
      : id_(id), emission_cost_(emission_cost) {
  }

  ObjectId id() const {
    return id_;
  }

  float emission_cost() const {
    return emission_cost_;
  }

  float transition_cost(ObjectId id) const {
    auto iter = transition_cost_.find(id);
    if (iter == transition_cost_.end()) {
      return -1.f;
    }
    return iter->second;
  }

  void set_transition_cost(ObjectId id, float cost) {
    transition_cost_[id] = cost;
  }

 protected:
  std::unordered_map<ObjectId, float> transition_cost_;

 private:
  ObjectId id_;
  float emission_cost_;
};


bool operator==(const Candidate& lhs, const Candidate& rhs)
{
  // TODO test internal members also? how?
  return lhs.id() == rhs.id() && lhs.emission_cost() == rhs.emission_cost();
}


class SimpleViterbiSearch: public ViterbiSearch<Candidate>
{
 protected:
  float TransitionCost(const CandidateWrapper<Candidate>& left,
                       const CandidateWrapper<Candidate>& right) const {
    assert(left.time() + 1 == right.time());
    auto right_id = right.candidate().id();
    return left.candidate().transition_cost(right_id);
  }

  float EmissionCost(const CandidateWrapper<Candidate>& candidate) const {
    return candidate.candidate().emission_cost();
  }

  double CostSofar(double prev_cost_sofar,
                   float transition_cost,
                   float emission_cost) const {
    return prev_cost_sofar + transition_cost + emission_cost;
  }
};


class SimpleNaiveViterbiSearch: public NaiveViterbiSearch<Candidate, false>
{
 protected:
  float TransitionCost(const CandidateWrapper<Candidate>& left,
                       const CandidateWrapper<Candidate>& right) const
  {
    assert(left.time() + 1 == right.time());
    auto right_id = right.candidate().id();
    auto cost = left.candidate().transition_cost(right_id);
    return cost < 0.f? kInvalidCost : cost;
  }

  float EmissionCost(const CandidateWrapper<Candidate>& candidate) const
  {
    auto cost = candidate.candidate().emission_cost();
    return cost < 0.f? kInvalidCost : cost;
  }

  double CostSofar(double prev_cost_sofar,
                   float transition_cost,
                   float emission_cost) const
  {
    assert(prev_cost_sofar >= 0.f
           && transition_cost >= 0.f
           && emission_cost >= 0.f);
    return prev_cost_sofar + transition_cost + emission_cost;
  }
};


unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine transition_cost_generator(seed),
  emission_cost_generator(seed);


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
    auto emission_cost = static_cast<float>(emission_cost_distribution(emission_cost_generator));
    candidates.emplace_back((*start_id)++, emission_cost);
    for (const auto& candidate : next_candidates) {
      auto transition_cost = static_cast<float>(transition_cost_distribution(transition_cost_generator));
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


void print_path(const std::vector<const CandidateWrapper<Candidate>*>& path)
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
                         size_t num_states,
                         size_t num_candidates)
{
  // Generate candidates for testing
  ObjectId start_id = 0;
  std::vector<Candidate> prev_candidates;
  std::vector<std::vector<Candidate>> states;
  for (size_t i = 0; i < num_states; i++) {
    auto candidates = generate_candidates(&start_id, num_candidates,
                                          transition_cost_distribution,
                                          emission_cost_distribution,
                                          prev_candidates);
    prev_candidates = candidates;
    states.push_back(candidates);
  }
  std::reverse(states.begin(), states.end());

  // print_trellis_diagram_vertically(states);

  // Test viterbi search
  SimpleNaiveViterbiSearch snvs;
  SimpleViterbiSearch svs;
  for (const auto& state : states) {
    auto svs_time = svs.AppendState(state.cbegin(), state.cend()),
        snvs_time = snvs.AppendState(state.cbegin(), state.cend());
    assert(svs_time == snvs_time);

    auto snvs_winner = snvs.SearchWinner(snvs_time);
    auto svs_winner = svs.SearchWinner(svs_time);

    if (svs_winner) {
      assert(svs_winner->time() == svs_time &&
             snvs_winner->time() == snvs_time);
      assert(svs.costsofar(*svs_winner) == snvs.costsofar(*snvs_winner));
      if (svs_winner->candidate() == snvs_winner->candidate()) {
      } else {
        // It happens often that multiple winners (with the same
        // costsofar) are found at the same time
        std::cout << "different winners with same costsofar at time " << svs_time << std::endl;
      }
    } else {
      assert(!snvs_winner);
    }

    auto svs_path = svs.SearchPath(svs_time);
    auto snvs_path = snvs.SearchPath(snvs_time);

    assert(svs_path.size() == snvs_path.size());
    assert(svs_path.size() == svs_time + 1);
    // print_path(svs_path);
    // print_path(snvs_path);

    // TODO verify if their costs
  }
}


void TestViterbiSearch()
{
  std::cout << "Iteration 1" << std::endl;
  std::uniform_int_distribution<int> transition_cost_distribution(0, 50);
  std::uniform_int_distribution<int> emission_cost_distribution(0, 100);
  size_t num_states = 1000, num_candidates = 20;
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution, num_states, num_candidates);

  std::cout << "Iteration 2" << std::endl;
  transition_cost_distribution = std::uniform_int_distribution<int>(-50, 10);
  emission_cost_distribution = std::uniform_int_distribution<int>(-100, 10);
  num_states = 1000, num_candidates = 20;
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution, num_states, num_candidates);

  std::cout << "Iteration 3" << std::endl;
  transition_cost_distribution = std::uniform_int_distribution<int>(-30, -3);
  emission_cost_distribution = std::uniform_int_distribution<int>(3, 30);
  num_states = 1000, num_candidates = 20;
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution, num_states, num_candidates);

  std::cout << "Iteration 4" << std::endl;
  transition_cost_distribution = std::uniform_int_distribution<int>(3, 30);
  emission_cost_distribution = std::uniform_int_distribution<int>(-30, -3);
  num_states = 1000, num_candidates = 20;
  test_viterbi_search(transition_cost_distribution, emission_cost_distribution, num_states, num_candidates);
}


void TestCandidatePair()
{
  CandidatePairId pair;
  pair = candidateid_make_pair(0, 1);
  assert(pair == 1);
  assert(candidateid_right(pair) == 0);
  assert(candidateid_left(pair) == 1);

  pair = candidateid_make_pair(1, 0);
  assert(pair == static_cast<CandidatePairId>(1) << 32);
  assert(candidateid_right(pair) == 1);
  assert(candidateid_left(pair) == 0);

  pair = candidateid_make_pair(12, 30);
  assert(candidateid_right(pair) == 12);
  assert(candidateid_left(pair) == 30);

  pair = ~0;
  assert(candidateid_right(pair) == ~0);
  assert(candidateid_left(pair) == ~0);
}


int main(int argc, char *argv[])
{
  TestViterbiSearch();
  TestCandidatePair();
  return 0;
}
