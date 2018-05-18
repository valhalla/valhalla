#include "thor/optimizer.h"
#include "midgard/logging.h"

namespace valhalla {
namespace thor {

// Optimize the tour through a set of locations given the cost matrix
// among all locations. The first location (origin) and last location
// (destination) remain fixed in the tour.
std::vector<uint32_t> Optimizer::Solve(const uint32_t count, const std::vector<float>& costs) {
  // Handle trivial cases.
  count_ = count;
  if (count == 2) {
    return {0, 1};
  } else if (count_ == 3) {
    return {0, 1, 2};
  } else if (count == 4) {
    // Only one possible way to alter the path.
    std::vector<uint32_t> tour1 = {0, 1, 2, 3};
    std::vector<uint32_t> tour2 = {0, 2, 1, 3};
    return (TourCost(costs, tour1) < TourCost(costs, tour2)) ? tour1 : tour2;
  }

  // Populate the initial tour with a random order. The first and last
  // locations must remain fixed as the tour begin and end locations do not
  // change.
  CreateRandomTour();

  // Copy current tour to best tour and get the tour cost. Set the initial
  // temperature based on tour cost
  best_tour_ = tour_;
  best_cost_ = TourCost(costs, tour_);
  float temperature = best_cost_ / count_;

  // Perform simulated annealing. Set a run limit per annealing step and a
  // success limit to break out early if enough successes are found.
  ntry_ = 0;
  attempts_ = 400 * count_;
  successes_ = 40 * count_;
  for (uint32_t i = 0; i < 100; i++) {
    // Break if no successes were found during this annealing step.
    if (Anneal(costs, temperature) == 0) {
      break;
    }

    // Reduce temperature
    temperature *= kCoolingRate;
  }

  // Return the best tour
  LOG_DEBUG("Best tour cost = " + std::to_string(best_cost_) + " ntries = " + std::to_string(ntry_));
  return best_tour_;
}

// Perform the annealing process.
uint32_t Optimizer::Anneal(const std::vector<float>& costs, float temperature) {
  uint32_t success_count = 0;
  for (uint32_t i = 0; i < attempts_; i++) {
    // Select a potential tour alteration. Get temperature difference.
    TourAlteration alteration = GetTourAlteration();
    float diff = TemperatureDifference(costs, alteration);
    ntry_++;

    // Check if we should keep this as the new best path. The idea behind
    // simulated annealing is that it tries to avoid becoming trapped in local
    // optima by occasionally allowing worse solutions at a probability that
    // declines with time.
    // (http://www.technical-recipes.com/2012/c-implementation-of-hill-climbing-and-simulated-annealing-applied-to-travelling-salesman-problems/
    if (diff < 0.0f || r01() < std::exp(-diff / temperature)) {
      if (alteration.alt == kReverse) {
        // Alter the tour by reversing the locations in the tour between a
        // start and end location. Add 1 to the end index since STL algorithm
        // reverse does not include the iterator at position it2.
        auto it1 = tour_.begin() + alteration.start;
        auto it2 = tour_.begin() + alteration.end + 1;
        std::reverse(it1, it2);
      } else {
        // Alter the tour by rotating the locations about a middle point.
        // The middle location becomes the new first location. Add 1 to
        // the end index so that is included in the rotated set.
        auto it1 = tour_.begin() + alteration.start;
        auto it2 = tour_.begin() + alteration.mid;
        auto it3 = tour_.begin() + alteration.end + 1;
        std::rotate(it1, it2, it3);
      }
      success_count++;

      // Get the tour cost and update the best tour if less cost
      float cost = TourCost(costs, tour_);
      if (cost < best_cost_) {
        best_cost_ = cost;
        best_tour_ = tour_;
      }
    }
    if (success_count >= successes_) {
      break;
    }
  }
  return success_count;
}

// Create a random initial tour. The first and last locations must remain
// fixed as the tour begin and end locations do not change.
void Optimizer::CreateRandomTour() {
  tour_.clear();
  for (uint32_t i = 1; i < count_ - 1; i++) {
    tour_.push_back(i);
  }
  std::random_shuffle(tour_.begin(), tour_.end());
  tour_.insert(tour_.begin(), 0);
  tour_.push_back(count_ - 1);
}

// Select a potential alteration of the tour.
TourAlteration Optimizer::GetTourAlteration() {
  // Select three unique locations between 1 and count-2
  std::vector<uint32_t> loc(3);
  while (true) {
    loc[0] = get_random_location();
    loc[1] = get_random_location();
    loc[2] = get_random_location();
    if (loc[0] != loc[1] && loc[0] != loc[2] && loc[1] != loc[2]) {
      break;
    }
  }

  // Sort location indexes so they have increasing values to use as
  // a start location index, mid location index (for rotation), and an
  // end location index.
  std::sort(loc.begin(), loc.end());

  // Randomly select the alteration type and return the
  // tour alteration (indexes and type).
  AlterationType t = (r01() < 0.5f) ? kReverse : kRotate;
  return {loc[0], loc[1], loc[2], t};
}

float Optimizer::TemperatureDifference(const std::vector<float>& costs,
                                       const TourAlteration& alteration) {
  float c = 0;
  uint32_t start = alteration.start;
  uint32_t end = alteration.end;
  if (alteration.alt == kRotate) {
    // Subtract costs of connections that are broken
    uint32_t mid = alteration.mid;
    c -= Cost(costs, tour_[start - 1], tour_[start]);
    c -= Cost(costs, tour_[end], tour_[end + 1]);
    c -= Cost(costs, tour_[mid - 1], tour_[mid]);

    // Add costs for the new connections
    c += Cost(costs, tour_[start - 1], tour_[mid]);
    c += Cost(costs, tour_[end], tour_[start]);
    c += Cost(costs, tour_[mid - 1], tour_[end + 1]);
  } else {
    // Reverse tour locations between a start and an end index.
    // Subtract costs between successive locations from start-1 to end+1
    for (uint32_t i = start - 1, j = i + 1; i <= end; i++, j++) {
      c -= Cost(costs, tour_[i], tour_[j]);
    }

    // Add costs for the new connections and the reversed order
    c += Cost(costs, tour_[start - 1], tour_[end]);
    c += Cost(costs, tour_[start], tour_[end + 1]);
    for (uint32_t i = end, j = i - 1; i > start; i--, j--) {
      c += Cost(costs, tour_[i], tour_[j]);
    }
  }
  return (c / static_cast<float>(count_));
}

// Get the cost for the specified tour (order of locations).
float Optimizer::TourCost(const std::vector<float>& costs, const std::vector<uint32_t>& tour) const {
  float c = 0;
  for (uint32_t i = 0; i < count_ - 1; i++) {
    c += costs[(tour[i] * count_) + tour[i + 1]];
  }
  return c;
}

} // namespace thor
} // namespace valhalla
