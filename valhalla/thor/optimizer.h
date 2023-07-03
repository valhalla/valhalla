#ifndef VALHALLA_THOR_OPTIMIZER_H_
#define VALHALLA_THOR_OPTIMIZER_H_

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace valhalla {
namespace thor {

// Annealing factor - the rate at which temperature is reduced between each
// annealing step. Recommend setting this to a value between 0.85 and 0.95.
// Cool too slowly and the probability of escaping local optima decreases and
// setting too high takes longer to converge on a solution.
constexpr float kCoolingRate = 0.93f;

// Alteration type.
// kRotate  - Alters a portion of the tour by rotating the locations about
//            a middle point. The middle location becomes the new start
//            location in the range.
// kReverse - Alters the tour by reversing the locations in the tour between
//            a start and end location.
enum AlterationType { kRotate, kReverse };

// Simple structure with 3 values describing a possible tour alteration
struct TourAlteration {
  uint32_t start;     // Index of 1st location
  uint32_t mid;       // Index of middle location (for rotate)
  uint32_t end;       // Index of 2nd location
  AlterationType alt; // Type of alteration
};

/**
 * Optimization method using simulated annealing. Optimizes the order of
 * locations - keeping the first location (origin) and last location
 * (destination) fixed.
 */
class Optimizer {
public:
  /**
   * Optimize the tour through a set of locations given the cost matrix
   * among all locations. The first location (origin) and last location
   * (destination) remain fixed in the tour.
   * @param  count  Number of locations.
   * @param  costs  2-D cost matrix.
   * @return Returns the tour as an updated order of locations visited to
   *         complete the tour.
   */
  std::vector<uint32_t> Solve(const uint32_t count, const std::vector<float>& costs);

  /**
   * Seed the random number generator. This is used by tests to create a
   * repeatable sequence.
   * @param  seed  Seed to use for the random number generator.
   */
  void Seed(const uint32_t seed) {
    random_generator_.seed(seed);
  }

protected:
  // Random number generation: 0 <= r < 1
  std::mt19937_64 random_generator_;
  std::uniform_real_distribution<float> uniform_distribution_{0.0, 1.0};

  uint32_t ntry_;                   // # of attempts (for debugging)
  uint32_t count_;                  // # of locations
  uint32_t attempts_;               // # of attempts per annealing cycle
  uint32_t successes_;              // # of success per annealing cycle
  float best_cost_;                 // Current best cost
  std::vector<uint32_t> tour_;      // Current tour (order of locations)
  std::vector<uint32_t> best_tour_; // Best tour so far

  /*
   * Perform the annealing process.
   * @param  costs        2-D cost matrix.
   * @param  temperature  Current temperature.
   * @return Returns number of successes.
   */
  uint32_t Anneal(const std::vector<float>& costs, float temperature);

  /**
   * Select a potential alteration of the tour.
   * @return  Returns a candidate tour alteration: type and location indexes.
   */
  TourAlteration GetTourAlteration();

  /**
   * Get the temperature difference (based on change in tour cost) given
   * an alteration to the tour.
   * @param  costs       2-D cost matrix.
   * @param  alteration  Tour alteration.
   * @return Returns the cost difference.
   */
  float TemperatureDifference(const std::vector<float>& costs, const TourAlteration& alteration);

  /**
   * Create a random initial tour. The first and last locations must remain
   * fixed as the tour begin and end locations do not change.
   */
  void CreateRandomTour();

  /**
   * Get the cost for the specified tour (order of locations).
   * @param  costs  2-D cost array between locations.
   * @param  tour   Order that locations are traversed.
   * @return Returns the total cost for the tour.
   */
  float TourCost(const std::vector<float>& costs, const std::vector<uint32_t>& tour) const;

  // ------------------------ Convenience methods (inline) ---------------- //

  /**
   * Get the cost between two locations from a 2-D cost array.
   * @param  costs  Cost array.
   * @param  loc1   Location index 1.
   * @param  loc2   Location index 1.
   * @return Returns the cost between the 2 locations.
   */
  float Cost(const std::vector<float>& costs, const uint32_t loc1, const uint32_t loc2) const {
    return costs[(loc1 * count_) + loc2];
  }

  /**
   * Get a random location. Makes sure it isn't the first or last location
   * (which are fixed).
   * @return  Returns the index of a random location.
   */
  uint32_t get_random_location() {
    return static_cast<uint32_t>(r01() * (count_ - 2) + 1);
  }

  /**
   * Convenience method to return a random floating point value: 0 <= r < 1
   * @return  Returns a random value between 0 and 1.
   */
  float r01() {
    return uniform_distribution_(random_generator_);
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_OPTIMIZER_H_
