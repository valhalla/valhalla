#include "skadi/util.h"
#include <algorithm>

namespace {

template <class T> T clamp(T val, const T low, const T high) {
  return std::min<T>(std::max<T>(val, low), high);
}

constexpr double NO_DATA_VALUE = -32768;
} // namespace

namespace valhalla {
namespace skadi {

double energy_weighting(double& grade) {
  // dont consider anything steeper than -10% or +15%
  grade = clamp(grade, -10.0, 15.0);
  // ascent gets weighted more than descent
  return 1.0 + grade / (grade > 0 ? 7.0 : 17.0);
}

std::tuple<double, double, double, double>
weighted_grade(const std::vector<double>& heights,
               const double interval_distance,
               const std::function<double(double&)>& grade_weighting) {
  // grab the scaled grade for each section
  double grade = 0.0;
  double total_grade = 0;
  double total_weight = 0;
  double max_up_grade = 0.0;
  double max_down_grade = 0.0;

  // Accumulate elevation - to compute mean_elevation
  uint32_t n = 0;
  double total_elev = 0.0;
  if (heights.front() != NO_DATA_VALUE) {
    total_elev += heights.front();
    n++;
  }

  // multiply grades by 100 to move from 0-1 into 0-100 for grade percentage
  auto scale = 100.0 / interval_distance;
  for (auto h = heights.cbegin() + 1; h != heights.cend(); ++h) {
    // get the grade for this section. Ignore any invalid elevation postings
    if (*h == NO_DATA_VALUE || *std::prev(h) == NO_DATA_VALUE) {
      grade = 0.0;
    } else {
      grade = (*h - *std::prev(h)) * scale;
    }

    // Add the elevation so we can compute mean (assumes uniform
    // sampling along the path)
    if (*h != NO_DATA_VALUE) {
      total_elev += *h;
      n++;
    }

    // Update max grades. TODO - do we need to filter or smooth these?
    max_up_grade = std::max(grade, max_up_grade);
    max_down_grade = std::min(grade, max_down_grade);

    // find the weight and possibly change the grade
    auto weight = grade_weighting(grade);
    // keep this part of weighted grade
    total_grade += grade * weight;
    // keep this part of weight
    total_weight += weight;
  }

  // Get the average weighted grade by homogenizing total weight. Return
  // max grades (up and down) and mean elevation.
  return std::make_tuple(total_grade * (1.0 / total_weight), max_up_grade, max_down_grade,
                         (total_elev / n));
}

} // namespace skadi
} // namespace valhalla
