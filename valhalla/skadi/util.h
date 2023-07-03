#ifndef __VALHALLA_UTIL_H__
#define __VALHALLA_UTIL_H__

#include <cstdint>
#include <functional>
#include <vector>

namespace valhalla {
namespace skadi {

/* Clamps the grade to between -10 and +15. Grades greater than 0 are weighted
 * using 1 + g / 7. This gives (grade,weight) of (0,1), (6,1.9), (15,3.2).
 * Grades less than 0 are weighted using 1 + g / 17, which gives (grade,weight)
 * of (0,1), (-6,.6), (-10,.4).
 */
double energy_weighting(double& grade);

/*
 * Returns the normalized grade using the weighting method provided. Each sections
 * grade is computed. The provided weighting function is applied to each grade. The
 * weightings are then normalized. The result is approximate most important grade
 * for the given stretch of heights. Also returns the maximum upward grade and the
 * maximum downward grade.
 *
 * @param    heights            the height reading at each sampled location
 * @param    interval_distance  the distance between each sampled location
 * @param    grade_weighting    the function which provides the weight that should be applied to a
 * specific grade the grade is pass by reference so you may clamp it to a range if you like
 * @return   the approximate grade on a scale from -100 to +100,
 *           maximum upward slope (or 0 if all downward),
 *           maximum downward slope (or 0 if all upward)
 *           mean elevation in meters
 */
std::tuple<double, double, double, double>
weighted_grade(const std::vector<double>& heights,
               const double interval_distance,
               const std::function<double(double&)>& grade_weighting = energy_weighting);

} // namespace skadi
} // namespace valhalla

#endif //__VALHALLA_UTIL_H__
