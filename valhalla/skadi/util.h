#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <utility>
#include <vector>

namespace valhalla {
  namespace skadi {

    /*
     * Returns the positive descent and positive ascent as a ratio of the max grade
     *
     * @param    heights  the height reading at each sampled location
     * @param    interval_distance  the distance between each sampled location
     * @param    maximum_grade  the maximum grade. if a computed grade is larger than
     *           this value its ratio will be returned as 1
     * @return   .first  = the descent over distance clamped to max grade and given as a ratio of same
     *           .second = the ascent over distance clamped to max grade and given as a ratio of same
     */
    std::pair<double, double> discretized_deltas(const std::vector<double>& heights, const double interval_distance, const double maximum_grade);

  }
}


#endif //__VALHALLA_SAMPLE_H__
