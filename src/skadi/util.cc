#include "skadi/util.h"

namespace {

  template <class T>
  T clamp(T val, T low, T high) {
    return std::min<T>(std::max<T>(val, low), high);
  }

}

namespace valhalla {
  namespace skadi {
    std::pair<double, double> discretized_deltas(const std::vector<double>& heights, const double interval_distance, const double maximum_grade){
      //nothing to do here
      std::pair<double, double> deltas{0, 0};
      if(heights.size() < 2)
        return deltas;

      double total_distance = 0;
      for(auto h = heights.cbegin() + 1; h != heights.cend(); ++h) {
        auto diff = static_cast<float>(*h - *std::prev(h));
        if(diff > 0)
          deltas.second += diff;
        else
          deltas.first -= diff;
        total_distance += interval_distance;
      }

      //clamp from 0 to max and get ratio of max
      deltas.first = clamp(deltas.first / total_distance, 0.0, maximum_grade) / maximum_grade;
      deltas.second = clamp(deltas.second / total_distance, 0.0, maximum_grade) / maximum_grade;
      return deltas;
    }
  }
}
