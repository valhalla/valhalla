#ifndef VALHALLA_MIDGARD_GRIDDEDDATA_H_
#define VALHALLA_MIDGARD_GRIDDEDDATA_H_

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace midgard {

// A special generalization value indicating that the application should
// compute an optimal generalization factor when creating contours.
constexpr float kOptimalGeneralization = std::numeric_limits<float>::max();

/**
 * Class to store data in a gridded/tiled data structure. Contains methods
 * to mark each tile with data using a compare operator.
 *
 * Currently its templated so we can store time and distance but really we should
 * move distance out of the edgelabel and into costing and then only support
 * sif::Cost as the main thing that we track here. In the future we could track more
 * things there like fuel usage or battery usage
 */
template <std::size_t dimensions_t> class GriddedData : public Tiles<PointLL> {
public:
  using value_type = std::array<float, dimensions_t>;

  /**
   * Constructor.
   * @param   bounds    Bounding box
   * @param   tilesize  Tile size
   * @param   value     Value to initialize data with.
   */
  GriddedData(const AABB2<PointLL>& bounds, const float tilesize, const value_type& value)
      : Tiles<PointLL>(bounds, tilesize), max_value_(value),
        data_(this->nrows_ * this->ncolumns_, value) {
  }

  /**
   * Set the value at a specified tile Id if the value is less than the current
   * value set at the grid location. Verifies that the tile is valid.
   * @param  tile_id  Tile Id to set value for.
   * @param  value    Value to set at the tile/grid location.
   * @param  get_data Functor to get the desired data value
   * @param  set_data Functor to set the desired data value
   */
  inline void SetIfLessThan(const int tile_id, const value_type& value) {
    if (tile_id >= 0 && tile_id < data_.size()) {
      auto& current_value = data_[tile_id];
      for (size_t i = 0; i < dimensions_t; ++i) {
        current_value[i] = std::min(value[i], current_value[i]);
      }
    }
  }

  float DataAt(size_t tileid, size_t metricid) const {
    return data_[tileid][metricid];
  }

  float MaxValue(size_t metricidx) const {
    return max_value_[metricidx];
  }

  using contour_t = std::list<PointLL>;
  using feature_t = std::list<contour_t>;
  using contours_t = std::vector<std::list<feature_t>>;
  // dimension, value (seconds/meters), name (time/distance), color
  using contour_interval_t = std::tuple<size_t, float, std::string, std::string>;
  /**
   * TODO: implement two versions of this, leave this one for linestring contours
   * and make another for polygons
   *
   * Generate contour lines from the gridded data.
   * contours is an ordered list of contour interval values
   * Derivation from the C code version of CONREC by Paul Bourke: http://paulbourke.net/papers/conrec/
   *
   * @param contour_intervals    the values at which the contour lines should occur
   *                             basically the lines on the measuring stick.
   * @param rings_only           only include geometry of contours that are polygonal
   * @param denoise              remove any contours whose size ratio is less than
   *                             this parameter with respect to the largest contour
   *                             with the same interval. by default only keep the largest
   * @param generalize           Generalization factor in meters. A special value
   *                             kOptimalGeneralization will let the method choose
   *                             an optimal generalization factor based on grid size.
   *
   * @return contour line geometries with the larger intervals first (for rendering purposes)
   */
  contours_t GenerateContours(std::vector<contour_interval_t>& intervals,
                              const bool rings_only = false,
                              const float denoise = 1.f,
                              const float generalize = 200.f) const {
    // sort the contours first on the metric index then on the values with the bigger contours first
    std::sort(intervals.begin(), intervals.end(), std::greater<>());

    // Values at tile corners and center (0 element is center)
    int sh[5];
    typename PointLL::first_type s[5]; // Values at the tile corners and center
    PointLL tile_corners[5];           // PointLL at tile corners and center
    int m1, m2, m3;                    // Indices into the tile corners
    PointLL from_pt, to_pt;            // The intersection points in the tile
    int tile_inc[4] = {0, 1, this->ncolumns_ + 1, this->ncolumns_};

    // Find the intersection along a tile edge
    auto intersect = [&tile_corners, &s](int p1, int p2) {
      auto ds = s[p2] - s[p1];
      auto x = (s[p2] * tile_corners[p1].first - s[p1] * tile_corners[p2].first) / ds;
      auto y = (s[p2] * tile_corners[p1].second - s[p1] * tile_corners[p2].second) / ds;
      // we round here because connecting the cell line segments requires finding points via equality
      // on some platforms the intersection arithmetic for adjacent cells results in floating point
      // noise that differs for the intersection point on either side of the cell boundary, snapping
      // to centimeter resolution lets us get usable results on those platforms (eg. aarch64)
      return PointLL(std::round(x * 1e7) / 1e7, std::round(y * 1e7) / 1e7);
    };

    // In the tight loop below, we need to decide where a contour intersects the triangles that make
    // up the given tile. this works out to a number of discrete cases which we lookup using the table
    // below. based on the case we perform the appropriate intersection. to avoid branching we store
    // the intersection operation for each case in an array and perform the correct one by calling the
    // function stored in the array
    int case_table[3][3][3] = {
        {{0, 0, 8}, {0, 2, 5}, {7, 6, 9}},
        {{0, 3, 4}, {1, 0, 1}, {4, 3, 0}},
        {{9, 6, 7}, {5, 2, 0}, {8, 0, 0}},
    };

    // swap_table indicates whether the (from_pt, to_pt) segment orientation should be changed or not
    // Lets take case_index=3: case[1][0][1] and case[1][2][1]. They correspond to the (0, -1, 0) and
    // (0, 1, 0) sh[] values respectively. Though both cases correspond to the case_value=3 but they
    // have different m1--m3 segment orientation.
    /*
     *          sh[m2]=-1        |          sh[m2]=1
     *           /\              |              /\
     *          /  \             |             /  \
     *         /    \            |            /    \
     * sh[m1]=0 ----> sh[m3]=0   |    sh[m1]=0 <---- sh[m3]=0
     */
    // This is needed to keep contours oriented correctly, according to the
    // right-hand rule:
    // "A linear ring MUST follow the right-hand rule with respect to the area it
    // bounds, i.e., exterior rings are counterclockwise, and holes are clockwise."  (c)
    // (c) https://tools.ietf.org/html/rfc7946#section-3.1.6
    bool swap_table[3][3][3] = {
        {{false, false, true}, {false, true, true}, {true, false, false}},
        {{false, true, false}, {true, false, false}, {true, false, false}},
        {{true, true, false}, {false, false, false}, {false, false, false}},
    };
    std::array<std::function<void()>, 10> cases{
        [&]() {},
        // Line between vertices 1 and 2
        [&]() {
          from_pt = tile_corners[m1];
          to_pt = tile_corners[m2];
        },
        // Line between vertices 2 and 3
        [&]() {
          from_pt = tile_corners[m2];
          to_pt = tile_corners[m3];
        },
        // Line between vertices 3 and 1
        [&]() {
          from_pt = tile_corners[m3];
          to_pt = tile_corners[m1];
        },
        // Line between vertex 1 and side 2-3
        [&]() {
          from_pt = tile_corners[m1];
          to_pt = intersect(m2, m3);
        },
        // Line between vertex 2 and side 3-1
        [&]() {
          from_pt = tile_corners[m2];
          to_pt = intersect(m3, m1);
        },
        // Line between vertex 3 and side 1-2
        [&]() {
          from_pt = tile_corners[m3];
          to_pt = intersect(m1, m2);
        },
        // Line between sides 1-2 and 2-3
        [&]() {
          from_pt = intersect(m1, m2);
          to_pt = intersect(m2, m3);
        },
        // Line between sides 2-3 and 3-1
        [&]() {
          from_pt = intersect(m2, m3);
          to_pt = intersect(m3, m1);
        },
        // Line between sides 3-1 and 1-2
        [&]() {
          from_pt = intersect(m3, m1);
          to_pt = intersect(m1, m2);
        },
    };

    // which metrics do we need contours for
    auto _ = std::make_pair(intervals.cbegin(), intervals.cend());
    std::vector<decltype(_)> metrics{std::move(_)};
    for (auto interval = intervals.cbegin(); interval != intervals.cend(); ++interval) {
      if (std::get<0>(*interval) != std::get<0>(*metrics.back().first)) {
        metrics.back().second = interval;
        metrics.emplace_back(interval, intervals.cend());
      }
    }

    // we need something to hold each iso-line
    contours_t contours(intervals.size(), std::list<feature_t>{feature_t{}});

    // and something to find them quickly
    using contour_lookup_t = std::map<PointLL, typename feature_t::iterator>;
    // store begins and ends of the segments separately not to loose segment orientation
    std::vector<contour_lookup_t> begin_lookups(intervals.size());
    std::vector<contour_lookup_t> end_lookups(intervals.size());
    // TODO: preallocate the lookups for each interval

    // For each metric we tracked
    for (const auto& metric : metrics) {
      size_t metric_index = std::get<0>(*metric.first);

      // For each cell, skipping the outer rim since its out of bounds
      for (int row = 1; row < this->nrows_ - 1; ++row) {
        for (int col = 1; col < this->ncolumns_ - 1; ++col) {
          int tileid = this->TileId(col, row);
          auto cell1 = data_[tileid][metric_index];
          auto cell2 = data_[tileid + this->ncolumns_][metric_index];     // TileId(col,   row+1)];
          auto cell3 = data_[tileid + 1][metric_index];                   // TileId(col+1, row)];
          auto cell4 = data_[tileid + this->ncolumns_ + 1][metric_index]; // TileId(col+1, row+1)];
          auto dmin = std::min(std::min(cell1, cell2), std::min(cell3, cell4));
          auto dmax = std::max(std::max(cell1, cell2), std::max(cell3, cell4));

          // Continue if outside the range of contour values for this metric_index
          if (dmax < std::get<1>(*std::prev(metric.second)) || dmin > std::get<1>(*metric.first)) {
            continue;
          }

          // For each requested contour value
          for (size_t i = 0; i < intervals.size(); ++i) {
            // some setup to process this contour
            auto& begin_lookup = begin_lookups[i];
            auto& end_lookup = end_lookups[i];
            auto& contour = contours[i];
            auto contour_value = std::get<1>(intervals[i]);

            // we skip this contour if its interested in a different metric_index or its value
            // would not intersect this cell
            if (std::get<0>(intervals[i]) != metric_index || contour_value < dmin ||
                contour_value > dmax) {
              continue;
            }

            for (int m = 4; m > 0; m--) {
              int newtileid = tileid + tile_inc[m - 1];
              // Make sure the tile corner value is not set to the max_value
              // (messes up the intersect method). Set a value slightly above
              // the contour (e.g. 1 minute higher).
              // TODO - the value 1 is a bit of a hack.
              float nd = data_[newtileid][metric_index];
              s[m] = nd < max_value_[metric_index] ? nd - contour_value : 1.0f;
              tile_corners[m] = this->Base(newtileid);
              sh[m] = (s[m] > 0.0f) - (s[m] < 0.0f); // pos = 1, neg = -1, 0 = 0
            }
            s[0] = 0.25 * (s[1] + s[2] + s[3] + s[4]);
            tile_corners[0] = this->Center(tileid);
            sh[0] = (s[0] > 0.0f) - (s[0] < 0.0f); // pos = 1, neg = -1, 0 = 0

            /*
             Note: at this stage the relative heights of the corners and the
             centre are in the h array, and the corresponding coordinates are
             in the xh and yh arrays. The centre of the box is indexed by 0
             and the 4 corners by 1 to 4 as shown below.
             Each triangle is then indexed by the parameter m, and the 3
             vertices of each triangle are indexed by parameters m1,m2,and m3.
             It is assumed that the centre of the box is always vertex 2
             though this is important only when all 3 vertices lie exactly on
             the same contour level, in which case only the side of the box
             is drawn.
                vertex 4 +-------------------+ vertex 3
                         | \               / |
                         |   \    m-3    /   |
                         |     \       /     |
                         |       \   /       |
                         |  m=2    X   m=2   |       the centre is vertex 0
                         |       /   \       |
                         |     /       \     |
                         |   /    m=1    \   |
                         | /               \ |
                vertex 1 +-------------------+ vertex 2
            */

            // Scan each triangle in the box
            for (int m = 1; m <= 4; m++) {
              // figure out which intersection we need to do
              m1 = m;
              m2 = 0;
              m3 = (m != 4) ? m + 1 : 1;
              int case_index = case_table[sh[m1] + 1][sh[m2] + 1][sh[m3] + 1];
              bool swap_points = swap_table[sh[m1] + 1][sh[m2] + 1][sh[m3] + 1];

              // there is no intersection of this triangle
              if (case_index == 0) {
                continue;
              }

              // do the intersection, assigns to pt1 and pt2 inside lambdas defined above
              cases[case_index]();

              // this isnt a segment..
              if (from_pt == to_pt) {
                continue;
              }
              if (swap_points) {
                std::swap(from_pt, to_pt);
              }

              // see if we have anything to connect this segment to
              typename contour_lookup_t::iterator end_lookup_it = end_lookup.find(from_pt);
              typename contour_lookup_t::iterator begin_lookup_it = begin_lookup.find(to_pt);

              if (end_lookup_it != end_lookup.end() && begin_lookup_it != begin_lookup.end()) {
                // we want to merge two records
                //   first_segment                               second_segment
                // (... ------> from_pt) + (from_pt, to_pt) + (to_pt ------> ...)
                auto first_segment = end_lookup_it->second;
                auto second_segment = begin_lookup_it->second;
                end_lookup.erase(end_lookup_it);
                begin_lookup.erase(begin_lookup_it);

                // this segment is now a ring
                if (first_segment == second_segment) {
                  first_segment->push_back(first_segment->front());
                  continue;
                }

                end_lookup[second_segment->back()] = first_segment;
                first_segment->splice(first_segment->end(), *second_segment);
                contour.front().erase(second_segment);
              } else if (end_lookup_it != end_lookup.end()) {
                // (... ------> from_pt) + (from_pt, to_pt)
                end_lookup_it->second->push_back(to_pt);
                end_lookup.emplace(to_pt, end_lookup_it->second);
                end_lookup.erase(end_lookup_it);
              } else if (begin_lookup_it != begin_lookup.end()) {
                // (from_pt, to_pt) + (to_pt ------> ...)
                begin_lookup_it->second->push_front(from_pt);
                begin_lookup.emplace(from_pt, begin_lookup_it->second);
                begin_lookup.erase(begin_lookup_it);
              } else {
                // this is an orphan segment for now
                contour.front().push_front(contour_t{from_pt, to_pt});
                begin_lookup.emplace(from_pt, contour.front().begin());
                end_lookup.emplace(to_pt, contour.front().begin());
              }
            }
          } // Each contour
        }   // Each tile col
      }     // Each tile row
    }       // Each dimension of the grid

    // If the generalization value equals kOptimalGeneralization then set
    // the generalization factor to 1/4 of the grid size
    float gen_factor = generalize;
    if (generalize == kOptimalGeneralization) {
      gen_factor = this->tilesize_ * 0.25f * kMetersPerDegreeLat;
    }

    // some info about the area the image covers
    auto h = this->tilesize_ / 2;
    // for each contour
    for (auto& collection : contours) {
      auto& contour = collection.front();
      // they only wanted rings
      if (rings_only) {
        contour.remove_if([](const contour_t& line) { return line.front() != line.back(); });
      }
      // sort them by area (maybe length would be sufficient?) biggest first
      std::unordered_map<const contour_t*, typename PointLL::first_type> cache(contour.size());
      std::for_each(contour.cbegin(), contour.cend(),
                    [&cache](const contour_t& c) { cache[&c] = polygon_area(c); });
      contour.sort([&cache](const contour_t& a, const contour_t& b) {
        return std::abs(cache[&a]) > std::abs(cache[&b]);
      });

      // they only want the most significant ones!
      if (denoise > 0.f) {
        contour.remove_if([&cache, &contour, denoise](const contour_t& c) {
          return std::abs(cache[&c] / cache[&contour.front()]) < denoise;
        });
      }
      // clean up the lines
      for (auto& line : contour) {
        if (gen_factor > 0.f) {
          Polyline2<PointLL>::Generalize(line, gen_factor, {}, /* avoid_self_intersections */ true);
        }
        // sampling the bottom left corner means everything is skewed, so unskew it
        for (auto& coord : line) {
          coord.first += h;
          coord.second += h;
        }
      }
      // remove points and lines
      contour.remove_if([](const contour_t& line) { return line.size() < 4; });

      // if they just wanted linestrings we need only one per feature
      if (!rings_only) {
        for (auto& linestring : contour) {
          collection.push_back({std::move(linestring)});
        }
        collection.pop_front();
      }
    }

    return contours;
  }

  /**
   * Determine the smallest subgrid that contains all valid (i.e. non-max) values

   * @return array with 4 elements: minimum column, minimum row, maximum column, maximum row
   */
  const std::array<int32_t, 4> MinExtent() const {
    // minx, miny, maxx, maxy
    std::array<int32_t, 4> box = {this->ncolumns_ / 2, this->nrows_ / 2, this->ncolumns_ / 2,
                                  this->nrows_ / 2};

    for (int32_t i = 0; i < this->nrows_; ++i) {
      for (int32_t j = 0; j < this->ncolumns_; ++j) {
        if (data_[this->TileId(j, i)][0] < max_value_[0] ||
            data_[this->TileId(j, i)][1] < max_value_[1]) {
          // pad by 1 row/column as a sanity check
          box[0] = std::min(std::max(j - 1, 0), box[0]);
          box[1] = std::min(std::max(i - 1, 0), box[1]);
          // +1 extra because range is exclusive
          box[2] = std::max(std::min(j + 2, this->ncolumns_ - 1), box[2]);
          box[3] = std::max(std::min(i + 2, this->ncolumns_ - 1), box[3]);
        }
      }
    }

    return box;
  }

protected:
  value_type max_value_;         // Maximum value stored in the tile
  std::vector<value_type> data_; // Data value within each tile
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_GRIDDEDDATA_H_
