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
    PointLL pt1, pt2;                  // The intersection points in the tile
    int tile_inc[4] = {0, 1, this->ncolumns_ + 1, this->ncolumns_};

    // Find the intersection along a tile edge
    auto intersect = [&tile_corners, &s](int p1, int p2) {
      auto ds = s[p2] - s[p1];
      return PointLL((s[p2] * tile_corners[p1].first - s[p1] * tile_corners[p2].first) / ds,
                     (s[p2] * tile_corners[p1].second - s[p1] * tile_corners[p2].second) / ds);
    };

    // In the tight loop below, we need to decide where a contour intersects the triangles that make
    // up the given tile. this works out to a number of discrete cases which we lookup using the table
    // below. based on the case we perform the appropriate intersection. to avoid branching we store
    // the intersection operation for each case in an array and perform the correct one by calling the
    // function stored in the array
    int case_table[3][3][3] = {
        {{0, 0, 8}, {0, 2, 5}, {7, 6, 9}},
        {{0, 3, 4}, {1, 3, 1}, {4, 3, 0}},
        {{9, 6, 7}, {5, 2, 0}, {8, 0, 0}},
    };
    std::array<std::function<void()>, 10> cases{
        []() {},
        // Line between vertices 1 and 2
        [&]() {
          pt1 = tile_corners[m1];
          pt2 = tile_corners[m2];
        },
        // Line between vertices 2 and 3
        [&]() {
          pt1 = tile_corners[m2];
          pt2 = tile_corners[m3];
        },
        // Line between vertices 3 and 1
        [&]() {
          pt1 = tile_corners[m3];
          pt2 = tile_corners[m1];
        },
        // Line between vertex 1 and side 2-3
        [&]() {
          pt1 = tile_corners[m1];
          pt2 = intersect(m2, m3);
        },
        // Line between vertex 2 and side 3-1
        [&]() {
          pt1 = tile_corners[m2];
          pt2 = intersect(m3, m1);
        },
        // Line between vertex 3 and side 1-2
        [&]() {
          pt1 = tile_corners[m3];
          pt2 = intersect(m1, m2);
        },
        // Line between sides 1-2 and 2-3
        [&]() {
          pt1 = intersect(m1, m2);
          pt2 = intersect(m2, m3);
        },
        // Line between sides 2-3 and 3-1
        [&]() {
          pt1 = intersect(m2, m3);
          pt2 = intersect(m3, m1);
        },
        // Line between sides 3-1 and 1-2
        [&]() {
          pt1 = intersect(m3, m1);
          pt2 = intersect(m1, m2);
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
    using contour_lookup_t = std::unordered_map<PointLL, typename feature_t::iterator>;
    std::vector<contour_lookup_t> lookups(intervals.size());
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
            auto& lookup = lookups[i];
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

              // there is no intersection of this triangle
              if (case_index == 0) {
                continue;
              }

              // do the intersection, assigns to pt1 and pt2 inside lambdas defined above
              cases[case_index]();

              // this isnt a segment..
              if (pt1 == pt2) {
                continue;
              }

              // see if we have anything to connect this segment to
              typename contour_lookup_t::iterator rec_a = lookup.find(pt1);
              typename contour_lookup_t::iterator rec_b = lookup.find(pt2);
              if (rec_b != lookup.end()) {
                std::swap(pt1, pt2);
                std::swap(rec_a, rec_b);
              }

              // we want to merge two records
              if (rec_b != lookup.end()) {
                // get the segments in question and remove their lookup info
                auto segment_a = rec_a->second;
                bool head_a = rec_a->first == segment_a->front();
                auto segment_b = rec_b->second;
                bool head_b = rec_b->first == segment_b->front();
                lookup.erase(rec_a);
                lookup.erase(rec_b);

                // this segment is now a ring
                if (segment_a == segment_b) {
                  segment_a->push_back(segment_a->front());
                  continue;
                }

                // erase the other lookups
                lookup.erase(
                    lookup.find(pt1 == segment_a->front() ? segment_a->back() : segment_a->front()));
                lookup.erase(
                    lookup.find(pt2 == segment_b->front() ? segment_b->back() : segment_b->front()));

                // add b to a
                if (!head_a && head_b) {
                  segment_a->splice(segment_a->end(), *segment_b);
                  contour.front().erase(segment_b);
                } // add a to b
                else if (!head_b && head_a) {
                  segment_b->splice(segment_b->end(), *segment_a);
                  contour.front().erase(segment_a);
                  segment_a = segment_b;
                } // flip a and add b
                else if (head_a && head_b) {
                  segment_a->reverse();
                  segment_a->splice(segment_a->end(), *segment_b);
                  contour.front().erase(segment_b);
                } // flip b and add to a
                else if (!head_a && !head_b) {
                  segment_b->reverse();
                  segment_a->splice(segment_a->end(), *segment_b);
                  contour.front().erase(segment_b);
                }

                // update the look up
                lookup.emplace(segment_a->front(), segment_a);
                lookup.emplace(segment_a->back(), segment_a);
              } // ap/prepend to an existing one
              else if (rec_a != lookup.end()) {
                // it goes on the front
                if (rec_a->second->front() == pt1) {
                  rec_a->second->push_front(pt2);
                  // it goes on the back
                } else {
                  rec_a->second->push_back(pt2);
                }

                // update the lookup table
                lookup.emplace(pt2, rec_a->second);
                lookup.erase(rec_a);
              } // this is an orphan segment for now
              else {
                contour.front().push_front(contour_t{pt1, pt2});
                lookup.emplace(pt1, contour.front().begin());
                lookup.emplace(pt2, contour.front().begin());
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
    auto c = this->TileBounds().Center();
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
      long orig_normalize_time = 0;
      long new_normalize_time = 0;
      for (auto& line : contour) {
        // TODO: generalizing makes self intersections which makes other libraries unhappy
        if (gen_factor > 0.f) {
          long told, tnew;
          std::tie(told, tnew) = Polyline2<PointLL>::Generalize(line, gen_factor, {});
          orig_normalize_time += told;
          new_normalize_time += tnew;
        }
        // if this ends up as an inner we'll undo this later
        if (cache[&line] > 0) {
          line.reverse();
        }
        // sampling the bottom left corner means everything is skewed, so unskew it
        for (auto& coord : line) {
          coord.first += h;
          coord.second += h;
        }
      }

      printf("Orig normalize time: %ld ms\n", orig_normalize_time);
      printf(" New normalize time: %ld ms\n", new_normalize_time);

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

protected:
  value_type max_value_;         // Maximum value stored in the tile
  std::vector<value_type> data_; // Data value within each tile
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_GRIDDEDDATA_H_
