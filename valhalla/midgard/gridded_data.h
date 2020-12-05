#ifndef VALHALLA_MIDGARD_GRIDDEDDATA_H_
#define VALHALLA_MIDGARD_GRIDDEDDATA_H_

#include <algorithm>
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

// TODO: this should really be sif::Cost, but it
// doesnt yet track distance, only the predecessor does
struct time_distance_t {
  float sec;
  float dist;

  bool operator<(const time_distance_t& other) const {
    return sec < other.sec;
  }
};

// const std::initializer_list<IsoMetric> IsoMetricsAll = {IsoMetric::kTime, IsoMetric::kDistance};

/**
 * Class to store data in a gridded/tiled data structure. Contains methods
 * to mark each tile with data using a compare operator.
 */
template <class value_t> class GriddedData : public Tiles<PointLL> {
public:
  /**
   * Constructor.
   * @param   bounds    Bounding box
   * @param   tilesize  Tile size
   * @param   value     Value to initialize data with.
   */
  GriddedData(const AABB2<PointLL>& bounds, const float tilesize, const value_t& value)
      : Tiles<PointLL>(bounds, tilesize), max_value_(value) {
    // Resize the data vector and fill with the value
    data_.resize(this->nrows_ * this->ncolumns_);
    std::fill(data_.begin(), data_.end(), value);
  }

  /**
   * Set the value at a specified point. Verifies that the point is within the
   * tiles.
   * @param  pt     Coordinate to set within the tiles.
   * @param  value  Value to set at the tile/grid location.
   * @return whether or not the value was set
   */
  bool Set(const PointLL& pt, const value_t& value) {
    auto cell_id = this->TileId(pt);
    if (cell_id >= 0 && cell_id < data_.size()) {
      data_[cell_id] = value;
      return true;
    }
    return false;
  }

  /**
   * Set the value at a specified tile Id if the value is less than the current
   * value set at the grid location. Verifies that the tile is valid.
   * @param  tile_id  Tile Id to set value for.
   * @param  value    Value to set at the tile/grid location.
   * @param  get_data Functor to get the desired data value
   * @param  set_data Functor to set the desired data value
   */
  void SetIfLessThan(const int tile_id,
                     const value_t& value,
                     const std::function<float(const value_t&)>& get_data,
                     const std::function<void(value_t&, const float)> set_data) {
    auto data_value = get_data(data_[tile_id]);
    auto test_value = get_data(value);
    if (tile_id >= 0 && tile_id < data_.size() && test_value < data_value) {
      set_data(data_[tile_id], test_value);
    }
  }

  /**
   * Set the value at a specified point if the value is less than the current
   * value set at the grid location. Verifies that the point is within the
   * tiles.
   * @param  pt       Coordinate to set within the tiles.
   * @param  value    Value to set at the tile/grid location.
   * @param  get_data Functor to get the desired data value
   * @param  set_data Functor to set the desired data value
   */
  void SetIfLessThan(const PointLL& pt,
                     const value_t& value,
                     const std::function<float(const value_t&)>& get_data,
                     const std::function<void(value_t&, const float)> set_data) {
    int32_t cell_id = this->TileId(pt);
    SetIfLessThan(cell_id, value, get_data, set_data);
  }

  /**
   * Get the array of data.
   * @return  Returns the data associated with the tiles.
   */
  const std::vector<value_t>& data() const {
    return data_;
  }

  using contour_t = std::list<PointLL>;
  using feature_t = std::list<contour_t>;
  using contours_t = std::map<float, std::list<feature_t>>;
  /**
   * TODO: implement two versions of this, leave this one for linestring contours
   * and make another for polygons
   *
   * Generate contour lines from the gridded data.
   * contours is an ordered list of contour interval values
   * Derivation from the C code version of CONREC by Paul Bourke: http://paulbourke.net/papers/conrec/
   *
   * @param contour_intervals    the values at which the contour lines should occur
   *                             basically the lines on the measuring stick
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
  contours_t GenerateContours(const std::vector<float>& contour_intervals,
                              const std::function<float(const value_t&)>& get_data,
                              const bool rings_only = false,
                              const float denoise = 1.f,
                              const float generalize = 200.f) const {
    // TODO: sort and validate contour range

    // Values at tile corners and center (0 element is center)
    int sh[5];
    typename PointLL::first_type s[5]; // Values at the tile corners and center
    PointLL tile_corners[5];           // PointLL at tile corners and center

    // Find the intersection along a tile edge
    auto intersect = [&tile_corners, &s](int p1, int p2) {
      auto ds = s[p2] - s[p1];
      return PointLL((s[p2] * tile_corners[p1].x() - s[p1] * tile_corners[p2].x()) / ds,
                     (s[p2] * tile_corners[p1].y() - s[p1] * tile_corners[p2].y()) / ds);
    };

    // we need something to hold each iso-line, they're already sorted
    contours_t contours;
    for (auto v : contour_intervals) {
      contours[v].emplace_back();
    }
    // and something to find them quickly
    using contour_lookup_t = std::unordered_map<PointLL, typename feature_t::iterator>;
    std::unordered_map<float, contour_lookup_t> lookup(contour_intervals.size());
    // TODO: preallocate the lookups for each interval

    int tile_inc[4] = {0, 1, this->ncolumns_ + 1, this->ncolumns_};
    int case_value;
    int case_table[3][3][3] = {{{0, 0, 8}, {0, 2, 5}, {7, 6, 9}},
                               {{0, 3, 4}, {1, 3, 1}, {4, 3, 0}},
                               {{9, 6, 7}, {5, 2, 0}, {8, 0, 0}}};

    // For each cell, skipping the outer rim since its out of bounds
    for (int row = 1; row < this->nrows_ - 1; ++row) {
      for (int col = 1; col < this->ncolumns_ - 1; ++col) {
        int tileid = this->TileId(col, row);
        auto cell1 = get_data(data_[tileid]);
        auto cell2 = get_data(data_[tileid + this->ncolumns_]);     // TileId(col,   row+1)];
        auto cell3 = get_data(data_[tileid + 1]);                   // TileId(col+1, row)];
        auto cell4 = get_data(data_[tileid + this->ncolumns_ + 1]); // TileId(col+1, row+1)];
        auto dmin = std::min(std::min(cell1, cell2), std::min(cell3, cell4));
        auto dmax = std::max(std::max(cell1, cell2), std::max(cell3, cell4));

        // Continue if outside the range of contour values
        if (dmax < contour_intervals.front() || dmin > contour_intervals.back()) {
          continue;
        }

        for (auto contour : contour_intervals) {
          if (contour < dmin || contour > dmax) {
            continue;
          }
          for (int m = 4; m >= 0; m--) {
            if (m > 0) {
              int newtileid = tileid + tile_inc[m - 1];
              // Make sure the tile corner value is not set to the max_value
              // (messes up the intersect method). Set a value slightly above
              // the contour (e.g. 1 minute higher).
              // TODO - the value 1 is a bit of a hack.
              float nd = get_data(data_[newtileid]);
              s[m] = (nd < get_data(max_value_)) ? nd - contour : 1.0f;
              tile_corners[m] = this->Base(newtileid);
            } else {
              s[0] = 0.25 * (s[1] + s[2] + s[3] + s[4]);
              tile_corners[0] = this->Center(tileid);
            }
            if (s[m] > 0.0f) {
              sh[m] = 1;
            } else if (s[m] < 0.0f) {
              sh[m] = -1;
            } else {
              sh[m] = 0;
            }
          }

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
          PointLL pt1, pt2;
          for (int m = 1; m <= 4; m++) {
            int m1 = m;
            int m2 = 0;
            int m3 = (m != 4) ? m + 1 : 1;
            if ((case_value = case_table[sh[m1] + 1][sh[m2] + 1][sh[m3] + 1]) == 0) {
              continue;
            }

            switch (case_value) {
              case 1: // Line between vertices 1 and 2
                pt1 = tile_corners[m1];
                pt2 = tile_corners[m2];
                break;
              case 2: // Line between vertices 2 and 3
                pt1 = tile_corners[m2];
                pt2 = tile_corners[m3];
                break;
              case 3: // Line between vertices 3 and 1
                pt1 = tile_corners[m3];
                pt2 = tile_corners[m1];
                break;
              case 4: // Line between vertex 1 and side 2-3
                pt1 = tile_corners[m1];
                pt2 = intersect(m2, m3);
                break;
              case 5: // Line between vertex 2 and side 3-1
                pt1 = tile_corners[m2];
                pt2 = intersect(m3, m1);
                break;
              case 6: // Line between vertex 3 and side 1-2
                pt1 = tile_corners[m3];
                pt2 = intersect(m1, m2);
                break;
              case 7: // Line between sides 1-2 and 2-3
                pt1 = intersect(m1, m2);
                pt2 = intersect(m2, m3);
                break;
              case 8: // Line between sides 2-3 and 3-1
                pt1 = intersect(m2, m3);
                pt2 = intersect(m3, m1);
                break;
              case 9: // Line between sides 3-1 and 1-2
                pt1 = intersect(m3, m1);
                pt2 = intersect(m1, m2);
                break;
              default:
                break;
            }

            // this isnt a segment..
            if (pt1 == pt2) {
              continue;
            }

            // see if we have anything to connect this segment to
            typename contour_lookup_t::iterator rec_a = lookup[contour].find(pt1);
            typename contour_lookup_t::iterator rec_b = lookup[contour].find(pt2);
            if (rec_b != lookup[contour].end()) {
              std::swap(pt1, pt2);
              std::swap(rec_a, rec_b);
            }

            // we want to merge two records
            if (rec_b != lookup[contour].end()) {
              // get the segments in question and remove their lookup info
              auto segment_a = rec_a->second;
              bool head_a = rec_a->first == segment_a->front();
              auto segment_b = rec_b->second;
              bool head_b = rec_b->first == segment_b->front();
              lookup[contour].erase(rec_a);
              lookup[contour].erase(rec_b);

              // this segment is now a ring
              if (segment_a == segment_b) {
                segment_a->push_back(segment_a->front());
                continue;
              }

              // erase the other lookups
              lookup[contour].erase(lookup[contour].find(
                  pt1 == segment_a->front() ? segment_a->back() : segment_a->front()));
              lookup[contour].erase(lookup[contour].find(
                  pt2 == segment_b->front() ? segment_b->back() : segment_b->front()));

              // add b to a
              if (!head_a && head_b) {
                segment_a->splice(segment_a->end(), *segment_b);
                contours[contour].front().erase(segment_b);
              } // add a to b
              else if (!head_b && head_a) {
                segment_b->splice(segment_b->end(), *segment_a);
                contours[contour].front().erase(segment_a);
                segment_a = segment_b;
              } // flip a and add b
              else if (head_a && head_b) {
                segment_a->reverse();
                segment_a->splice(segment_a->end(), *segment_b);
                contours[contour].front().erase(segment_b);
              } // flip b and add to a
              else if (!head_a && !head_b) {
                segment_b->reverse();
                segment_a->splice(segment_a->end(), *segment_b);
                contours[contour].front().erase(segment_b);
              }

              // update the look up
              lookup[contour].emplace(segment_a->front(), segment_a);
              lookup[contour].emplace(segment_a->back(), segment_a);
            } // ap/prepend to an existing one
            else if (rec_a != lookup[contour].end()) {
              // it goes on the front
              if (rec_a->second->front() == pt1) {
                rec_a->second->push_front(pt2);
                // it goes on the back
              } else {
                rec_a->second->push_back(pt2);
              }

              // update the lookup table
              lookup[contour].emplace(pt2, rec_a->second);
              lookup[contour].erase(rec_a);
            } // this is an orphan segment for now
            else {
              contours[contour].front().push_front(contour_t{pt1, pt2});
              lookup[contour].emplace(pt1, contours[contour].front().begin());
              lookup[contour].emplace(pt2, contours[contour].front().begin());
            }
          }
        } // Each contour
      }   // Each tile col
    }     // Each tile row

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
      auto& contour = collection.second.front();
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
        // TODO: generalizing makes self intersections which makes other libraries unhappy
        if (gen_factor > 0.f) {
          Polyline2<PointLL>::Generalize(line, gen_factor, {});
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
      // if they just wanted linestrings we need only one per feature
      if (!rings_only) {
        for (auto& linestring : contour) {
          collection.second.push_back({std::move(linestring)});
        }
        collection.second.pop_front();
      }
    }

    return contours;
  }

protected:
  value_t max_value_;         // Maximum value stored in the tile
  std::vector<value_t> data_; // Data value within each tile
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_GRIDDEDDATA_H_
