#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"
#include "midgard/gridded_data.h"
#include "midgard/logging.h"

namespace valhalla {
namespace midgard {

// Constructor.
template <class coord_t>
GriddedData<coord_t>::GriddedData(const AABB2<coord_t>& bounds, const float tilesize,
            const float value)
    : Tiles<coord_t>(bounds, tilesize) {
  // Resize the data vector and fill with the value
  data_.resize(this->nrows_ * this->ncolumns_);
  std::fill(data_.begin(), data_.end(), value);
}

// Set the value at a specified coordinate.
template <class coord_t>
bool GriddedData<coord_t>::Set(const coord_t& pt, const float value) {
  auto cell_id = this->TileId(pt);
  if (cell_id >= 0 && cell_id < data_.size()) {
    data_[cell_id] = value;
    return true;
  }
  return false;
}

// Set the value at a specified coordinate if less than the current value
template <class coord_t>
bool GriddedData<coord_t>::SetIfLessThan(const coord_t& pt, const float value) {
  int32_t cell_id = this->TileId(pt);
  if (cell_id >= 0 && cell_id < data_.size() && value < data_[cell_id]) {
      data_[cell_id] = value;
      return true;
  }
  return false;
}

// Get the array of times
template <class coord_t>
const std::vector<float>& GriddedData<coord_t>::data() const {
  return data_;
}

// Generate contour lines from the isotile data.
// contours is an ordered list of contour interval values
// Derivation from the C code version of CONREC by Paul Bourke:
// http://paulbourke.net/papers/conrec/
template <class coord_t>
typename GriddedData<coord_t>::contours_t GriddedData<coord_t>::GenerateContourLines(const std::vector<float>& contours_lines) {
  //TODO: sort and validate contour range

  // Values at tile corners and center (0 element is center)
  int sh[5];
  typename coord_t::first_type s[5];               // Values at the tile corners and center
  coord_t tile_corners[5];    // coord_t at tile corners and center

  // Find the intersection along a tile edge
  auto intersect = [&tile_corners, &s](int p1, int p2) {
    auto ds = s[p2] - s[p1];
    return coord_t((s[p2] * tile_corners[p1].x() - s[p1] * tile_corners[p2].x()) / ds,
                   (s[p2] * tile_corners[p1].y() - s[p1] * tile_corners[p2].y()) / ds);
  };

  //we need something to hold each iso-line
  contours_t contours(contours_lines.size());
  //and something to find them quickly
  using contour_lookup_t = std::unordered_map<coord_t, typename std::list<contour_t>::iterator>;
  std::vector<contour_lookup_t> lookup(contours_lines.size());

  int tile_inc[4] = { 0, 1, this->ncolumns_ + 1, this->ncolumns_ };
  int case_value;
  int case_table[3][3][3] = {
     { {0,0,8},{0,2,5},{7,6,9} },
     { {0,3,4},{1,3,1},{4,3,0} },
     { {9,6,7},{5,2,0},{8,0,0} }
   };

  // For each cell, skipping the outer rim since its out of bounds
  for (int row = 1; row < this->nrows_ - 1; ++row) {
    for (int col = 1; col < this->ncolumns_ - 1; ++col) {
      int tileid = this->TileId(col, row);
      auto cell1 = data_[tileid];
      auto cell2 = data_[tileid + this->ncolumns_];     // TileId(col,   row+1)];
      auto cell3 = data_[tileid + 1];                   // TileId(col+1, row)];
      auto cell4 = data_[tileid + this->ncolumns_ + 1]; // TileId(col+1, row+1)];
      auto dmin  = std::min(std::min(cell1, cell2), std::min(cell3, cell4));
      auto dmax  = std::max(std::max(cell1, cell2), std::max(cell3, cell4));

      // Continue if outside the range of contour values
      if (dmax < contours_lines.front() || dmin > contours_lines.back())
         continue;

      int k = 0;
      for (auto contour : contours_lines) {
        if (contour < dmin || contour > dmax) {
          k++;
          continue;
        }
        for (int m = 4; m >= 0; m--) {
          if (m > 0) {
            int newtileid = tileid + tile_inc[m-1];
            s[m]  = data_[newtileid] - contour;
            tile_corners[m] = this->Base(newtileid);
          } else {
            s[0]  = 0.25 * (s[1] + s[2] + s[3] + s[4]);
            tile_corners[0] = this->Center(tileid);
          }
          if (s[m] > 0.0f)
            sh[m] = 1;
          else if (s[m] < 0.0f)
            sh[m] = -1;
          else
            sh[m] = 0;
        }

        /*
         Note: at this stage the relative heights of the corners and the
         centre are in the h array, and the corresponding coordinates are
         in the xh and yh arrays. The centre of the box is indexed by 0
         and the 4 corners by 1 to 4 as shown below.
         Each triangle is then indexed by the parameter m, and the 3
         vertices of each triangle are indexed by parameters m1,m2,and m3.
         It is assumed that the centre of the box is always vertex 2
         though this isimportant only when all 3 vertices lie exactly on
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
        coord_t pt1, pt2;
        for (int m = 1; m <= 4; m++) {
          int m1 = m;
          int m2 = 0;
          int m3 = (m != 4) ? m + 1 : 1;
          if ((case_value = case_table[sh[m1]+1][sh[m2]+1][sh[m3]+1]) == 0) {
            continue;
          }

          switch (case_value) {
          case 1:              // Line between vertices 1 and 2
            pt1 = tile_corners[m1];
            pt2 = tile_corners[m2];
            break;
          case 2:              // Line between vertices 2 and 3
            pt1 = tile_corners[m2];
            pt2 = tile_corners[m3];
            break;
          case 3:              // Line between vertices 3 and 1
            pt1 = tile_corners[m3];
            pt2 = tile_corners[m1];
            break;
          case 4:              // Line between vertex 1 and side 2-3
            pt1 = tile_corners[m1];
            pt2 = intersect(m2, m3);
            break;
          case 5:              // Line between vertex 2 and side 3-1
            pt1 = tile_corners[m2];
            pt2 = intersect(m3, m1);
            break;
          case 6:              // Line between vertex 3 and side 1-2
            pt1 = tile_corners[m3];
            pt2 = intersect(m1, m2);
            break;
          case 7:              // Line between sides 1-2 and 2-3
            pt1 = intersect(m1, m2);
            pt2 = intersect(m2, m3);
          break;
          case 8:              // Line between sides 2-3 and 3-1
            pt1 = intersect(m2, m3);
            pt2 = intersect(m3, m1);
            break;
          case 9:              // Line between sides 3-1 and 1-2
            pt1 = intersect(m3, m1);
            pt2 = intersect(m1, m2);
            break;
          default:
            break;
          }

          //can we lengthen a segment from this point
          typename contour_lookup_t::iterator it;
          if((it = lookup[k].find(pt1)) != lookup[k].end()) {
            //update the segment
            if(it->second->front() == pt1)
              it->second->push_front(pt2);
            else
              it->second->push_back(pt2);
            //update the lookup table
            lookup[k].emplace(pt2, it->second);
            lookup[k].erase(it);
          }//can we lengthen a segment from this other point
          else if((it = lookup[k].find(pt2)) != lookup[k].end()) {
            //update the segment
            if(it->second->front() == pt2)
              it->second->push_front(pt1);
            else
              it->second->push_back(pt1);
            //update the lookup table
            lookup[k].emplace(pt1, it->second);
            lookup[k].erase(it);
          }//this is an orphan segment for now
          else {
            //new segment
            contours[k].push_front(contour_t{pt1, pt2});
            //indexed
            lookup[k].emplace(pt1, contours[k].begin());
            lookup[k].emplace(pt2, contours[k].begin());
          }

        }
        k++;
      } // Each contour
    } // Each tile col
  } // Each tile row

  //for each iso line
  for(auto& contour_segments : contours) {
    //lace up as many segments as possible
    size_t laced;
    do {
      //look for any pairs we can mash together
      laced = 0;
      for(auto& a : contour_segments) {
        for(auto& b : contour_segments) {
          //cant merge segment to itself or empty segments
          if(!a.size() || !b.size() || &a == &b)
            continue;
          //assume splicing
          ++laced;
          //a adds b on the end
          if(a.back() == b.front()) { b.pop_front(); a.splice(a.end(), b); }
          //b adds a on the end
          else if(b.back() == a.front()) { a.pop_front(); b.splice(b.end(), a); }
          //a adds reverse b on the end
          else if(a.back() == b.back()) { b.pop_back(); b.reverse(); a.splice(a.end(), b); }
          //reverse b adds a on the end
          else if(a.front() == b.front()) { b.pop_front(); b.reverse(); b.splice(b.end(), a); }
          //didn't splice them
          else { --laced; }
        }
      }
    } while(laced > 0);
    //clean out the empty segments
    contour_segments.remove_if([](const std::list<coord_t>& s){return !s.size();});
  }

  return contours;
}

// Explicit instantiation
template class GriddedData<Point2>;
template class GriddedData<PointLL>;

}
}
