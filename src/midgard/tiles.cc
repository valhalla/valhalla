#include "midgard/tiles.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "midgard/distanceapproximator.h"
#include <cmath>
#include <set>

namespace {

  //this is modified to include all pixels that are intersected by the floating point line
  //at each step it decides to either move in the x or y direction based on which pixels midpoint
  //forms a smaller triangle with the line. to avoid edge cases we allow set_pixel to make the
  //the loop bail if we leave the valid drawing region
  void bresenham_line(float x0, float y0, float x1, float y1, const std::function<bool (int32_t, int32_t)>& set_pixel) {
    //this one for sure
    bool outside = set_pixel(std::floor(x0), std::floor(y0));
    //steps in the proper direction and constants for shoelace formula
    float sx = x0 < x1 ? 1 : -1, dx = x1 - x0, x = std::floor(x0) + .5f;
    float sy = y0 < y1 ? 1 : -1, dy = y1 - y0, y = std::floor(y0) + .5f;
    //keep going until we make it to the ending pixel
    while(std::floor(x) != std::floor(x1) || std::floor(y) != std::floor(y1)) {
      float tx = std::abs(dx*(y - y0) - dy*((x + sx) - x0));
      float ty = std::abs(dx*((y + sy) - y0) - dy*(x - x0));
      //less error moving in the x
      if(tx < ty) { x += sx; }
      //less error moving in the y
      else { y += sy; }
      //mark this pixel
      bool o = set_pixel(std::floor(x), std::floor(y));
      if(outside == false && o == true)
        return;
      outside = o;
    }
  }

  //a functor to generate closest first subdivisions of a set of tiles
  template <class coord_t>
  struct closest_first_generator_t {
    coord_t seed;
    valhalla::midgard::Tiles<coord_t> tiles;
    int32_t subcols, subrows;
    std::unordered_set<int32_t> queued;
    using best_t = std::pair<float, int32_t>;
    std::set<best_t, std::function<bool (const best_t&, const best_t&) > > queue;

    closest_first_generator_t(const valhalla::midgard::Tiles<coord_t>& tiles, const coord_t& seed):
      tiles(tiles), seed(seed), queued(100),
      queue([](const best_t&a, const best_t& b)
        {return a.first == b.first ? a.second < b.second : a.first < b.first;}) {
      //what global subdivision are we starting in
      //TODO: worry about wrapping around valid range
      subcols = tiles.ncolumns() * tiles.nsubdivisions();
      subrows = tiles.nrows() * tiles.nsubdivisions();
      auto x = (seed.first - tiles.TileBounds().minx()) / tiles.TileBounds().Width() * subcols;
      auto y = (seed.second - tiles.TileBounds().miny()) / tiles.TileBounds().Height() * subrows;
      auto subdivision = static_cast<int32_t>(y) * subcols + static_cast<int32_t>(x);
      queued.emplace(subdivision);
      queue.emplace(std::make_pair(0, subdivision));
      neighbors(subdivision);
    }

    //something to measure the closest possible point of a subdivision from the given seed point
    float dist(int32_t sub) {
      auto x = sub % subcols;
      auto x0 = tiles.TileBounds().minx() + x * tiles.SubdivisionSize();
      auto x1 = tiles.TileBounds().minx() + (x + 1) * tiles.SubdivisionSize();
      auto y = sub / subcols;
      auto y0 = tiles.TileBounds().miny() + y * tiles.SubdivisionSize();
      auto y1 = tiles.TileBounds().miny() + (y + 1) * tiles.SubdivisionSize();
      auto distance = std::numeric_limits<float>::max();
      std::list<coord_t> corners{ {x0, y0}, {x1, y0}, {x0, y1}, {x1, y1} };
      if(x0 < seed.first && x1 > seed.first) { corners.emplace_back(seed.first, y0); corners.emplace_back(seed.first, y1); }
      if(y0 < seed.second && y1 > seed.second) { corners.emplace_back(x0, seed.second); corners.emplace_back(x1, seed.second); }
      for(const auto& c : corners) {
        auto d = seed.Distance(c);
        if(d < distance)
          distance = d;
      }
      return distance;
    }

    //something to add the neighbors of a given subdivision
    const std::list<std::pair<int, int> > neighbor_offsets{ {0,-1}, {-1,0}, {1,0}, {0,1} };
    void neighbors(int32_t s) {
      //walk over all adjacent subdivisions in row major order
      auto x = s % subcols;
      auto y = s / subcols;
      for(const auto& off : neighbor_offsets) {
        //skip y out of bounds
        auto ny = y + off.second;
        if(ny == -1 || ny == subrows)
          continue;
        //fix x
        auto nx = x + off.first;
        if(nx == -1 || nx == subcols){
          if(!coord_t::IsSpherical())
            continue;
          nx = (nx + subcols) % subcols;
        }
        //actually add the thing
        auto neighbor = ny * subcols + nx;
        if(queued.find(neighbor) == queued.cend()) {
          queued.emplace(neighbor);
          queue.emplace(std::make_pair(dist(neighbor), neighbor));
        }
      }
    }

    //get the next closest subdivision
    std::tuple<int32_t, unsigned short, float> next(){
      //get the next closest one or bail
      if(!queue.size())
        throw std::runtime_error("Subdivisions were exhausted");
      auto best = *queue.cbegin();
      queue.erase(queue.cbegin());
      //add its neighbors
      neighbors(best.second);
      //return it
      auto sx = best.second % subcols;
      auto sy = best.second / subcols;
      auto tile_column = sx / tiles.nsubdivisions();
      auto tile_row = sy / tiles.nsubdivisions();
      auto tile = tile_row * tiles.ncolumns() + tile_column;
      unsigned short subdivision = (sy - tile_row * tiles.nsubdivisions()) * tiles.nsubdivisions() +
                                   (sx - tile_column * tiles.nsubdivisions());
      return std::make_tuple(tile, subdivision, best.first);
    }
  };

}

namespace valhalla {
namespace midgard {

// Constructor.  A bounding box and tile size is specified.
// Sets class data members and computes the number of rows and columns
// based on the bounding box and tile size.
template <class coord_t>
Tiles<coord_t>::Tiles(const AABB2<coord_t>& bounds, const float tilesize, unsigned short subdivisions, bool wrapx):
  tilebounds_(bounds), tilesize_(tilesize), nsubdivisions_(subdivisions), wrapx_(wrapx) {
  tilebounds_ = bounds;
  tilesize_ = tilesize;
  subdivision_size_ = tilesize_ / nsubdivisions_;
  ncolumns_ = static_cast<int32_t>(ceil((bounds.maxx() - bounds.minx()) / tilesize_));
  nrows_    = static_cast<int32_t>(ceil((bounds.maxy() - bounds.miny()) / tilesize_));
}

// Shift the tilebounds
template <class coord_t>
void Tiles<coord_t>::ShiftTileBounds(const coord_t& shift) {
  tilebounds_ = AABB2<coord_t>(tilebounds_.minx() - shift.first,
                               tilebounds_.miny() - shift.second,
                               tilebounds_.maxx() - shift.first,
                               tilebounds_.maxy() - shift.second);
}

// Get the "row" based on y.
template <class coord_t>
int32_t Tiles<coord_t>::Row(const float y) const {
  // Return -1 if outside the tile system bounds
  if (y < tilebounds_.miny() || y > tilebounds_.maxy())
    return -1;

  // If equal to the max y return the largest row
  if (y == tilebounds_.maxy())
    return nrows_ - 1;
  else {
    return static_cast<int32_t>((y - tilebounds_.miny()) / tilesize_);
  }
}

// Get the "column" based on x.
template <class coord_t>
int32_t Tiles<coord_t>::Col(const float x) const {
  // Return -1 if outside the tile system bounds
  if (x < tilebounds_.minx() || x > tilebounds_.maxx())
    return -1;

  // If equal to the max x return the largest column
  if (x == tilebounds_.maxx())
    return ncolumns_ - 1;
  else {
    float col = (x - tilebounds_.minx()) / tilesize_;
    return (col >= 0.0) ? static_cast<int32_t>(col) :
                          static_cast<int32_t>(col - 1);
  }
}

// Convert x,y to a tile Id.
template <class coord_t>
int32_t Tiles<coord_t>::TileId(const float y, const float x) const {
  // Return -1 if totally outside the extent.
  if (y < tilebounds_.miny() || x < tilebounds_.minx() ||
      y > tilebounds_.maxy() || x > tilebounds_.maxx())
    return -1;

  // Find the tileid by finding the latitude row and longitude column
  return (Row(y) * ncolumns_) + Col(x);
}

// Get a maximum tileid given a bounds and a tile size.
template <class coord_t>
uint32_t Tiles<coord_t>::MaxTileId(const AABB2<coord_t>& bbox,
                                   const float tile_size) {
  uint32_t cols = static_cast<uint32_t>(std::ceil(bbox.Width() / tile_size));
  uint32_t rows = static_cast<uint32_t>(std::ceil(bbox.Height() / tile_size));
  return (cols * rows) - 1;
}

// Get the base x,y (or lng,lat) of a specified tile.
template <class coord_t>
coord_t Tiles<coord_t>::Base(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  return coord_t(tilebounds_.minx() + (col * tilesize_),
                 tilebounds_.miny() + (row * tilesize_));
}

// Get the bounding box of the specified tile.
template <class coord_t>
AABB2<coord_t> Tiles<coord_t>::TileBounds(const int32_t tileid) const {
  Point2 base = Base(tileid);
  return AABB2<coord_t>(base.x(), base.y(),
                        base.x() + tilesize_, base.y() + tilesize_);
}

// Get the bounding box of the tile with specified row, column.
template <class coord_t>
AABB2<coord_t> Tiles<coord_t>::TileBounds(const int32_t col,
                                          const int32_t row) const {
  float basex = tilebounds_.minx() + ((float) col * tilesize_);
  float basey = tilebounds_.miny() + ((float) row * tilesize_);
  return AABB2<coord_t>(basex, basey, basex + tilesize_, basey + tilesize_);
}

// Get the center of the specified tile.
template <class coord_t>
coord_t Tiles<coord_t>::Center(const int32_t tileid) const {
  auto base = Base(tileid);
  return coord_t(base.x() + tilesize_ * 0.5, base.y() + tilesize_ * 0.5);
}

// Get the tile offsets (row,column) between the previous tile Id and
// a new tileid.  The offsets are returned through arguments (references).
// Offsets can be positive or negative or 0.
template <class coord_t>
void Tiles<coord_t>::TileOffsets(const int32_t initial_tileid, const int32_t newtileid,
                        int& delta_rows, int& delta_cols) const {
  int32_t deltaTile = newtileid - initial_tileid;
  delta_rows = (newtileid / ncolumns_) - (initial_tileid / ncolumns_);
  delta_cols = deltaTile - (delta_rows * ncolumns_);
}

// Get the number of tiles in the tiling system.
template <class coord_t>
uint32_t Tiles<coord_t>::TileCount() const {
  float nrows = (tilebounds_.maxy() - tilebounds_.miny()) / tilesize_;
  return ncolumns_ * static_cast<int32_t>(ceil(nrows));
}

// Get the neighboring tileid to the right/east.
template <class coord_t>
int32_t Tiles<coord_t>::RightNeighbor(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  if (col < ncolumns_ - 1) {
    return tileid + 1;
  } else {
    return wrapx_ ? tileid - ncolumns_ + 1 : tileid;
  }
}

// Get the neighboring tileid to the left/west.
template <class coord_t>
int32_t Tiles<coord_t>::LeftNeighbor(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  if (col > 0) {
    return tileid - 1;
  } else {
    return wrapx_ ? tileid + ncolumns_ - 1 : tileid;
  }
}

// Get the list of tiles that lie within the specified bounding box.
// The method finds the center tile and spirals out by finding neighbors
// and recursively checking if tile is inside and checking/adding
// neighboring tiles
template <class coord_t>
std::vector<int> Tiles<coord_t>::TileList(const AABB2<coord_t>& bbox) const {
  // Check if x range needs to be split
  std::vector<AABB2<coord_t>> bboxes;
  if (wrapx_) {
    if (bbox.minx() < tilebounds_.minx() &&
        bbox.maxx() > tilebounds_.minx()) {
      // Create 2 bounding boxes
      bboxes.emplace_back(tilebounds_.minx(), bbox.miny(),
                          bbox.maxx(), bbox.maxy());
      bboxes.emplace_back(bbox.minx() +  tilebounds_.Width(), bbox.miny(),
                          tilebounds_.maxx(), bbox.maxy());
    } else if (bbox.minx() < tilebounds_.maxx() &&
               bbox.maxx() > tilebounds_.maxx()) {
      // Create 2 bounding boxes
      bboxes.emplace_back(bbox.minx(), bbox.miny(),
                          tilebounds_.maxx(), bbox.maxy());
      bboxes.emplace_back(tilebounds_.minx(), bbox.miny(),
                          bbox.maxx() - tilebounds_.Width(), bbox.maxy());
    } else {
      bboxes.push_back(bbox.Intersection(tilebounds_));
    }
  } else {
    bboxes.push_back(bbox.Intersection(tilebounds_));
  }

  std::vector<int32_t> tilelist;
  for (auto bb : bboxes) {
    int32_t minrow = std::max(Row(bb.miny()), 0);
    int32_t maxrow = std::max(Row(bb.maxy()), 0);
    int32_t mincol = std::max(Col(bb.minx()), 0);
    int32_t maxcol = std::max(Col(bb.maxx()), 0);
    for (int32_t row = minrow; row <= maxrow; ++row) {
      int32_t tileid = TileId(mincol, row);
      for (int32_t col = mincol; col <= maxcol; ++col, ++tileid) {
        tilelist.push_back(tileid);
      }
    }
  }
  return tilelist;
}

// Color a "connectivity map" starting with a sparse map of uncolored tiles.
// Any 2 tiles that have a connected path between them will have the same
// value in the connectivity map.
template <class coord_t>
void Tiles<coord_t>::ColorMap(std::unordered_map<uint32_t,
                              size_t>& connectivity_map) const {
  // Connectivity map - all connected regions will have a unique Id. If any 2
  // tile Ids have a different Id they are judged to be not-connected.

  // Iterate through tiles
  size_t color = 1;
  for (auto& tile : connectivity_map) {
    // Continue if already visited
    if (tile.second > 0) {
      continue;
    }

    // Mark this tile Id with the current color and find all its
    // accessible neighbors
    tile.second = color;
    std::list<uint32_t> checklist{tile.first};
    while (!checklist.empty()) {
      uint32_t next_tile = checklist.front();
      checklist.pop_front();

      // Check neighbors.
      uint32_t neighbor = LeftNeighbor(next_tile);
      auto neighbor_itr = connectivity_map.find(neighbor);
      if (neighbor_itr != connectivity_map.cend() && neighbor_itr->second == 0) {
        checklist.push_back(neighbor);
        neighbor_itr->second = color;
      }
      neighbor = RightNeighbor(next_tile);
      neighbor_itr = connectivity_map.find(neighbor);
      if (neighbor_itr != connectivity_map.cend() && neighbor_itr->second == 0) {
        checklist.push_back(neighbor);
        neighbor_itr->second = color;
      }
      neighbor = TopNeighbor(next_tile);
      neighbor_itr = connectivity_map.find(neighbor);
      if (neighbor_itr != connectivity_map.cend() && neighbor_itr->second == 0) {
        checklist.push_back(neighbor);
        neighbor_itr->second = color;
      }
      neighbor = BottomNeighbor(next_tile);
      neighbor_itr = connectivity_map.find(neighbor);
      if (neighbor_itr != connectivity_map.cend() && neighbor_itr->second == 0) {
        checklist.push_back(neighbor);
        neighbor_itr->second = color;
      }
    }

    // Increment color
    color++;
  }
}

template <class coord_t>
template <class container_t>
std::unordered_map<int32_t, std::unordered_set<unsigned short> > Tiles<coord_t>::Intersect(const container_t& linestring) const {
  std::unordered_map<int32_t, std::unordered_set<unsigned short> > intersection;

  //what to do when we want to mark a subdivision as containing a segment of this linestring
  const auto set_pixel = [this, &intersection](int32_t x, int32_t y) {
    //cant mark ones that are outside the valid range of tiles
    //TODO: wrap coordinates around x and y?
    if(x < 0 || y < 0 || x >= nsubdivisions_ * ncolumns_ || y >= nsubdivisions_ * nrows_)
      return true;
    //find the tile
    int32_t tile_column = x / nsubdivisions_;
    int32_t tile_row = y / nsubdivisions_;
    int32_t tile = tile_row * ncolumns_ + tile_column;
    //find the subdivision
    unsigned short subdivision = (y % nsubdivisions_) * nsubdivisions_ + (x % nsubdivisions_);
    intersection[tile].insert(subdivision);
    return false;
  };

  //if coord_t is spherical and the segment uv is sufficiently long then the geodesic along it
  //cannot be approximated with linear constructs so instead we resample it at a sufficiently
  //small interval so as to approximate the arc with piecewise linear segments
  container_t resampled;
  auto max_meters = std::max(1.f,
    subdivision_size_ * .25f * DistanceApproximator::MetersPerLngDegree(linestring.front().second));
  if(coord_t::IsSpherical() && Polyline2<coord_t>::Length(linestring) > max_meters)
    resampled = resample_spherical_polyline(linestring, max_meters, true);

  //for each segment
  const auto& line = resampled.size() ? resampled : linestring;
  auto ui = line.cbegin(), vi = line.cbegin();
  while(vi != line.cend()) {
    //figure out what the segment is
    auto u = *ui;
    auto v = u;
    std::advance(vi, 1);
    if(vi != line.cend())
      v = *vi;
    else if(line.size() > 1)
      return intersection;
    ui = vi;

    //figure out global subdivision start and end points
    auto x0 = (u.first - tilebounds_.minx()) / tilebounds_.Width() * ncolumns_ * nsubdivisions_;
    auto y0 = (u.second - tilebounds_.miny()) / tilebounds_.Height() * nrows_ * nsubdivisions_;
    auto x1 = (v.first - tilebounds_.minx()) / tilebounds_.Width() * ncolumns_ * nsubdivisions_;
    auto y1 = (v.second - tilebounds_.miny()) / tilebounds_.Height() * nrows_ * nsubdivisions_;

    int ix0 = std::floor(x0), ix1 = std::floor(x1);
    int iy0 = std::floor(y0), iy1 = std::floor(y1);
    int dx = ix0 - ix1, dy = iy0 - iy1;
    int ds = dx*dx + dy*dy;
    //its likely for our use case that its all in one cell
    if(ds == 0) { set_pixel(ix0, iy0); }
    //if not the next most likley thing is adjacent cells
    else if(ds == 1) { set_pixel(ix0, iy0); set_pixel(ix1, iy1); }
    //pretend the subdivisions are pixels and we are doing line rasterization
    else { bresenham_line(x0, y0, x1, y1, set_pixel); }
  }

  //give them back
  return intersection;
}

template <class coord_t>
std::unordered_map<int32_t, std::unordered_set<uint16_t> > Tiles<coord_t>::Intersect(const AABB2<coord_t> &box) const {
  std::unordered_map<int32_t, std::unordered_set<uint16_t> > intersection;

  // to calculate the bounds within each tile, we first calculate all the
  // subdivisions (bins) which the bounding box covers in global space, and
  // then iterate over them to fill in each "pixel" or bin.
  const int32_t x_pixels = ncolumns_ * nsubdivisions_;
  const int32_t y_pixels = nrows_ * nsubdivisions_;
  // NOTE: multiply by pixels before dividing to keep as much precision as
  // possible.
  int32_t x0 = std::floor(((box.minx() - tilebounds_.minx()) * x_pixels) / tilebounds_.Width());
  int32_t y0 = std::floor(((box.miny() - tilebounds_.miny()) * y_pixels) / tilebounds_.Height());
  int32_t x1 = std::floor(((box.maxx() - tilebounds_.minx()) * x_pixels) / tilebounds_.Width());
  int32_t y1 = std::floor(((box.maxy() - tilebounds_.miny()) * y_pixels) / tilebounds_.Height());

  // clamp ranges to within the bounds of the tile.
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 >= x_pixels) { x1 = x_pixels - 1; }
  if (y1 >= y_pixels) { y1 = y_pixels - 1; }

  for (auto y = y0; y <= y1; ++y) {
    for (auto x = x0; x <= x1; ++x) {
      auto tile_id = (y / nsubdivisions_) * ncolumns_ + (x / nsubdivisions_);
      auto bin = (y % nsubdivisions_) * nsubdivisions_ + (x % nsubdivisions_);
      intersection[tile_id].insert(bin);
    }
  }

  return intersection;
}

template <class coord_t>
std::function<std::tuple<int32_t, unsigned short, float>() > Tiles<coord_t>::ClosestFirst(const coord_t& seed) const {
  return std::bind(&closest_first_generator_t<coord_t>::next, closest_first_generator_t<coord_t>(*this, seed));
}

// Explicit instantiation
template class Tiles<Point2>;
template class Tiles<PointLL>;

template class std::unordered_map<int32_t, std::unordered_set<unsigned short> > Tiles<Point2>::Intersect(const std::list<Point2>&) const;
template class std::unordered_map<int32_t, std::unordered_set<unsigned short> > Tiles<PointLL>::Intersect(const std::list<PointLL>&) const;
template class std::unordered_map<int32_t, std::unordered_set<unsigned short> > Tiles<Point2>::Intersect(const std::vector<Point2>&) const;
template class std::unordered_map<int32_t, std::unordered_set<unsigned short> > Tiles<PointLL>::Intersect(const std::vector<PointLL>&) const;

}
}
