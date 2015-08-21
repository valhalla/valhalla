#include "midgard/tiles.h"
#include <cmath>

namespace valhalla {
namespace midgard {

// Constructor.  A bounding box and tile size is specified.
// Sets class data members and computes the number of rows and columns
// based on the bounding box and tile size.
template <class coord_t>
Tiles<coord_t>::Tiles(const AABB2<coord_t>& bounds, const float tilesize) {
  tilebounds_ = bounds;
  tilesize_ = tilesize;
  ncolumns_ = static_cast<int32_t>(ceil((bounds.maxx() - bounds.minx()) /
                                        tilesize_));
  nrows_    = static_cast<int32_t>(ceil((bounds.maxy() - bounds.miny()) /
                                        tilesize_));
}

// Get the tile size. Tiles are square.
template <class coord_t>
float Tiles<coord_t>::TileSize() const {
  return tilesize_;
}

// Get the bounding box of the tiling system.
template <class coord_t>
AABB2<coord_t> Tiles<coord_t>::TileBounds() const {
  return tilebounds_;
}

// Get the number of rows in the tiling system.
template <class coord_t>
int32_t Tiles<coord_t>::nrows() const {
  return nrows_;
}

// Get the number of columns in the tiling system.
template <class coord_t>
int32_t Tiles<coord_t>::ncolumns() const {
  return ncolumns_;
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

// Convert a coordinate into a tile Id. The point is within the tile.
template <class coord_t>
int32_t Tiles<coord_t>::TileId(const coord_t& c) const {
  return TileId(c.y(), c.x());
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

// Get the tile Id given the row Id and column Id.
template <class coord_t>
int32_t Tiles<coord_t>::TileId(const int32_t col, const int32_t row) const {
  return (row * ncolumns_) + col;
}

// Get the tile row, col based on tile Id.
template <class coord_t>
std::pair<int32_t, int32_t> Tiles<coord_t>::GetRowColumn(
                        const int32_t tileid) const {
  return { tileid / ncolumns_, tileid % ncolumns_ };
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
  float basey = ((float) row * tilesize_) + tilebounds_.miny();
  float basex = ((float) col * tilesize_) + tilebounds_.minx();
  return AABB2<coord_t>(basey, basex, basey + tilesize_, basex + tilesize_);
}

// Get the center of the specified tile.
template <class coord_t>
coord_t Tiles<coord_t>::Center(const int32_t tileid) const {
  Point2 base = Base(tileid);
  return coord_t(base.x() + tilesize_ * 0.5, base.y() + tilesize_ * 0.5);
}

// Get the tile Id given a previous tile and a row, column offset.
template <class coord_t>
int32_t Tiles<coord_t>::GetRelativeTileId(const int32_t initial_tile,
                             const int32_t delta_rows,
                             const int32_t delta_cols) const {
  return initial_tile + (delta_rows * ncolumns_) + delta_cols;
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
  return (col < ncolumns_ - 1) ? tileid + 1 : tileid - ncolumns_ + 1;
}

// Get the neighboring tileid to the left/west.
template <class coord_t>
int32_t Tiles<coord_t>::LeftNeighbor(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  return (col > 0) ? tileid - 1 : tileid + ncolumns_ - 1;
}

// Get the neighboring tileid above or north.
template <class coord_t>
int32_t Tiles<coord_t>::TopNeighbor(const int32_t tileid) const {
  return (tileid < static_cast<int32_t>((TileCount() - ncolumns_))) ?
              tileid + ncolumns_ : tileid;
}

// Get the neighboring tileid below or south.
template <class coord_t>
int32_t Tiles<coord_t>::BottomNeighbor(const int32_t tileid) const {
  return (tileid < ncolumns_) ? tileid : tileid - ncolumns_;
}

// Checks if 2 tiles are neighbors (N,E,S,W).
template <class coord_t>
bool Tiles<coord_t>::AreNeighbors(const uint32_t id1, const uint32_t id2) const {
  return (id2 == TopNeighbor(id1) ||
          id2 == RightNeighbor(id1) ||
          id2 == BottomNeighbor(id1) ||
          id2 == LeftNeighbor(id1));
}

// Get the list of tiles that lie within the specified bounding box.
// The method finds the center tile and spirals out by finding neighbors
// and recursively checking if tile is inside and checking/adding
// neighboring tiles
template <class coord_t>
const std::vector<int>& Tiles<coord_t>::TileList(
              const AABB2<coord_t>& boundingbox) {
  // Get tile at the center of the bounding box. Return -1 if the center
  // of the bounding box is not within the tiling system bounding box.
  // TODO - relax this to check edges of the bounding box?
  tilelist_.clear();
  int32_t tileid = TileId(boundingbox.Center());
  if (tileid == -1)
    return tilelist_;

  // List of tiles to check if in view. Use a list: push new entries on the
  // back and pop off the front. The tile search tends to spiral out from
  // the center.
  std::list<int32_t> checklist;

  // Visited tiles
  std::unordered_set<int32_t> visited_tiles;

  // Set this tile in the checklist and it to the list of visited tiles.
  checklist.push_back(tileid);
  visited_tiles.insert(tileid);

  // Get neighboring tiles in bounding box until NextTile returns -1
  // or the maximum number specified is reached
  while (!checklist.empty()) {
    // Get the element off the front of the list and add it to the tile list.
    tileid = checklist.front();
    checklist.pop_front();
    tilelist_.push_back(tileid);

    // Check neighbors
    int32_t neighbor = LeftNeighbor(tileid);
    if (visited_tiles.find(neighbor) == visited_tiles.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist.push_back(neighbor);
      visited_tiles.insert(neighbor);
    }
    neighbor = RightNeighbor(tileid);
    if (visited_tiles.find(neighbor) == visited_tiles.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist.push_back(neighbor);
      visited_tiles.insert(neighbor);
    }
    neighbor = TopNeighbor(tileid);
    if (visited_tiles.find(neighbor) == visited_tiles.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist.push_back(neighbor);
      visited_tiles.insert(neighbor);
    }
    neighbor = BottomNeighbor(tileid);
    if (visited_tiles.find(neighbor) == visited_tiles.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist.push_back(neighbor);
      visited_tiles.insert(neighbor);
    }
  }
  return tilelist_;
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

// Explicit instantiation
template class Tiles<Point2>;
template class Tiles<PointLL>;

}
}
