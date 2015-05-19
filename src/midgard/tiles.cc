#include "midgard/tiles.h"
#include "midgard/distanceapproximator.h"
#include <cmath>

namespace valhalla {
namespace midgard {

Tiles::Tiles(const AABB2& bounds, const float tilesize) {
  tilebounds_ = bounds;
  tilesize_ = tilesize;
  ncolumns_ = static_cast<int32_t>(ceil((bounds.maxx() - bounds.minx()) /
                                        tilesize_));
  nrows_    = static_cast<int32_t>(ceil((bounds.maxy() - bounds.miny()) /
                                        tilesize_));
}

float Tiles::TileSize() const {
  return tilesize_;
}

AABB2 Tiles::TileBounds() const {
  return tilebounds_;
}

// Get the number of rows in the tiling system.
int32_t Tiles::nrows() const {
  return nrows_;
}

// Get the number of columns in the tiling system.
int32_t Tiles::ncolumns() const {
  return ncolumns_;
}

int32_t Tiles::Row(const float y) const {
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

int32_t Tiles::Col(const float x) const {
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

int32_t Tiles::TileId(const Point2& c) const {
  return TileId(c.y(), c.x());
}

int32_t Tiles::TileId(const float y, const float x) const {
  // Return -1 if totally outside the extent.
  if (y < tilebounds_.miny() || x < tilebounds_.minx() ||
      y > tilebounds_.maxy() || x > tilebounds_.maxx())
    return -1;

  // Find the tileid by finding the latitude row and longitude column
  return (Row(y) * ncolumns_) + Col(x);
}

int32_t Tiles::TileId(const int32_t col, const int32_t row) const {
  return (row * ncolumns_) + col;
}

// Get the tile row, col based on tile Id.
std::pair<int32_t, int32_t> Tiles::GetRowColumn(const int32_t tileid) const {
  return { tileid / ncolumns_, tileid % ncolumns_ };
}

uint32_t Tiles::MaxTileId(const AABB2& bounds, const float tile_size) {
  uint32_t cols = static_cast<uint32_t>(std::ceil(bounds.Width() / tile_size));
  uint32_t rows = static_cast<uint32_t>(std::ceil(bounds.Height() / tile_size));
  return (cols * rows) - 1;
}

Point2 Tiles::Base(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  return Point2(tilebounds_.miny() + (row * tilesize_),
                tilebounds_.minx() + (col * tilesize_));
}

AABB2 Tiles::TileBounds(const int32_t tileid) const {
  Point2 base = Base(tileid);
  return AABB2(base.y(), base.x(), base.y() + tilesize_, base.x() + tilesize_);
}

AABB2 Tiles::TileBounds(const int32_t col, const int32_t row) const {
  float basey = ((float) row * tilesize_) + tilebounds_.miny();
  float basex = ((float) col * tilesize_) + tilebounds_.minx();
  return AABB2(basey, basex, basey + tilesize_, basex + tilesize_);
}

// Get the tile area in square kilometers.
float Tiles::Area(const int32_t tileid) const {
  AABB2 bb = TileBounds(tileid);
  return ((bb.maxy() - bb.miny()) * kMetersPerDegreeLat * kKmPerMeter) *
         ((bb.maxx() - bb.minx()) *
             DistanceApproximator::MetersPerLngDegree(bb.Center().y()) * kKmPerMeter);
}

Point2 Tiles::Center(const int32_t tileid) const {
  Point2 base = Base(tileid);
  return Point2(base.y() + tilesize_ * 0.5, base.x() + tilesize_ * 0.5);
}

int32_t Tiles::GetRelativeTileId(const int32_t initial_tile, const int32_t delta_rows,
                             const int32_t delta_cols) const {
  return initial_tile + (delta_rows * ncolumns_) + delta_cols;
}

void Tiles::TileOffsets(const int32_t initial_tileid, const int32_t newtileid,
                        int& delta_rows, int& delta_cols) const {
  int32_t deltaTile = newtileid - initial_tileid;
  delta_rows = (newtileid / ncolumns_) - (initial_tileid / ncolumns_);
  delta_cols = deltaTile - (delta_rows * ncolumns_);
}

uint32_t Tiles::TileCount() const {
  float nrows = (tilebounds_.maxy() - tilebounds_.miny()) / tilesize_;
  return ncolumns_ * static_cast<int32_t>(ceil(nrows));
}

int32_t Tiles::RightNeighbor(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  return (col < ncolumns_ - 1) ? tileid + 1 : tileid - ncolumns_ + 1;
}

int32_t Tiles::LeftNeighbor(const int32_t tileid) const {
  int32_t row = tileid / ncolumns_;
  int32_t col = tileid - (row * ncolumns_);
  return (col > 0) ? tileid - 1 : tileid + ncolumns_ - 1;
}

int32_t Tiles::TopNeighbor(const int32_t tileid) const {
  return (tileid < static_cast<int32_t>((TileCount() - ncolumns_))) ?
              tileid + ncolumns_ : tileid;
}

int32_t Tiles::BottomNeighbor(const int32_t tileid) const {
  return (tileid < ncolumns_) ? tileid : tileid - ncolumns_;
}

// Checks if 2 tiles are neighbors (N,E,S,W).
bool Tiles::AreNeighbors(const uint32_t id1, const uint32_t id2) const {
  return (id2 == TopNeighbor(id1) ||
          id2 == RightNeighbor(id1) ||
          id2 == BottomNeighbor(id1) ||
          id2 == LeftNeighbor(id1));
}

const std::vector<int>& Tiles::TileList(const AABB2& boundingbox) {
  // Clear lists
  checklist_.clear();
  tilelist_.clear();
  visitedtiles_.clear();

  // Get tile at the center of the bounding box. Return -1 if the center
  // of the bounding box is not within the tiling system bounding box.
  // TODO - relax this to check edges of the bounding box?
  int32_t tileid = TileId(boundingbox.Center());
  if (tileid == -1)
    return tilelist_;

  // Set this tile in the checklist and it to the list of visited tiles.
  checklist_.push_back(tileid);
  visitedtiles_.insert(tileid);

  // Get neighboring tiles in bounding box until NextTile returns -1
  // or the maximum number specified is reached
  while (!checklist_.empty()) {
    // Get the element off the front of the list and add it to the tile list.
    tileid = checklist_.front();
    checklist_.pop_front();
    tilelist_.push_back(tileid);

    // Check neighbors
    int32_t neighbor = LeftNeighbor(tileid);
    if (visitedtiles_.find(neighbor) == visitedtiles_.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist_.push_back(neighbor);
      visitedtiles_.insert(neighbor);
    }
    neighbor = RightNeighbor(tileid);
    if (visitedtiles_.find(neighbor) == visitedtiles_.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist_.push_back(neighbor);
      visitedtiles_.insert(neighbor);
    }
    neighbor = TopNeighbor(tileid);
    if (visitedtiles_.find(neighbor) == visitedtiles_.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist_.push_back(neighbor);
      visitedtiles_.insert(neighbor);
    }
    neighbor = BottomNeighbor(tileid);
    if (visitedtiles_.find(neighbor) == visitedtiles_.end() &&
        boundingbox.Intersects(TileBounds(neighbor))) {
      checklist_.push_back(neighbor);
      visitedtiles_.insert(neighbor);
    }
  }
  return tilelist_;
}

void Tiles::ColorMap(std::unordered_map<uint32_t, size_t>& connectivity_map) const {
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

}
}
