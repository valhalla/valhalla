
#ifndef __tilesll_h_
#define __tilesll_h_

#include <list>
#include <map>

/**
 * A class that provides a uniform (square) tiling system for a specified
 * bounding box (either in x,y or f,lng) and tile size.
 * A unique tile ID is assigned for each tile based on the following rules:
 *    Tile numbers start at 0 at the min y, x (lower left)
 *    Tile numbers increase by column (x,longitude) then by row (y,latitude)
 *    Tile numbers increase along each row by increasing x,longitude.
 * Contains methods for converting x,y or lat,lng into tile ID and
 * vice-versa.  Methods for relative tiles (using row and column offsets).
 * are also provided.
 * @author  David W. Nesbitt
 */
class Tiles {
 public:
  /**
   * Constructor.  A bounding box and tile size is specified.
   * Sets class data members and precalculates the number of rows and columns
   * based on the bounding box and tile size.
   * @param   bounds    Bounding box
   * @param   tilesize  Tile size
   */
  Tiles(const AABB2& bounds, const float tilesize) {
    tilebounds_ = bounds;
    tilesize_   = tilesize;
    ncolumns_   = (int)ceil((bounds.maxx() - bounds.minx()) / tilesize_);
    nrows       = (int)ceil((bounds.maxy() - bounds.miny()) / tilesize_);
  }

  /**
   * Destructor.
   */
  virtual ~Tiles() { }

  /**
  * Get the tile size.
  * @return Tile size.
  */
  float TileSize() const {
    return tilesize_;
  }

  /**
   * Returns the bounding box of the tiling system
   * @return Bounding box.
   */
  AABB2 TileBounds() const {
    return tilebounds_;
  }

  /**
   * Gets the "row" based on y.
   * @param   y   y coordinate
   * @return  Returns the tile row. Returns -1 if outside the
   *          tile system bounds.
   */
  int Row(const float y) const {
    // Return -1 if outside the tile system bounds
    if (y < tilebounds_.miny() || y > tilebounds_.maxy())
      return -1;

    // If equal to the max y return the largest row
    if (y == tilebounds_.maxy())
      return nrows - 1;
    else {
      return (int)((y - tilebounds_.miny()) / tilesize_);
    }
  }

  /**
   * Gets the "column" based on x.
   * @param   x   x coordinate
   * @return  Returns the tile column. Returns -1 if outside the
   *          tile system bounds.
   */
  int Col(const float x) const {
    // Return -1 if outside the tile system bounds
    if (x < tilebounds_.minx() || x > tilebounds_.maxx())
      return -1;

    // If equal to the max x return the largest column
    if (x == tilebounds_.maxx())
      return ncolumns_ - 1;
    else {
      float col = (x - tilebounds_.minx()) / tilesize_;
      return (col >= 0.0) ? (int)col : (int)col - 1;
    }
  }

  /**
   * Converts the center of a bounding box to a tile ID.
   * @param   c   Center point.
   * @return  Returns the tile ID. If the latitude, longitude is outside
   *          the extent, an error (-1) is returned.
   */
  int TileId(const Point2& c) {
    return TileId(c.y(), c.x());
  }

  /**
   * Converts x,y to a tile ID.
   * @param   y   x (or lng)
   * @param   x   y (or lat)
   * @return  Returns the tile ID. -1 (errors( is returned if the x,y is
   *          outside the bounding box of the tiles.
   */
  int TileId(const float y, const float x) {
    // Returns if totally outside the extent.
    if (y < tilebounds_.miny() || x < tilebounds_.minx() ||
        y > tilebounds_.maxy() || x > tilebounds_.maxx())
      return -1;

    // Find the tileid by finding the latitude row and longitude column
    return (Row(y) * ncolumns_) + Col(x);
  }

  /**
   * Gets the tile ID given the row ID and column ID.
   */
  int TileId(const int col, const int row) {
    return (row * ncolumns_) + col;
  }

  /**
   * Get the base x,y of a specified tile.
   * @param   tileid   Tile ID.
   * @return  The base x,y of the specified tile.
   */
  Point2 Base(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return Point2(tilebounds_.miny() + (row * tilesize_),
                  tilebounds_.minx() + (col * tilesize_));
  }

  /**
   * Gets the y,x extent of the specified tile.
   * @param   tileid   Tile ID.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABB2 TileBounds(const int tileid) const {
    Point2 base = Base(tileid);
    return AABB2(base.y(), base.x(), base.y() + tilesize_,
                 base.x() + tilesize_);
  }

  /**
   * Gets the y,x extent of the tile with specified row, column.
   * @param   col   Tile column.
   * @param   row   Tile row.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABB2 TileBounds(const int col, const int row) const {
    float basey = ((float)row * tilesize_) + tilebounds_.miny();
    float basex = ((float)col * tilesize_) + tilebounds_.minx();
    return AABB2(basey, basex, basey + tilesize_, basex + tilesize_);
  }

  /**
   * Gets the center of the specified tile.
   * @param   tileid   Tile ID.
   * @return  The center x,y of the specified tile.
   */
  Point2 Center(const int tileid) const {
    Point2 base = Base(tileid);
    return Point2(base.y() + tilesize_ * 0.5, base.x() + tilesize_ * 0.5);
  }

  /**
   * Returns the new tile given a previous tile and a row, column offset.
   * @param   initial_tile      ID of the tile to offset from.
   * @param   delta_rows    Number of rows to offset (can be negative).
   * @param   delta_cols    Number of columns to offset (can be negative).
   * @return  Tile ID of the new tile.
   */
  int GetRelativeTileId(const int initial_tile, const int delta_rows,
                        const int delta_cols) const {
    return initial_tile + (delta_rows * ncolumns_) + delta_cols;
  }

   /**
    * Returns the tile offsets (row,column) between the previous tile ID and
    * a new tileid.  The offsets are returned through arguments (references).
    * Offsets can be positive or negative or 0.
    * @param   initial_tileid     Original tile.
    * @param   newtileid      Tile to which relative offset is desired.
    * @param   delta_rows    Return: Relative number of rows.
    * @param   delta_cols    Return: Relative number of columns.
    */
   void TileOffsets(const int initial_tileid, const int newtileid,
                       int& delta_rows, int& delta_cols) const
   {
      int deltaTile = newtileid - initial_tileid;
      delta_rows = (newtileid  / ncolumns_) - (initial_tileid / ncolumns_);
      delta_cols = deltaTile - (delta_rows * ncolumns_);
   }

  /**
   * Get the number of tiles in the extent.
   * @return  Number of tiles.
   */
  unsigned int TileCount() const {
    float nrows = (tilebounds_.maxy() - tilebounds_.miny()) /tilesize_;
    return ncolumns_ * (int)ceil(nrows);
  }

  /**
   * Gets the neighboring tileid to the right/east.
   */
  int RightNeighbor(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return (col < ncolumns_ - 1) ?  tileid + 1 : tileid - ncolumns_ + 1;
  }

  /**
   * Gets the neighboring tileid - west.
   */
  int LeftNeighbor(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return (col > 0) ?  tileid - 1 : tileid + ncolumns_ - 1;
  }

  /**
   * Gets the neighboring tileid - north.
   */
  int TopNeighbor(const int tileid) const {
    return (tileid < (int)(TileCount() - ncolumns_)) ?
            tileid + ncolumns_ : tileid;
  }

   /**
    * Gets the neighboring tileid - south.
    */
   int BottomNeighbor(const int tileid) const {
      return (tileid < ncolumns_) ? tileid : tileid - ncolumns_;
   }

  /**
   * Gets the list of tiles that lie within the specified bounding box.
   * The method finds the center tile and spirals out by finding neighbors
   * and recursively checking if tile is inside and checking/adding
   * neighboring tiles
   * @param  boundingbox  Bounding box
   * @param  maxTiles  Maximum number of tiles to find.
   */
  const std::vector<int>& TileList(const AABB2& boundingbox,
               const unsigned int maxtiles = 4096) {
    // Clear lists
    checklist_.clear();
    tilelist_.clear();
    visitedtiles_.clear();

    // Get tile at the center of the boundinb box. Return -1 if the center
    // of the bounding box is not within the tiling system bounding box.
    // TODO - relax this to check edges of the bounding box?
    int tileid = TileId(boundingbox.Center());
    if (tileid == -1)
      return tilelist_;

    // Set this tile in the list and it to the list of visited tiles.
    tilelist_.push_back(tileid);
    visitedtiles_[tileid] = 1;

    // Add neighbors to the "check" list
    addNeighbors(tileid);

    // Get neighboring tiles in bounding box until NextTile returns -1
    // or the maximum number specified is reached
    while ((tileid = NextTile(boundingbox)) >= 0) {
      tilelist_.push_back(tileid);
      if (tilelist_.size() == maxtiles) {
        break;
      }
    }
    return tilelist_;
  }

 protected:
  // Bounding box of the tiling system.
  AABB2 tilebounds_;

  // Tile size.  Tiles are square (equal y and x size).
  float tilesize_;

  // Number of rows ( y or latitude)
  int nrows;

  // Number of longitude (x or longitude).
  int ncolumns_;

  // Tile list being constructed
  std::vector<int> tilelist_;

  // List of tiles to check if in view. Use a list: push new entries on the
  // back and pop off the front. The tile search tends to spiral out from
  // the center.
  std::list<int> checklist_;

  // Visited tiles
  std::map<int, int> visitedtiles_;

  // Default constructor (private).  Forces use of the bounding box
  Tiles() { }

  // This function checks neighboring tiles. It adds these tiles to the
  // end of the CheckList if they are not already in there.
  void addNeighbors(const int tileid) {
    // Make sure we check that the neighbor tile is not equal to the
    // current tile - that happens at the edge of the coverage
    int neighbor = LeftNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = RightNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = TopNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = BottomNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }
  }

  // Returns the next tile from the check list that is inside the bounding
  // box. Adds its neighbors to the check list.
  // Returns the tileId or -1 if no more tiles are inside the bounding box.
  int NextTile(const AABB2& boundingbox) {
    int tileid;
    while (!checklist_.empty()) {
      // Get the element off the front of the list
      tileid = checklist_.front();
      checklist_.pop_front();

      // Check if tile is visible
      if (boundingbox.Intersects(TileBounds(tileid))) {
        // Add neighbors
        addNeighbors(tileid);
        return tileid;
      }
    }
    return -1;
  }

  // Convenience method to check if the tile has already been considered
  // (added to tile list or check list)
  bool InList(const int id) {
    std::map<int, int>::iterator iter = visitedtiles_.find(id);
    return (iter != visitedtiles_.end());
  }
};

#endif
