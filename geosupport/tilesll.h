
#ifndef __tilesll_h_
#define __tilesll_h_

#include <list>
#include <map>

/**
 * A class that provides a uniform tiling system for a specified
 * latitude,longitude bounding box and tile size.
 * A unique tile ID is assigned for each tile based on the following rules:
 *    Tile numbers start at 0 at the min lat, lng (lower left)
 *    Tile numbers increase by column (longitude) then by row (latitude)
 *    Tile numbers increase along each row by increasing longitude.
 * Contains methods for converting latitude, longitude into tile ID and
 * vice-versa.  Methods for relative tiles (using row and column offsets)
 * are also provided.
 * @author  David W. Nesbitt
 */
class TilesLL {
 public:
  /**
   * Constructor.  A latitude, longitude extent and tile size is specified.
   * Sets class data members and precalculates the number of rows and columns
   * based on the longitude extent and tile size.  Tiles must be square.
   * @param   bounds    Latitude, longitude bounding box
   * @param   tilesize  Tile size (in degrees)
   */
  TilesLL(const AABBLL& bounds, const float tilesize) {
    tilebounds_ = bounds;
    tilesize_   = tilesize;
    ncolumns_   = (int)ceil((bounds.maxlng() - bounds.minlng()) / tilesize_);
    nrows       = (int)ceil((bounds.maxlat() - bounds.minlat()) / tilesize_);
  }

  /**
   * Destructor.
   */
  virtual ~TilesLL() { }

  /**
  * Get the tile size in degrees.
  * @return Tile size in degrees.
  */
  float TileSize() const {
    return tilesize_;
  }

  /**
   * Returns the extent of the tiling system
   * @return The latitude,longitude extent (using LatLngExtent).
   */
  AABBLL TileBounds() const {
    return tilebounds_;
  }

  /**
   * Gets the "row" based on latitude.
   * @param   lat   Latitude
   * @return  Returns the latitude row. Returns -1 if outside the
   *          tile system bounds.
   */
  int Row(const float lat) const {
    // Return -1 if outside the tile system bounds
    if (lat < tilebounds_.minlat() || lat > tilebounds_.maxlat())
      return -1;

    // If equal to the max lat return the largest row
    if (lat == tilebounds_.maxlat())
      return nrows - 1;
    else {
      return (int)((lat - tilebounds_.minlat()) / tilesize_);
    }
  }

  /**
   * Gets the "column" based on longitude.
   * @param   lng   Longitude
   * @return  Returns the longitude column. Returns -1 if outside the
   *          tile system bounds.
   */
  int Col(const float lng) const {
    // Return -1 if outside the tile system bounds
    if (lng < tilebounds_.minlng() || lng > tilebounds_.maxlng())
      return -1;

    // If equal to the max lng return the largest column
    if (lng == tilebounds_.maxlng())
      return ncolumns_ - 1;
    else {
      float col = (lng - tilebounds_.minlng()) / tilesize_;
      return (col >= 0.0) ? (int)col : (int)col - 1;
    }
  }

  /**
   * Converts the center of a bounding box to a tile ID.
   * @param   ll   Center point.
   * @return  Returns the tile ID. If the latitude, longitude is outside
   *          the extent, an error (-1) is returned.
   */
  int TileId(const PointLL& ll) {
    return TileId(ll.lat(), ll.lng());
  }

  /**
   * Converts a latitude, longitude to a tile ID.
   * @param   lat   Latitude (degrees)
   * @param   lng   Longitude (degrees)
   * @return  Returns the tile ID. If the latitude, longitude is outside
   *          the extent, an error (-1) is returned.
   */
  int TileId(const float lat, const float lng) {
    // Returns if totally outside the extent.
    if (lat < tilebounds_.minlat() || lng < tilebounds_.minlng() ||
        lat > tilebounds_.maxlat() || lng > tilebounds_.maxlng())
      return -1;

    // Find the tileid by finding the latitude row and longitude column
    return (Row(lat) * ncolumns_) + Col(lng);
  }

  /**
   * Gets the tile ID given the row ID and column ID.
   */
  int TileId(const int col, const int row) {
    return (row * ncolumns_) + col;
  }

  /**
   * Get the base latitude, longitude of a specified tile.
   * @param   tileid   Tile ID.
   * @return  The base latitude, longitude of the specified tile.
   */
  PointLL BaseLatLng(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return PointLL(tilebounds_.minlat() + (row * tilesize_),
                   tilebounds_.minlng() + (col * tilesize_));
  }

  /**
   * Gets the lat,lng extent of the specified tile.
   * @param   tileid   Tile ID.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABBLL TileBounds(const int tileid) const {
    PointLL baseLL = BaseLatLng(tileid);
    return AABBLL(baseLL.lat(), baseLL.lng(),
                  baseLL.lat() + tilesize_, baseLL.lng() + tilesize_);
  }

  /**
   * Gets the lat,lng extent of the tile with specified row, column.
   * @param   col   Tile column.
   * @param   row   Tile row.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABBLL TileBounds(const int col, const int row) const {
    float baseLat = ((float)row * tilesize_) + tilebounds_.minlat();
    float baseLng = ((float)col * tilesize_) + tilebounds_.minlng();
    return AABBLL(baseLat, baseLng,
                  baseLat + tilesize_, baseLng + tilesize_);
  }

  /**
   * Gets the center lat,lng of the specified tile.
   * @param   tileid   Tile ID.
   * @return  The center latitude, longitude of the specified tile.
   */
  PointLL Center(const int tileid) const {
    PointLL baseLL = BaseLatLng(tileid);
    return PointLL(baseLL.lat() + tilesize_ * 0.5,
                   baseLL.lng() + tilesize_ * 0.5);
  }

  /**
   * Returns the new tile given a previous tile and a row, column offset.
   * @param   initial_tile      ID of the tile to offset from.
   * @param   delta_lat_rows    Number of rows to offset (can be negative).
   * @param   delta_lng_cols    Number of columns to offset (can be negative).
   * @return  Tile ID of the new tile.
   */
  int GetRelativeTileId(const int initial_tile, const int delta_lat_rows,
                        const int delta_lng_cols) const {
    return initial_tile + (delta_lat_rows * ncolumns_) + delta_lng_cols;
  }

   /**
    * Returns the tile offsets (row,column) between the previous tile ID and
    * a new tileid.  The offsets are returned through arguments (references).
    * Offsets can be positive or negative or 0.
    * @param   initial_tileid     Original tile.
    * @param   newtileid      Tile to which relative offset is desired.
    * @param   delta_lat_rows    Return: Relative number of rows.
    * @param   delta_lng_cols    Return: Relative number of columns.
    */
   void TileOffsets(const int initial_tileid, const int newtileid,
                       int& delta_lat_rows, int& delta_lng_cols) const
   {
      int deltaTile = newtileid - initial_tileid;
      delta_lat_rows = (newtileid  / ncolumns_) - (initial_tileid / ncolumns_);
      delta_lng_cols = deltaTile - (delta_lat_rows * ncolumns_);
   }

  /**
   * Get the number of tiles in the extent.
   * @return  Number of tiles.
   */
  unsigned int TileCount() const {
    float nrows = (tilebounds_.maxlat() - tilebounds_.minlat()) /tilesize_;
    return ncolumns_ * (int)ceil(nrows);
  }

  /**
   * Gets the neighboring tileid - east.
   */
  int GetEastNeighbor(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return (col < ncolumns_ - 1) ?  tileid + 1 : tileid - ncolumns_ + 1;
  }

  /**
   * Gets the neighboring tileid - west.
   */
  int WestNeighbor(const int tileid) const {
    int row = tileid / ncolumns_;
    int col = tileid - (row * ncolumns_);
    return (col > 0) ?  tileid - 1 : tileid + ncolumns_ - 1;
  }

  /**
   * Gets the neighboring tileid - north.
   */
  int NorthNeighbor(const int tileid) const {
    return (tileid < (int)(TileCount() - ncolumns_)) ?
            tileid + ncolumns_ : tileid;
  }

   /**
    * Gets the neighboring tileid - south.
    */
   int SouthNeighbor(const int tileid) const {
      return (tileid < ncolumns_) ? tileid : tileid - ncolumns_;
   }

  /**
   * Gets the list of tiles required to support the given view. The method
   * finds the center tile and spirals out by finding neighbors and
   * recursively checking tile visibility and adding new tiles
   * @param  view      View description. The view class must support the
   *                   IsTileInView method.
   * @param  maxTiles  Maximum number of tiles to find.
   */
  const std::vector<int>& GetTileList(const AABBLL& boundingbox,
               const unsigned int maxtiles = 4096) {
    // Clear lists
    checklist_.clear();
    tilelist_.clear();
    visitedtiles_.clear();

    // Get tile at the center of the boundinb box. Return -1 if the center
    // of the bounding box is not within the tiling system bounding box.
    // TODO - relax this to check edges of the bounding box?
    // TODO - better way to use derived class?
    Point2 center = boundingbox.Center();
    PointLL ll(center.y(), center.x());
    int tileid = TileId(ll);
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
  AABBLL tilebounds_;

  // Tile size in degrees.  Tiles are square (equal lat and lng size).
  float tilesize_;

  // Number of latitude rows.
  int nrows;

  // Number of longitude columns.
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
  TilesLL() { }

  // This function checks neighboring tiles. It adds these tiles to the
  // end of the CheckList if they are not already in there.
  void addNeighbors(const int tileid) {
    // Make sure we check that the neighbor tile is not equal to the
    // current tile - that happens at the edge of the coverage
    int neighbor = WestNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = GetEastNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = NorthNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }

    neighbor = SouthNeighbor(tileid);
    if (neighbor != tileid && !InList(neighbor)) {
      checklist_.push_back(neighbor);
      visitedtiles_[neighbor] = 1;
    }
  }

  // Returns the next tile from the check list that is inside the bounding
  // box. Adds its neighbors to the check list.
  // Returns the tileId or -1 if no more tiles are inside the bounding box.
  int NextTile(const AABBLL& boundingbox) {
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
