
#ifndef VALHALLA_MIDGARD_TILES_H_
#define VALHALLA_MIDGARD_TILES_H_

#include <list>
#include <unordered_set>
#include <cstdint>

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/aabb2.h>

namespace valhalla {
namespace midgard {

/**
 * A class that provides a uniform (square) tiling system for a specified
 * bounding box (either in x,y or lat,lng) and tile size.
 * A unique tile Id is assigned for each tile based on the following rules:
 *    Tile numbers start at 0 at the min y, x (lower left)
 *    Tile numbers increase by column (x,longitude) then by row (y,latitude)
 *    Tile numbers increase along each row by increasing x,longitude.
 * Contains methods for converting x,y or lat,lng into tile Id and
 * vice-versa.  Methods for relative tiles (using row and column offsets).
 * are also provided.
 */
class Tiles {
 public:
  /**
   * Constructor.  A bounding box and tile size is specified.
   * Sets class data members and computes the number of rows and columns
   * based on the bounding box and tile size.
   * @param   bounds    Bounding box
   * @param   tilesize  Tile size
   */
  Tiles(const AABB2& bounds, const float tilesize);

  /**
   * Get the tile size.
   * @return Tile size.
   */
  float TileSize() const;

  /**
   * Returns the bounding box of the tiling system.
   * @return Bounding box.
   */
  AABB2 TileBounds() const;

  /**
   * Gets the "row" based on y.
   * @param   y   y coordinate
   * @return  Returns the tile row. Returns -1 if outside the
   *          tile system bounds.
   */
  int32_t Row(const float y) const;

  /**
   * Gets the "column" based on x.
   * @param   x   x coordinate
   * @return  Returns the tile column. Returns -1 if outside the
   *          tile system bounds.
   */
  int32_t Col(const float x) const;

  /**
   * Converts the center of a bounding box to a tile Id.
   * @param   c   Center point.
   * @return  Returns the tile Id. If the latitude, longitude is outside
   *          the extent, an error (-1) is returned.
   */
  int32_t TileId(const Point2& c) const;

  /**
   * Converts x,y to a tile Id.
   * @param   y   y (or lat)
   * @param   x   x (or lng)
   * @return  Returns the tile Id. -1 (errors( is returned if the x,y is
   *          outside the bounding box of the tiles.
   */
  int32_t TileId(const float y, const float x) const;

  /**
   * Gets the tile Id given the row Id and column Id.
   * @param  col  Tile column.
   * @param  row  Tile row.
   * @return  Returns the tile Id.
   */
  int32_t TileId(const int32_t col, const int32_t row) const;

  /**
   * Gets a maximum tileid given a bounds and a tile size
   * @param bound       the region for which to compute the maximum tile id
   * @param tile_size   the size of a tile within the region
   * @return the highest tile number within the region
   */
  static uint32_t MaxTileId(const AABB2& bounds, const float tile_size);

  /**
   * Get the base x,y of a specified tile.
   * @param   tileid   Tile Id.
   * @return  The base x,y of the specified tile.
   */
  Point2 Base(const int32_t tileid) const;

  /**
   * Gets the y,x extent of the specified tile.
   * @param   tileid   Tile Id.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABB2 TileBounds(const int32_t tileid) const;

  /**
   * Gets the y,x extent of the tile with specified row, column.
   * @param   col   Tile column.
   * @param   row   Tile row.
   * @return  The latitude, longitude extent of the specified tile.
   */
  AABB2 TileBounds(const int32_t col, const int32_t row) const;

  /**
   * Get the tile area in square kilometers.
   * @return  Returns the tile area in kilometers squared.
   */
  float Area(const int32_t tileid) const;

  /**
   * Gets the center of the specified tile.
   * @param   tileid   Tile Id.
   * @return  The center x,y of the specified tile.
   */
  Point2 Center(const int32_t tileid) const;

  /**
   * Returns the new tile given a previous tile and a row, column offset.
   * @param   initial_tile      Id of the tile to offset from.
   * @param   delta_rows    Number of rows to offset (can be negative).
   * @param   delta_cols    Number of columns to offset (can be negative).
   * @return  Tile Id of the new tile.
   */
  int32_t GetRelativeTileId(const int32_t initial_tile, const int32_t delta_rows,
                        const int32_t delta_cols) const;

   /**
    * Returns the tile offsets (row,column) between the previous tile Id and
    * a new tileid.  The offsets are returned through arguments (references).
    * Offsets can be positive or negative or 0.
    * @param   initial_tileid     Original tile.
    * @param   newtileid      Tile to which relative offset is desired.
    * @param   delta_rows    Return: Relative number of rows.
    * @param   delta_cols    Return: Relative number of columns.
    */
   void TileOffsets(const int32_t initial_tileid, const int32_t newtileid,
                       int32_t& delta_rows, int32_t& delta_cols) const;

  /**
   * Get the number of tiles in the extent.
   * @return  Number of tiles.
   */
  uint32_t TileCount() const;

  /**
   * Gets the neighboring tileid to the right/east.
   * @param  tileid   Tile Id.
   * @return  Returns the tile Id of the tile to the right/east.
   */
  int32_t RightNeighbor(const int32_t tileid) const;

  /**
   * Gets the neighboring tileid - west.
   * @param  tileid   Tile Id.
   * @return  Returns the tile Id of the tile to the left/west.
   */
  int32_t LeftNeighbor(const int32_t tileid) const;

  /**
   * Gets the neighboring tileid - north.
   * @param  tileid   Tile Id.
   * @return  Returns the tile Id of the tile to the north. Return tileid
   *          if tile Id is on the top row (no neighbor to the north).
   */
  int32_t TopNeighbor(const int32_t tileid) const;

  /**
   * Gets the neighboring tileid - south.
   * @param  tileid   Tile Id.
   * @return  Returns the tile Id of the tile to the south. Return tileid
   *          if tile Id is on the bottom row (no neighbor to the south).
   */
  int32_t BottomNeighbor(const int32_t tileid) const;

  /**
   * Gets the list of tiles that lie within the specified bounding box.
   * The method finds the center tile and spirals out by finding neighbors
   * and recursively checking if tile is inside and checking/adding
   * neighboring tiles
   * @param  boundingbox  Bounding box
   * @param  maxTiles  Maximum number of tiles to find.
   */
  const std::vector<int32_t>& TileList(const AABB2& boundingbox);

  /**
   * Check if a path from the origin tile to the destination tile exists
   * given a map of which tiles are populated.
   * @param  tilemap  Vector of bool for each tile Id where each true value
   *                  indicates the tile is populated.
   * @param  origin_tile Tile Id of the origin.
   * @param  dest_tile   Tile Id of the destination.
   * @return  Returns true if there is a path through existing tiles from the
   *          origin to the destination tile.
   */
  bool PathExists(const std::vector<bool>& tilemap, const uint32_t origin_tile,
                  const uint32_t dest_tile) const;

  /**
   * Generate a "connectivity map" given the map of existing tiles. Any 2 tiles
   * that have a connected path between them will have the same value in the
   * connectivity map.
   * @param  tilemap  Vector of bool for each tile Id where each true value
   *                  indicates the tile is populated.
   * @return  Returns a vector with a value for each tile Id. Values of 0 are
   *          empty tiles (false in the input tilemap). Non-zero values that
   *          are equal have a path through valid tiles.
   */
  std::vector<uint32_t> ConnectivityMap(const std::vector<bool>& tilemap) const;

 protected:
  // Bounding box of the tiling system.
  AABB2 tilebounds_;

  // Tile size.  Tiles are square (equal y and x size).
  float tilesize_;

  // Number of rows ( y or latitude)
  int32_t nrows_;

  // Number of longitude (x or longitude).
  int32_t ncolumns_;

  // Tile list being constructed
  std::vector<int32_t> tilelist_;

  // List of tiles to check if in view. Use a list: push new entries on the
  // back and pop off the front. The tile search tends to spiral out from
  // the center.
  std::list<int32_t> checklist_;

  // Visited tiles
  std::unordered_set<int32_t> visitedtiles_;
};

}
}

#endif  // VALHALLA_MIDGARD_TILES_H_
