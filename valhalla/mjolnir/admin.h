#ifndef VALHALLA_MJOLNIR_ADMIN_H_
#define VALHALLA_MJOLNIR_ADMIN_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <cstdint>
#include <sqlite3.h>
#include <unordered_map>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <valhalla/mjolnir/graphtilebuilder.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

// Geometry types for admin queries
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;

/**
 * Get the dbhandle of a sqlite db.  Used for timezones and admins DBs.
 * @param  database   db file location.
 */
sqlite3* GetDBHandle(const std::string& database);

/**
 * Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
 * poly.
 * @param  polys   unordered map of polys.
 * @param  ll      point that needs to be checked.
 */
uint32_t GetMultiPolyId(const std::unordered_map<uint32_t, multi_polygon_type>& polys,
                        const PointLL& ll);

/**
 * Get the timezone polys from the db
 * @param  db_handle    sqlite3 db handle
 * @param  aabb         bb of the tile
 */
std::unordered_map<uint32_t, multi_polygon_type> GetTimeZones(sqlite3* db_handle,
                                                              const AABB2<PointLL>& aabb);

/**
 * Get the admin polys that intersect with the tile bounding box.
 * @param  db_handle        sqlite3 db handle
 * @param  drive_on_right   unordered map that indicates if a country drives on right side of the
 * road
 * @param  aabb             bb of the tile
 * @param  tilebuilder      Graph tile builder
 */
std::unordered_map<uint32_t, multi_polygon_type>
GetAdminInfo(sqlite3* db_handle,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder);

/**
 * Get all the country access records from the db and save them to a map.
 * @param  db_handle    sqlite3 db handle
 */
std::unordered_map<std::string, std::vector<int>> GetCountryAccess(sqlite3* db_handle);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_ADMIN_H_
