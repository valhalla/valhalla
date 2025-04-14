#ifndef VALHALLA_MJOLNIR_ADMIN_H_
#define VALHALLA_MJOLNIR_ADMIN_H_

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/mjolnir/sqlite3.h>

#include <geos_c.h>

#include <cstdint>
#include <unordered_map>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

typedef std::shared_ptr<GEOSContextHandle_HS> geos_context_type;
inline geos_context_type NewGEOSContext() {
  return geos_context_type(GEOS_init_r(), GEOS_finish_r);
}

// Helper struct for simple wrapping of GEOS types by `std::unique_ptr`
struct GEOSDeleter {
  geos_context_type context;

  GEOSDeleter(geos_context_type ctx) : context(std::move(ctx)) {
  }

  void operator()(GEOSGeometry* p) const {
    GEOSGeom_destroy_r(context.get(), p);
  };
  void operator()(GEOSWKBReader* p) const {
    GEOSWKBReader_destroy_r(context.get(), p);
  };
};

typedef std::unique_ptr<GEOSGeometry, GEOSDeleter> geometry_type;
typedef std::vector<std::tuple<geometry_type, std::vector<std::string>, bool>> language_poly_index;

/**
 * Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
 * poly.
 * @param  polys      unordered map of polys.
 * @param  ll         point that needs to be checked.
 * @param  graphtile  graphtilebuilder that is used to determine if we are a country poly or not.
 */
uint32_t GetMultiPolyId(const std::multimap<uint32_t, geometry_type>& polys,
                        geos_context_type context,
                        const PointLL& ll,
                        GraphTileBuilder& graphtile);

/**
 * Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
 * poly.
 * @param  polys      unordered map of polys.
 * @param  ll         point that needs to be checked.
 */
uint32_t GetMultiPolyId(const std::multimap<uint32_t, geometry_type>& polys,
                        geos_context_type context,
                        const PointLL& ll);

/**
 * Get the vector of languages for this LL.  Used by admin areas.  Checks if the pointLL is covered_by
 * the poly.
 * @param  language_polys      tuple that contains a language, poly, is_default_language.
 * @param  ll         point that needs to be checked.
 * @return  Returns the vector of pairs {language, is_default_language}
 */
std::vector<std::pair<std::string, bool>>
GetMultiPolyIndexes(const language_poly_index& language_ploys,
                    geos_context_type context,
                    const PointLL& ll);

/**
 * Get the timezone polys from the db
 * @param  db           sqlite3 db handle
 * @param  aabb         bb of the tile
 */
std::multimap<uint32_t, geometry_type>
GetTimeZones(Sqlite3& db, geos_context_type context, const AABB2<PointLL>& aabb);

/**
 * Get the admin data from the spatialite db given an SQL statement
 * @param  db               sqlite3 db handle
 * @param  stmt             prepared statement object
 * @param  sql              sql commend to run.
 * @param  tilebuilder      Graph tile builder
 * @param  polys            unordered multimap of admin polys
 * @param  drive_on_right   unordered map that indicates if a country drives on right side of the
 * road
 * @param  default_languages ordered map that is used for lower admins that have an
 * default language set
 * @param  language_polys    ordered map that is used for lower admins that have an
 * default language set
 * @param  languages_only    should we only process the languages with this query
 */
void GetData(Sqlite3& db,
             sqlite3_stmt* stmt,
             const std::string& sql,
             geos_context_type context,
             GraphTileBuilder& tilebuilder,
             std::multimap<uint32_t, geometry_type>& polys,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             language_poly_index& language_polys,
             bool languages_only);

/**
 * Get the admin polys that intersect with the tile bounding box.
 * @param  db               sqlite3 db handle
 * @param  drive_on_right   unordered map that indicates if a country drives on right side of the
 * road
 * @param  allow_intersection_names   unordered map that indicates if we call out intersections
 * names for this country
 * @param  default_languages ordered map that is used for lower admins that have an
 * default language set
 * @param  language_polys    ordered map that is used for lower admins that have an
 * default language set
 * @param  aabb              bb of the tile
 * @param  tilebuilder       Graph tile builder
 */
std::multimap<uint32_t, geometry_type>
GetAdminInfo(Sqlite3& db,
             geos_context_type context,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_polys,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder);

/**
 * Get all the country access records from the db and save them to a map.
 * @param  db    sqlite3 db handle
 */
std::unordered_map<std::string, std::vector<int>> GetCountryAccess(Sqlite3& db);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_ADMIN_H_
