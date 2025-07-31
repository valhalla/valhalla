#ifndef VALHALLA_MJOLNIR_ADMIN_H_
#define VALHALLA_MJOLNIR_ADMIN_H_

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/mjolnir/sqlite3.h>

#include <cstdint>
#include <unordered_map>

struct GEOSContextHandle_HS;
struct GEOSGeom_t;
struct GEOSPrepGeom_t;
struct GEOSWKBReader_t;

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

// GEOS thread-safe API requires a context handle for each operation, that should be unique for every
// thread. See https://libgeos.org/usage/c_api/#reentrantthreadsafe-api for more details.
typedef std::shared_ptr<GEOSContextHandle_HS> geos_context_type;

// RAII wrapper for GEOSGeometry
struct Geometry {
  geos_context_type context;
  GEOSGeom_t* geometry;
  const GEOSPrepGeom_t* prepared;

  Geometry(geos_context_type ctx, GEOSGeom_t* geom);
  ~Geometry();

  // This class cannot be copied, but can be moved
  Geometry(Geometry const&) = delete;
  Geometry& operator=(Geometry const&) = delete;
  Geometry(Geometry&& other) noexcept
      : context(std::move(other.context)), geometry(other.geometry), prepared(other.prepared) {
    other.geometry = nullptr;
    other.prepared = nullptr;
  }
  Geometry& operator=(Geometry&& other) noexcept {
    // move and swap idiom via local variable
    Geometry local = std::move(other);
    std::swap(geometry, local.geometry);
    std::swap(context, local.context);
    return *this;
  }

  // Returns true if the geometry intersects the given point
  bool intersects(const PointLL& ll) const;
  // Creates a clone of the current geometry
  Geometry clone() const;
};

typedef std::vector<std::tuple<Geometry, std::vector<std::string>, bool>> language_poly_index;

class AdminDB {
  Sqlite3 db;
  geos_context_type geos_context;
  GEOSWKBReader_t* wkb_reader;

  // Constructor is private, use `AdminDB::open()` instead.
  AdminDB(Sqlite3&& db);

public:
  // Tries to open an AdminDB from the given path. Returns std::nullopt if failed.
  static std::optional<AdminDB> open(const std::string& path);
  ~AdminDB();

  // This class cannot be copied, but can be moved
  AdminDB(AdminDB const&) = delete;
  AdminDB& operator=(AdminDB const&) = delete;
  AdminDB(AdminDB&& other) noexcept
      : db(std::move(other.db)), geos_context(std::move(other.geos_context)),
        wkb_reader(other.wkb_reader) {
    other.wkb_reader = nullptr;
  }
  AdminDB& operator=(AdminDB&& other) noexcept {
    // move and swap idiom via local variable
    AdminDB local = std::move(other);
    std::swap(db, local.db);
    std::swap(geos_context, local.geos_context);
    std::swap(wkb_reader, local.wkb_reader);
    return *this;
  }

  sqlite3* get() {
    return db.get();
  }

  // Reads a WKB blob and clips it to the given bounding box.
  // These two operations are combined into a single function to simplify amount of abstractions
  // and leave `Geometry` always deal with GEOSPreparedGeometry without intermediate entity.
  Geometry read_wkb_and_clip(const unsigned char* wkb_blob, int wkb_size, const AABB2<PointLL>& bbox);
};

/**
 * Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
 * poly.
 * @param  polys      unordered map of polys.
 * @param  ll         point that needs to be checked.
 * @param  graphtile  graphtilebuilder that is used to determine if we are a country poly or not.
 */
uint32_t GetMultiPolyId(const std::multimap<uint32_t, Geometry>& polys,
                        const PointLL& ll,
                        GraphTileBuilder& graphtile);

/**
 * Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
 * poly.
 * @param  polys      unordered map of polys.
 * @param  ll         point that needs to be checked.
 */
uint32_t GetMultiPolyId(const std::multimap<uint32_t, Geometry>& polys, const PointLL& ll);

/**
 * Get the vector of languages for this LL.  Used by admin areas.  Checks if the pointLL is covered_by
 * the poly.
 * @param  language_polys      tuple that contains a language, poly, is_default_language.
 * @param  ll         point that needs to be checked.
 * @return  Returns the vector of pairs {language, is_default_language}
 */
std::vector<std::pair<std::string, bool>>
GetMultiPolyIndexes(const language_poly_index& language_ploys, const PointLL& ll);

/**
 * Get the timezone polys from the db
 * @param  db           sqlite3 db handle
 * @param  aabb         bb of the tile
 */
std::multimap<uint32_t, Geometry> GetTimeZones(AdminDB& db, const AABB2<PointLL>& aabb);

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
std::multimap<uint32_t, Geometry>
GetAdminInfo(AdminDB& db,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_polys,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder);

/**
 * Get all the country access records from the db and save them to a map.
 * @param  db    sqlite3 db handle
 */
std::unordered_map<std::string, std::vector<int>> GetCountryAccess(AdminDB& db);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_ADMIN_H_
