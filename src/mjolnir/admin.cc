#include "mjolnir/admin.h"
#include "baldr/datetime.h"
#include "mjolnir/util.h"

#include <geos_c.h>
#include <sqlite3.h>

#include <unordered_map>

namespace valhalla {
namespace mjolnir {

namespace {

// Tiles might have nodes slightly off the tile boundary. As for performance optimization the returned
// geometry is clipped, a small buffer around should be added to handle edge cases.
constexpr double kTileBboxBuffer = 1e-3;

} // namespace

Geometry::Geometry(geos_context_type ctx, GEOSGeometry* geom)
    : context(std::move(ctx)), geometry(geom) {
  prepared = GEOSPrepare_r(context.get(), geometry);
}

Geometry::~Geometry() {
  GEOSPreparedGeom_destroy_r(context.get(), prepared);
  GEOSGeom_destroy_r(context.get(), geometry);
}

bool Geometry::intersects(const PointLL& ll) const {
#if GEOS_VERSION_MINOR < 12
  auto* p = GEOSGeom_createPointFromXY_r(context.get(), ll.lng(), ll.lat());
  bool intersects = GEOSPreparedIntersects_r(context.get(), prepared, p);
  GEOSGeom_destroy_r(context.get(), p);
  return intersects;
#else
  return GEOSPreparedIntersectsXY_r(context.get(), prepared, ll.lng(), ll.lat());
#endif
}

Geometry Geometry::clone() const {
  return Geometry(context, GEOSGeom_clone_r(context.get(), geometry));
}

AdminDB::AdminDB(Sqlite3&& sqlite3)
    : db(std::move(sqlite3)), geos_context(geos_context_type(GEOS_init_r(), GEOS_finish_r)) {
  wkb_reader = GEOSWKBReader_create_r(geos_context.get());
}

AdminDB::~AdminDB() {
  GEOSWKBReader_destroy_r(geos_context.get(), wkb_reader);
}

std::optional<AdminDB> AdminDB::open(const std::string& path) {
  auto db = Sqlite3::open(path, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX);
  if (db) {
    return AdminDB(std::move(*db));
  }
  return {};
}

Geometry
AdminDB::read_wkb_and_clip(const unsigned char* wkb_blob, int wkb_size, const AABB2<PointLL>& bbox) {
  GEOSGeometry* geom = GEOSWKBReader_read_r(geos_context.get(), wkb_reader, wkb_blob, wkb_size);
  GEOSGeometry* clipped =
      GEOSClipByRect_r(geos_context.get(), geom, bbox.minx(), bbox.miny(), bbox.maxx(), bbox.maxy());
  GEOSGeom_destroy_r(geos_context.get(), geom);
  return Geometry(geos_context, clipped);
}

// Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
// poly.
uint32_t GetMultiPolyId(const std::multimap<uint32_t, Geometry>& polys,
                        const PointLL& ll,
                        GraphTileBuilder& graphtile) {
  uint32_t index = 0;
  for (const auto& poly : polys) {
    if (poly.second.intersects(ll)) {
      const auto& admin = graphtile.admins_builder(poly.first);
      if (!admin.state_offset())
        index = poly.first;
      else
        return poly.first;
    }
  }
  return index;
}

// Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
// poly.
uint32_t GetMultiPolyId(const std::multimap<uint32_t, Geometry>& polys, const PointLL& ll) {
  for (const auto& [index, poly] : polys) {
    if (poly.intersects(ll)) {
      return index;
    }
  }
  return 0; // default index
}

// This function returns a vector pairs.  The pair is a string and boolean {language,
// is_default_language}. The function takes a LL and checks if it is covered by a linguistic,
// state/providence, and country polygon. If the LL is covered by the polygon, then the language is
// added to the vector and the flag is set for whether this is a default or supported language.  The
// default language is the default_language key that is set by users for the administrative relations.
// Supported are values we can add under in the adminconstants.h.  This vector of pairs is our
// languages that will be considered for any name* or destination* keys.  Basically, we only support
// the languages that are on the signs in that area. Note:  The first pair always contains an empty
// language which makes the name key with no language the most important key.
std::vector<std::pair<std::string, bool>> GetMultiPolyIndexes(const language_poly_index& polys,
                                                              const PointLL& ll) {
  std::vector<std::pair<std::string, bool>> languages;

  // first entry is blank for the default name
  languages.emplace_back("", false);

  for (const auto& [poly, langs, is_default] : polys) {
    if (poly.intersects(ll)) {
      for (const auto& l : langs) {
        if (stringLanguage(l) != Language::kNone) {
          auto needle =
              std::find_if(languages.begin(), languages.end(),
                           [&l](const std::pair<std::string, bool>& p) { return p.first == l; });

          if (needle == languages.end()) {
            languages.emplace_back(l, is_default);
          } else if (is_default) { // fr - nl or fr;en in default lang column
            needle->second = false;
          }
        }
      }
    }
  }

  return languages;
}

// Get the timezone polys from the db
std::multimap<uint32_t, Geometry> GetTimeZones(AdminDB& db, const AABB2<PointLL>& aabb) {
  const AABB2<PointLL> bbox(aabb.minx() - kTileBboxBuffer, aabb.miny() - kTileBboxBuffer,
                            aabb.maxx() + kTileBboxBuffer, aabb.maxy() + kTileBboxBuffer);
  std::multimap<uint32_t, Geometry> polys;
  sqlite3_stmt* stmt = 0;
  uint32_t ret;
  uint32_t result = 0;

  std::string sql = "select TZID, ST_AsBinary(geom) as geom_text from tz_world where ";
  sql += "ST_Intersects(geom, BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'tz_world' AND search_frame = BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + "));";

  ret = sqlite3_prepare_v2(db.get(), sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    result = sqlite3_step(stmt);

    while (result == SQLITE_ROW) {
      std::string tz_id;
      const unsigned char* wkb_blob = nullptr;
      int wkb_size = 0;

      if (sqlite3_column_type(stmt, 0) == SQLITE_TEXT) {
        tz_id = (char*)sqlite3_column_text(stmt, 0);
      }
      if (sqlite3_column_type(stmt, 1) == SQLITE_BLOB) {
        wkb_blob = static_cast<const unsigned char*>(sqlite3_column_blob(stmt, 1));
        wkb_size = sqlite3_column_bytes(stmt, 1);
      }

      uint32_t idx = DateTime::get_tz_db().to_index(tz_id);
      if (idx == 0) {
        sqlite3_finalize(stmt);
        throw std::runtime_error("Can't find timezone ID " + std::string(tz_id));
      }

      auto geom = db.read_wkb_and_clip(wkb_blob, wkb_size, bbox);
      polys.emplace(idx, std::move(geom));
      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return polys;
}

/***
 * Parses a language tag into a vector of individual tokens.
 */
std::vector<std::string> ParseLanguageTokens(const std::string& lang_tag) {
  auto langs = GetTagTokens(lang_tag, " - ");
  if (langs.size() == 1) {
    langs = GetTagTokens(langs.at(0));
  }

  return langs;
}

// Get the admin data from the spatialite db given an SQL statement
void GetData(AdminDB& db,
             sqlite3_stmt* stmt,
             const std::string& sql,
             const AABB2<PointLL>& bbox,
             GraphTileBuilder& tilebuilder,
             std::multimap<uint32_t, Geometry>& polys,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_polys,
             bool languages_only = false) {
  uint32_t result = 0;
  bool dor = true;
  bool intersection_name = false;
  uint32_t ret = sqlite3_prepare_v2(db.get(), sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK || ret == SQLITE_ERROR) {
    result = sqlite3_step(stmt);

    if (result == SQLITE_DONE) {
      sqlite3_finalize(stmt);
      stmt = 0;
      return;
    }
  }

  while (result == SQLITE_ROW) {

    if (!languages_only) {

      std::string country_name, state_name, country_iso, state_iso, supported_languages,
          default_language;

      if (sqlite3_column_type(stmt, 0) == SQLITE_TEXT) {
        country_name = (char*)sqlite3_column_text(stmt, 0);
      }

      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT) {
        state_name = (char*)sqlite3_column_text(stmt, 1);
      }

      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) {
        country_iso = (char*)sqlite3_column_text(stmt, 2);
      }

      if (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) {
        state_iso = (char*)sqlite3_column_text(stmt, 3);
      }

      dor = true;
      if (sqlite3_column_type(stmt, 4) == SQLITE_INTEGER) {
        dor = sqlite3_column_int(stmt, 4);
      }

      intersection_name = false;
      if (sqlite3_column_type(stmt, 5) == SQLITE_INTEGER) {
        intersection_name = sqlite3_column_int(stmt, 5);
      }

      if (sqlite3_column_type(stmt, 7) == SQLITE_TEXT) {
        supported_languages = (char*)sqlite3_column_text(stmt, 7);
      }

      if (sqlite3_column_type(stmt, 8) == SQLITE_TEXT) {
        default_language = (char*)sqlite3_column_text(stmt, 8);
      }

      const unsigned char* wkb_blob = nullptr;
      int wkb_size = 0;
      if (sqlite3_column_type(stmt, 9) == SQLITE_BLOB) {
        wkb_blob = static_cast<const unsigned char*>(sqlite3_column_blob(stmt, 9));
        wkb_size = sqlite3_column_bytes(stmt, 9);
      }

      uint32_t index = tilebuilder.AddAdmin(country_name, state_name, country_iso, state_iso);

      auto geom = db.read_wkb_and_clip(wkb_blob, wkb_size, bbox);

      if (!default_language.empty()) {
        auto langs = ParseLanguageTokens(default_language);
        language_polys.push_back(std::make_tuple(geom.clone(), std::move(langs), true));
      }
      if (!supported_languages.empty()) {
        auto langs = ParseLanguageTokens(supported_languages);
        language_polys.push_back(std::make_tuple(geom.clone(), std::move(langs), false));
      }

      polys.emplace(index, std::move(geom));
      drive_on_right.emplace(index, dor);
      allow_intersection_names.emplace(index, intersection_name);

    } else {

      std::string supported_languages;
      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT) {
        supported_languages = (char*)sqlite3_column_text(stmt, 1);
      }

      std::string default_language;
      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) {
        default_language = (char*)sqlite3_column_text(stmt, 2);
      }

      const unsigned char* wkb_blob = nullptr;
      int wkb_size = 0;
      if (sqlite3_column_type(stmt, 3) == SQLITE_BLOB) {
        wkb_blob = static_cast<const unsigned char*>(sqlite3_column_blob(stmt, 3));
        wkb_size = sqlite3_column_bytes(stmt, 3);
      }

      auto geom = db.read_wkb_and_clip(wkb_blob, wkb_size, bbox);

      if (!default_language.empty()) {
        auto langs = ParseLanguageTokens(default_language);
        language_polys.push_back(std::make_tuple(geom.clone(), std::move(langs), true));
      }
      if (!supported_languages.empty()) {
        auto langs = ParseLanguageTokens(supported_languages);
        language_polys.push_back(std::make_tuple(std::move(geom), std::move(langs), false));
      }
    }

    result = sqlite3_step(stmt);
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
}

// Get the admin polys that intersect with the tile bounding box.
std::multimap<uint32_t, Geometry>
GetAdminInfo(AdminDB& db,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_polys,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder) {
  const AABB2<PointLL> bbox(aabb.minx() - kTileBboxBuffer, aabb.miny() - kTileBboxBuffer,
                            aabb.maxx() + kTileBboxBuffer, aabb.maxy() + kTileBboxBuffer);

  std::multimap<uint32_t, Geometry> polys;
  sqlite3_stmt* stmt = 0;

  // default language query
  std::string sql =
      "SELECT admin_level, supported_languages, default_language, ST_AsBinary(geom) from ";
  sql +=
      " admins where (supported_languages is NOT NULL or default_language is NOT NULL) and ST_Intersects(geom, BuildMBR(" +
      std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) and admin_level>4 ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) order by admin_level desc, name;";
  GetData(db, stmt, sql, bbox, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys, true);

  // state query
  sql = "SELECT country.name, state.name, country.iso_code, ";
  sql += "state.iso_code, state.drive_on_right, state.allow_intersection_names, state.admin_level, ";
  sql +=
      "state.supported_languages, state.default_language, ST_AsBinary(state.geom) from admins state, admins country where ";
  sql += "ST_Intersects(state.geom, BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) and ";
  sql += "country.rowid = state.parent_admin and state.admin_level=4 ";
  sql += "and state.rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) order by state.name, country.name;";
  GetData(db, stmt, sql, bbox, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys);

  // country query
  sql = "SELECT name, \"\", iso_code, \"\", drive_on_right, allow_intersection_names, admin_level, ";
  sql +=
      "supported_languages, default_language, ST_AsBinary(geom) from  admins where ST_Intersects(geom, BuildMBR(" +
      std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) and admin_level=2 ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(bbox.minx()) + ",";
  sql += std::to_string(bbox.miny()) + ", " + std::to_string(bbox.maxx()) + ",";
  sql += std::to_string(bbox.maxy()) + ")) order by name;";
  GetData(db, stmt, sql, bbox, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys);

  if (stmt) { // just in case something bad happened.
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return polys;
}

// Get all the country access records from the db and save them to a map.
std::unordered_map<std::string, std::vector<int>> GetCountryAccess(AdminDB& db) {
  std::unordered_map<std::string, std::vector<int>> country_access;
  sqlite3_stmt* stmt = 0;
  uint32_t ret;
  uint32_t result = 0;
  std::string sql = "SELECT iso_code, trunk, trunk_link, track, footway, pedestrian, bridleway, "
                    "cycleway, path, motorroad from admin_access";

  ret = sqlite3_prepare_v2(db.get(), sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    result = sqlite3_step(stmt);

    while (result == SQLITE_ROW) {

      std::vector<int> access;
      std::string country_iso;
      if (sqlite3_column_type(stmt, 0) == SQLITE_TEXT) {
        country_iso = (char*)sqlite3_column_text(stmt, 0);
      }

      if (sqlite3_column_type(stmt, 1) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 1));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 2) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 2));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 3) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 3));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 4) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 4));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 5) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 5));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 6) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 6));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 7) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 7));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 8) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 8));
      } else {
        access.push_back(-1);
      }

      if (sqlite3_column_type(stmt, 9) == SQLITE_INTEGER) {
        access.push_back(sqlite3_column_int(stmt, 9));
      } else {
        access.push_back(-1);
      }

      country_access.emplace(country_iso, access);

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }

  return country_access;
}

} // namespace mjolnir
} // namespace valhalla
