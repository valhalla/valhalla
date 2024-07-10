#include "mjolnir/admin.h"
#include "baldr/datetime.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/util.h"
#include <sqlite3.h>
#include <unordered_map>

#include <spatialite.h>

namespace valhalla {
namespace mjolnir {

// Get the dbhandle of a sqlite db.  Used for timezones and admins DBs.
sqlite3* GetDBHandle(const std::string& database) {

  // Initialize the admin DB (if it exists)
  sqlite3* db_handle = nullptr;
  if (!database.empty() && filesystem::exists(database)) {
    uint32_t ret = sqlite3_open_v2(database.c_str(), &db_handle,
                                   SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
      return nullptr;
    }
    sqlite3_extended_result_codes(db_handle, 1);
  }
  return db_handle;
}

// Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
// poly.
uint32_t GetMultiPolyId(const std::multimap<uint32_t, multi_polygon_type>& polys,
                        const PointLL& ll,
                        GraphTileBuilder& graphtile) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    if (bg::covered_by(p, poly.second, bg::strategy::within::crossings_multiply<point_type>())) {
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
uint32_t GetMultiPolyId(const std::multimap<uint32_t, multi_polygon_type>& polys, const PointLL& ll) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    // TODO: we recently discovered that boost::geometry doesn't do bbox checks to speed things up
    if (bg::covered_by(p, poly.second, bg::strategy::within::crossings_multiply<point_type>())) {
      return poly.first;
    }
  }
  return index;
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

  point_type p(ll.lng(), ll.lat());

  for (auto it = polys.qbegin(bg::index::covers(p)); it != polys.qend(); ++it) {
    // Make sure the point is in the admin poly (not just its envelope)
    if (bg::covered_by(p, std::get<1>(*it), bg::strategy::within::crossings_multiply<point_type>())) {
      auto& langs = std::get<2>(*it);
      bool is_default = std::get<3>(*it);

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
std::multimap<uint32_t, multi_polygon_type> GetTimeZones(sqlite3* db_handle,
                                                         const AABB2<PointLL>& aabb) {
  std::multimap<uint32_t, multi_polygon_type> polys;
  if (!db_handle) {
    return polys;
  }

  sqlite3_stmt* stmt = 0;
  uint32_t ret;
  uint32_t result = 0;

  std::string sql = "select TZID, st_astext(geom) as geom_text from tz_world where ";
  sql += "ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'tz_world' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    result = sqlite3_step(stmt);

    while (result == SQLITE_ROW) {
      std::string tz_id;
      std::string geom;

      if (sqlite3_column_type(stmt, 0) == SQLITE_TEXT) {
        tz_id = (char*)sqlite3_column_text(stmt, 0);
      }
      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT) {
        geom = (char*)sqlite3_column_text(stmt, 1);
      }

      uint32_t idx = DateTime::get_tz_db().to_index(tz_id);
      if (idx == 0) {
        sqlite3_finalize(stmt);
        throw std::runtime_error("Can't find timezone ID " + std::string(tz_id));
      }

      multi_polygon_type multi_poly;
      boost::geometry::read_wkt(geom, multi_poly);
      polys.emplace(idx, multi_poly);
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

void GetData(sqlite3* db_handle,
             sqlite3_stmt* stmt,
             const std::string& sql,
             GraphTileBuilder& tilebuilder,
             std::multimap<uint32_t, multi_polygon_type>& polys,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_ploys,
             bool languages_only = false) {
  uint32_t result = 0;
  bool dor = true;
  bool intersection_name = false;
  uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

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

      std::string geom;
      if (sqlite3_column_type(stmt, 9) == SQLITE_TEXT) {
        geom = (char*)sqlite3_column_text(stmt, 9);
      }

      uint32_t index = tilebuilder.AddAdmin(country_name, state_name, country_iso, state_iso);
      multi_polygon_type multi_poly;
      bg::read_wkt(geom, multi_poly);
      polys.emplace(index, multi_poly);
      drive_on_right.emplace(index, dor);
      allow_intersection_names.emplace(index, intersection_name);

      bg::model::box<point_type> box{};
      bg::envelope(multi_poly, box);
      if (!default_language.empty()) {
        auto langs = ParseLanguageTokens(default_language);
        language_ploys.insert(std::make_tuple(box, multi_poly, langs, true));
      }
      if (!supported_languages.empty()) {
        auto langs = ParseLanguageTokens(supported_languages);
        language_ploys.insert(std::make_tuple(box, multi_poly, langs, false));
      }

    } else {

      std::string supported_languages;
      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT) {
        supported_languages = (char*)sqlite3_column_text(stmt, 1);
      }

      std::string default_language;
      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) {
        default_language = (char*)sqlite3_column_text(stmt, 2);
      }

      std::string geom;
      if (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) {
        geom = (char*)sqlite3_column_text(stmt, 3);
      }

      multi_polygon_type multi_poly;
      bg::read_wkt(geom, multi_poly);
      bg::model::box<point_type> box{};
      bg::envelope(multi_poly, box);

      if (!default_language.empty()) {
        auto langs = ParseLanguageTokens(default_language);
        language_ploys.insert(std::make_tuple(box, multi_poly, langs, true));
      }
      if (!supported_languages.empty()) {
        auto langs = ParseLanguageTokens(supported_languages);
        language_ploys.insert(std::make_tuple(box, multi_poly, langs, false));
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
std::multimap<uint32_t, multi_polygon_type>
GetAdminInfo(sqlite3* db_handle,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             std::unordered_map<uint32_t, bool>& allow_intersection_names,
             language_poly_index& language_polys,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder) {
  std::multimap<uint32_t, multi_polygon_type> polys;
  if (!db_handle) {
    return polys;
  }

  sqlite3_stmt* stmt = 0;

  // default language query
  std::string sql =
      "SELECT admin_level, supported_languages, default_language, st_astext(geom) from ";
  sql +=
      " admins where (supported_languages is NOT NULL or default_language is NOT NULL) and ST_Intersects(geom, BuildMBR(" +
      std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and admin_level>4 ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) order by admin_level desc, name;";
  GetData(db_handle, stmt, sql, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys, true);

  // state query
  sql = "SELECT country.name, state.name, country.iso_code, ";
  sql += "state.iso_code, state.drive_on_right, state.allow_intersection_names, state.admin_level, ";
  sql +=
      "state.supported_languages, state.default_language, st_astext(state.geom) from admins state, admins country where ";
  sql += "ST_Intersects(state.geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and ";
  sql += "country.rowid = state.parent_admin and state.admin_level=4 ";
  sql += "and state.rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) order by state.name, country.name;";
  GetData(db_handle, stmt, sql, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys);

  // country query
  sql = "SELECT name, \"\", iso_code, \"\", drive_on_right, allow_intersection_names, admin_level, ";
  sql +=
      "supported_languages, default_language, st_astext(geom) from  admins where ST_Intersects(geom, BuildMBR(" +
      std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and admin_level=2 ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) order by name;";
  GetData(db_handle, stmt, sql, tilebuilder, polys, drive_on_right, allow_intersection_names,
          language_polys);

  if (stmt) { // just in case something bad happened.
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return polys;
}

// Get all the country access records from the db and save them to a map.
std::unordered_map<std::string, std::vector<int>> GetCountryAccess(sqlite3* db_handle) {

  std::unordered_map<std::string, std::vector<int>> country_access;

  if (!db_handle) {
    return country_access;
  }

  sqlite3_stmt* stmt = 0;
  uint32_t ret;
  uint32_t result = 0;
  std::string sql = "SELECT iso_code, trunk, trunk_link, track, footway, pedestrian, bridleway, "
                    "cycleway, path, motorroad from admin_access";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

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
