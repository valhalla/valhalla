#include "mjolnir/admin.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include <boost/filesystem/operations.hpp>
#include <spatialite.h>
#include <sqlite3.h>
#include <unordered_map>

namespace valhalla {
namespace mjolnir {

// Get the dbhandle of a sqlite db.  Used for timezones and admins DBs.
sqlite3* GetDBHandle(const std::string& database) {

  // Initialize the admin DB (if it exists)
  sqlite3* db_handle = nullptr;
  if (!database.empty() && boost::filesystem::exists(database)) {
    spatialite_init(0);
    sqlite3_stmt* stmt = 0;
    char* err_msg = nullptr;
    std::string sql;
    uint32_t ret = sqlite3_open_v2(database.c_str(), &db_handle,
                                   SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
      db_handle = nullptr;
      return db_handle;
    }

    // loading SpatiaLite as an extension
    sqlite3_enable_load_extension(db_handle, 1);
#if SQLITE_VERSION_NUMBER > 3008007
    sql = "SELECT load_extension('mod_spatialite')";
#else
    sql = "SELECT load_extension('libspatialite')";
#endif
    ret = sqlite3_exec(db_handle, sql.c_str(), nullptr, nullptr, &err_msg);
    if (ret != SQLITE_OK) {
      LOG_ERROR("load_extension() error: " + std::string(err_msg));
      sqlite3_free(err_msg);
      sqlite3_close(db_handle);
    }
  }
  return db_handle;
}

// Get the polygon index.  Used by tz and admin areas.  Checks if the pointLL is covered_by the
// poly.
uint32_t GetMultiPolyId(const std::unordered_multimap<uint32_t, multi_polygon_type>& polys,
                        const PointLL& ll,
                        GraphTileBuilder& graphtile) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    if (boost::geometry::covered_by(p, poly.second)) {
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
uint32_t GetMultiPolyId(const std::unordered_multimap<uint32_t, multi_polygon_type>& polys,
                        const PointLL& ll) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    if (boost::geometry::covered_by(p, poly.second))
      return poly.first;
  }
  return index;
}

// Get the timezone polys from the db
std::unordered_multimap<uint32_t, multi_polygon_type> GetTimeZones(sqlite3* db_handle,
                                                                   const AABB2<PointLL>& aabb) {
  std::unordered_multimap<uint32_t, multi_polygon_type> polys;
  if (!db_handle) {
    return polys;
  }

  sqlite3_stmt* stmt = 0;
  uint32_t ret;
  char* err_msg = nullptr;
  uint32_t result = 0;

  std::string sql = "select TZID, st_astext(geom) from tz_world where ";
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
        result = sqlite3_step(stmt);
        continue;
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

void GetData(sqlite3* db_handle,
             sqlite3_stmt* stmt,
             const std::string& sql,
             GraphTileBuilder& tilebuilder,
             std::unordered_multimap<uint32_t, multi_polygon_type>& polys,
             std::unordered_map<uint32_t, bool>& drive_on_right) {
  uint32_t result = 0;
  bool dor = true;

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

    std::string country_name, state_name, country_iso, state_iso;

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

    std::string geom;
    if (sqlite3_column_type(stmt, 5) == SQLITE_TEXT) {
      geom = (char*)sqlite3_column_text(stmt, 5);
    }

    uint32_t index = tilebuilder.AddAdmin(country_name, state_name, country_iso, state_iso);
    multi_polygon_type multi_poly;
    boost::geometry::read_wkt(geom, multi_poly);
    polys.emplace(index, multi_poly);
    drive_on_right.emplace(index, dor);

    result = sqlite3_step(stmt);
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
}

// Get the admin polys that intersect with the tile bounding box.
std::unordered_multimap<uint32_t, multi_polygon_type>
GetAdminInfo(sqlite3* db_handle,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             const AABB2<PointLL>& aabb,
             GraphTileBuilder& tilebuilder) {
  std::unordered_multimap<uint32_t, multi_polygon_type> polys;
  if (!db_handle) {
    return polys;
  }

  sqlite3_stmt* stmt = 0;
  // state query
  std::string sql = "SELECT country.name, state.name, country.iso_code, ";
  sql += "state.iso_code, state.drive_on_right, st_astext(state.geom) ";
  sql += "from admins state, admins country where ";
  sql += "ST_Intersects(state.geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and ";
  sql += "country.rowid = state.parent_admin and state.admin_level=4 ";
  sql += "and state.rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";
  GetData(db_handle, stmt, sql, tilebuilder, polys, drive_on_right);

  // country query
  sql = "SELECT name, \"\", iso_code, \"\", drive_on_right, st_astext(geom) from ";
  sql += " admins where ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and admin_level=2 ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";
  GetData(db_handle, stmt, sql, tilebuilder, polys, drive_on_right);

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
  char* err_msg = nullptr;
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
