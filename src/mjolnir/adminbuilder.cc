#include <cstdint>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/optional.hpp>

#include "baldr/graphconstants.h"
#include "filesystem.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/adminconstants.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/util.h"

#include "config.h"

// For OSM pbf reader
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
namespace bg = boost::geometry;

BOOST_GEOMETRY_REGISTER_POINT_2D(PointLL, double, bg::cs::geographic<bg::degree>, first, second)
typedef boost::geometry::model::polygon<PointLL> polygon_t;
typedef boost::geometry::model::multi_polygon<polygon_t> mpolygon_t;

namespace {

struct polygondata {
  polygon_t polygon;
  double area;
  unsigned containedbyid;
};

int polygondata_comparearea(const void* vp1, const void* vp2) {
  const polygondata* p1 = (const polygondata*)vp1;
  const polygondata* p2 = (const polygondata*)vp2;

  if (p1->area == p2->area) {
    return 0;
  }
  if (p1->area > p2->area) {
    return -1;
  }
  return 1;
}

std::string GetWkt(std::vector<polygon_t>& polygons) {
  std::ostringstream wkt;

  // something to hold the polygons for processing
  polygondata* polys = new polygondata[polygons.size()];

  unsigned totalpolys = 0;
  for (const auto& polygon : polygons) {
    if (polygon.outer().size() > 3) {
      polys[totalpolys].polygon = polygon;
      polys[totalpolys].area = bg::area(polygon);
      polys[totalpolys].containedbyid = 0;
      if (polys[totalpolys].area > 0.0) {
        totalpolys++;
      }
    }
  }

  if (totalpolys) {
    // sort the polygons by area in descending order
    qsort(polys, totalpolys, sizeof(polygondata), polygondata_comparearea);

    for (unsigned i = 0; i < totalpolys; ++i) {
      if (polys[i].containedbyid != 0) {
        continue;
      }

      for (unsigned j = i + 1; j < totalpolys; ++j) {
        // Does polygon[i] contain the smaller polygon[j]?
        if (polys[j].containedbyid == 0 && bg::within(polys[j].polygon, polys[i].polygon)) {
          // are we in a [i] contains [k] contains [j] situation
          // e.g. vatican state in rome, we want to cut vatican from italy
          // which would actually make j top level
          bool istoplevelafterall = false;
          for (unsigned k = i + 1; k < j; ++k) {
            if (polys[k].containedbyid == i && bg::within(polys[j].polygon, polys[k].polygon)) {
              istoplevelafterall = true;
              break;
            }
          }
          if (istoplevelafterall) {
            polys[j].containedbyid = i;
          }
        }
      }
    }
    // polys now is a list of polygons tagged with which ones are inside each other

    // List of polygons for multipolygon
    mpolygon_t multipoly;

    // For each top level polygon create a new polygon including any holes
    for (unsigned i = 0; i < totalpolys; ++i) {
      if (polys[i].containedbyid != 0) {
        continue;
      }

      // fill outer ring, inners further down
      multipoly.push_back(polys[i].polygon);

      // List of holes for this top level polygon, i.e. remove a country within a country
      for (unsigned j = i + 1; j < totalpolys; ++j) {
        if (polys[j].containedbyid == i) {
          multipoly.back().inners().push_back(polys[j].polygon.outer());
        }
      }

      // self-intersections?
      if (!bg::is_valid(multipoly.back())) {
        multipoly.pop_back();
        continue;
      }
    }

    // make sure the windings are correct and the rings are closed
    bg::correct(multipoly);
    wkt << bg::wkt(multipoly);
  }

  return wkt.str();
}

} // anonymous namespace

namespace valhalla {
namespace mjolnir {

/**
 * Build admins from protocol buffer input.
 */
void BuildAdminFromPBF(const boost::property_tree::ptree& pt,
                       const std::vector<std::string>& input_files) {

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  OSMAdminData osm_admin_data = PBFAdminParser::Parse(pt, input_files);

  // done with the protobuffer library, cant use it again after this
  OSMPBF::Parser::free();

  // Bail if bad path
  auto database = pt.get_optional<std::string>("admin");

  if (!database) {
    LOG_INFO("Admin config info not found. Admins will not be created.");
    return;
  }

  if (!filesystem::exists(filesystem::path(*database).parent_path())) {
    filesystem::create_directories(filesystem::path(*database).parent_path());
  }

  if (!filesystem::exists(filesystem::path(*database).parent_path())) {
    LOG_INFO("Admin directory not found. Admins will not be created.");
    return;
  }

  if (filesystem::exists(*database)) {
    filesystem::remove(*database);
  }

  sqlite3* db_handle;
  sqlite3_stmt* stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  ret = sqlite3_open_v2((*database).c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE,
                        NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + (*database));
    sqlite3_close(db_handle);
    return;
  }

  // loading SpatiaLite as an extension
  auto db_conn = make_spatialite_cache(db_handle);

  /* creating an admin POLYGON table */
  sql = "SELECT InitSpatialMetaData(1); CREATE TABLE admins (";
  sql += "admin_level INTEGER NOT NULL,";
  sql += "iso_code TEXT,";
  sql += "parent_admin INTEGER,";
  sql += "name TEXT NOT NULL,";
  sql += "name_en TEXT,";
  sql += "drive_on_right INTEGER NOT NULL,";
  sql += "allow_intersection_names INTEGER NOT NULL)";

  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  /* creating an admin access table
   * We could support all the commented out
   * columns below; however, for now we only
   * need the following ones until more people
   * update the specs on the wiki.
   */

  sql = "CREATE TABLE admin_access (";
  sql += "admin_id INTEGER NOT NULL,";
  sql += "iso_code TEXT,";
  // sql += "motorway INTEGER DEFAULT NULL,";
  // sql += "motorway_link INTEGER DEFAULT NULL,";
  sql += "trunk INTEGER DEFAULT NULL,";
  sql += "trunk_link INTEGER DEFAULT NULL,";
  // sql += "prim_ary INTEGER DEFAULT NULL,";
  // sql += "prim_ary_link INTEGER DEFAULT NULL,";
  // sql += "secondary INTEGER DEFAULT NULL,";
  // sql += "secondary_link INTEGER DEFAULT NULL,";
  // sql += "residential INTEGER DEFAULT NULL,";
  // sql += "residential_link INTEGER DEFAULT NULL,";
  // sql += "service INTEGER DEFAULT NULL,";
  // sql += "tertiary INTEGER DEFAULT NULL,";
  // sql += "tertiary_link INTEGER DEFAULT NULL,";
  // sql += "road INTEGER DEFAULT NULL,";
  sql += "track INTEGER DEFAULT NULL,";
  // sql += "unclassified INTEGER DEFAULT NULL,";
  // sql += "undefined INTEGER DEFAULT NULL,";
  // sql += "unknown INTEGER DEFAULT NULL,";
  // sql += "living_street INTEGER DEFAULT NULL,";
  sql += "footway INTEGER DEFAULT NULL,";
  sql += "pedestrian INTEGER DEFAULT NULL,";
  // sql += "steps INTEGER DEFAULT NULL,";
  sql += "bridleway INTEGER DEFAULT NULL,";
  // sql += "construction INTEGER DEFAULT NULL,";
  sql += "cycleway INTEGER DEFAULT NULL,";
  // sql += "bus_guideway INTEGER DEFAULT NULL,";
  sql += "path INTEGER DEFAULT NULL,";
  sql += "motorroad INTEGER DEFAULT NULL)";

  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  LOG_INFO("Created admin access table.");

  /* creating a MULTIPOLYGON Geometry column */
  sql = "SELECT AddGeometryColumn('admins', ";
  sql += "'geom', 4326, 'MULTIPOLYGON', 2)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  LOG_INFO("Created admin table.");

  /*
   * inserting some MULTIPOLYGONs
   * this time too we'll use a Prepared Statement
   */
  sql = "INSERT INTO admins (admin_level, iso_code, parent_admin, name, name_en, ";
  sql += "drive_on_right, allow_intersection_names, geom) VALUES (?, ?, ?, ?, ?, ? ,?, ";
  sql += "CastToMulti(GeomFromText(?, 4326)))";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
  }
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  uint32_t count = 0;
  try {

    for (const auto& admin : osm_admin_data.admins_) {
      std::vector<polygon_t> polygons;

      bool has_data = true;
      for (const auto memberid : admin.ways()) {
        auto itr = osm_admin_data.way_map.find(memberid);

        // A relation may be included in an extract but it's members may not.
        // Example:  PA extract can contain a NY relation.
        if (itr == osm_admin_data.way_map.end()) {
          has_data = false;
          break;
        }

        polygon_t polygon;
        for (const auto ref_id : itr->second) {
          bg::append(polygon, osm_admin_data.shape_map.at(ref_id));
        }

        // close the ring if it's open and fix the winding if necessary
        bg::correct(polygon);
        polygons.push_back(polygon);

      } // member loop

      if (has_data) {
        std::string wkt = GetWkt(polygons);
        std::string name;
        std::string name_en;
        std::string iso;

        count++;
        sqlite3_reset(stmt);
        sqlite3_clear_bindings(stmt);
        sqlite3_bind_int(stmt, 1, admin.admin_level());

        if (admin.iso_code_index()) {
          iso = osm_admin_data.name_offset_map.name(admin.iso_code_index());
          sqlite3_bind_text(stmt, 2, iso.c_str(), iso.length(), SQLITE_STATIC);
        } else {
          sqlite3_bind_null(stmt, 2);
        }

        sqlite3_bind_null(stmt, 3);

        name = osm_admin_data.name_offset_map.name(admin.name_index());
        sqlite3_bind_text(stmt, 4, name.c_str(), name.length(), SQLITE_STATIC);

        if (admin.name_en_index()) {
          name_en = osm_admin_data.name_offset_map.name(admin.name_en_index());
          sqlite3_bind_text(stmt, 5, name_en.c_str(), name_en.length(), SQLITE_STATIC);
        } else {
          sqlite3_bind_null(stmt, 5);
        }

        sqlite3_bind_int(stmt, 6, admin.drive_on_right());
        sqlite3_bind_int(stmt, 7, admin.allow_intersection_names());
        sqlite3_bind_text(stmt, 8, wkt.c_str(), wkt.length(), SQLITE_STATIC);
        /* performing INSERT INTO */
        ret = sqlite3_step(stmt);
        if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
          continue;
        }
        LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
        LOG_ERROR("sqlite3_step() Name: " + osm_admin_data.name_offset_map.name(admin.name_index()));
        LOG_ERROR("sqlite3_step() Name:en: " +
                  osm_admin_data.name_offset_map.name(admin.name_en_index()));
        LOG_ERROR("sqlite3_step() Admin Level: " + std::to_string(admin.admin_level()));
        LOG_ERROR("sqlite3_step() Drive on Right: " + std::to_string(admin.drive_on_right()));
        LOG_ERROR("sqlite3_step() Allow Intersection Names: " +
                  std::to_string(admin.allow_intersection_names()));
      } // has data
    }   // admins
  } catch (std::exception& e) {
    LOG_ERROR("Standard exception processing relation: " + std::string(e.what()));
  } catch (...) { LOG_ERROR("Exception caught processing relations."); }

  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Inserted " + std::to_string(count) + " admin areas");

  sql = "SELECT CreateSpatialIndex('admins', 'geom')";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Created spatial index");

  sql = "CREATE INDEX IdxLevel ON admins (admin_level)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Created Level index");

  sql = "CREATE INDEX IdxDriveOnRight ON admins (drive_on_right)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Created Drive On Right index");

  sql = "CREATE INDEX IdxAllowIntersectionNames ON admins (allow_intersection_names)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Created allow intersection names index");

  sql = "update admins set drive_on_right = (select a.drive_on_right from admins";
  sql += " a where ST_Covers(a.geom, admins.geom) and admins.admin_level != ";
  sql += "a.admin_level and a.drive_on_right=0) where rowid = ";
  sql += "(select admins.rowid from admins a where ST_Covers(a.geom, admins.geom) ";
  sql += "and admins.admin_level != a.admin_level and a.drive_on_right=0)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Done updating drive on right column.");

  sql = "update admins set allow_intersection_names = (select a.allow_intersection_names from admins";
  sql += " a where ST_Covers(a.geom, admins.geom) and admins.admin_level != ";
  sql += "a.admin_level and a.allow_intersection_names=1) where rowid = ";
  sql += "(select admins.rowid from admins a where ST_Covers(a.geom, admins.geom) ";
  sql += "and admins.admin_level != a.admin_level and a.allow_intersection_names=1)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Done updating allow intersection names column.");

  sql = "update admins set parent_admin = (select a.rowid from admins";
  sql += " a where ST_Covers(a.geom, admins.geom) and admins.admin_level != ";
  sql += "a.admin_level) where rowid = ";
  sql += "(select admins.rowid from admins a where ST_Covers(a.geom, admins.geom) ";
  sql += "and admins.admin_level != a.admin_level)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Done updating Parent admin");

  sql = "INSERT into admin_access (admin_id, iso_code, trunk, trunk_link, track, footway, ";
  sql += "pedestrian, bridleway, cycleway, path, motorroad) VALUES (";
  sql += "(select rowid from admins where (name = ? or name_en = ?)), ";
  sql += "(select iso_code from admins where (name = ? or name_en = ?)), ";
  sql += "?, ?, ?, ?, ?, ?, ?, ?, ?)";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
  }
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  for (const auto& access : kCountryAccess) {

    const std::vector<int> column_values = access.second;

    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    sqlite3_bind_text(stmt, 1, access.first.c_str(), access.first.length(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, access.first.c_str(), access.first.length(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, access.first.c_str(), access.first.length(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 4, access.first.c_str(), access.first.length(), SQLITE_STATIC);

    for (uint32_t col = 0; col != column_values.size(); ++col) {
      int val = column_values.at(col);
      if (val != -1) {
        sqlite3_bind_int(stmt, col + 5, val);
      } else {
        sqlite3_bind_null(stmt, col + 5);
      }
    }

    /* performing INSERT INTO */
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)) +
              ".  Ignore if not using a planet extract or check if there was a name change for " +
              access.first.c_str());
  }

  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sqlite3_close(db_handle);

  LOG_INFO("Finished.");
}

} // namespace mjolnir

} // namespace valhalla
