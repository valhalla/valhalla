#include <cstdint>
#include <string>
#include <vector>

#include "baldr/graphconstants.h"
#include "filesystem.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/adminconstants.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/util.h"

// sqlite is included in util.h and must be before spatialite
#include <spatialite.h>

#include "config.h"

/* Need to know which geos version we have to work out which headers to include */
#include <geos/version.h>

#define USE_UNSTABLE_GEOS_CPP_API
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
#include <geos/geom/CoordinateArraySequence.h>
#else
#include <geos/geom/CoordinateSequenceFactory.h>
#endif
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/MultiPolygon.h>
#include <geos/geom/Point.h>
#include <geos/geom/Polygon.h>
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 9
#include <geos/operation/linemerge/LineMerger.h>
#else
#include <geos/opLinemerge.h>
#endif
#include <geos/util/GEOSException.h>

#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace geos::geom;
using namespace geos::io;
using namespace geos::util;
using namespace geos::operation::linemerge;

// For OSM pbf reader
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace bpo = boost::program_options;
using namespace valhalla::midgard;

namespace {

struct polygondata {
  Polygon* polygon;
  LinearRing* ring;
  double area;
  int iscontained;
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

std::vector<std::string> GetWkts(std::unique_ptr<Geometry>& mline) {
  std::vector<std::string> wkts;

#if 3 == GEOS_VERSION_MAJOR && 6 <= GEOS_VERSION_MINOR
  auto gf = GeometryFactory::create();
#else
  std::unique_ptr<GeometryFactory> gf(new GeometryFactory());
#endif

  LineMerger merger;
  merger.add(mline.get());
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 9
  std::vector<std::unique_ptr<LineString>> merged(merger.getMergedLineStrings());
#else
  std::unique_ptr<std::vector<LineString*>> merged(merger.getMergedLineStrings());
#endif
  WKTWriter writer;

  // Procces ways into lines or simple polygon list
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 9
  polygondata* polys = new polygondata[merged.size()];
#else
  polygondata* polys = new polygondata[merged->size()];
#endif

  unsigned totalpolys = 0;
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 9
  for (unsigned i = 0; i < merged.size(); ++i) {
    std::unique_ptr<LineString> pline(merged[i].release());
#else
  for (unsigned i = 0; i < merged->size(); ++i) {
    std::unique_ptr<LineString> pline((*merged)[i]);
#endif
    if (pline->getNumPoints() > 3 && pline->isClosed()) {
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
      polys[totalpolys].polygon =
          gf->createPolygon(gf->createLinearRing(pline->getCoordinates())).release();
      polys[totalpolys].ring = gf->createLinearRing(pline->getCoordinates()).release();
#else
      polys[totalpolys].polygon = gf->createPolygon(gf->createLinearRing(pline->getCoordinates()), 0);
      polys[totalpolys].ring = gf->createLinearRing(pline->getCoordinates());
#endif
      polys[totalpolys].area = polys[totalpolys].polygon->getArea();
      polys[totalpolys].iscontained = 0;
      polys[totalpolys].containedbyid = 0;
      if (polys[totalpolys].area > 0.0) {
        totalpolys++;
      } else {
        delete (polys[totalpolys].polygon);
        delete (polys[totalpolys].ring);
      }
    }
  }

  if (totalpolys) {
    qsort(polys, totalpolys, sizeof(polygondata), polygondata_comparearea);

    for (unsigned i = 0; i < totalpolys; ++i) {
      if (polys[i].iscontained != 0) {
        continue;
      }

      for (unsigned j = i + 1; j < totalpolys; ++j) {
        // Does polygon[i] contain the smaller polygon[j]?
        if (polys[j].containedbyid == 0 && polys[i].polygon->contains(polys[j].polygon)) {
          // are we in a [i] contains [k] contains [j] situation
          // which would actually make j top level
          bool istoplevelafterall = false;
          for (unsigned k = i + 1; k < j; ++k) {
            if (polys[k].iscontained && polys[k].containedbyid == i &&
                polys[k].polygon->contains(polys[j].polygon)) {
              istoplevelafterall = true;
              break;
            }
          }
          if (istoplevelafterall) {
            polys[j].iscontained = 1;
            polys[j].containedbyid = i;
          }
        }
      }
    }
    // polys now is a list of polygons tagged with which ones are inside each other

    // List of polygons for multipolygon
    std::unique_ptr<std::vector<Geometry*>> polygons(new std::vector<Geometry*>);

    // For each top level polygon create a new polygon including any holes
    for (unsigned i = 0; i < totalpolys; ++i) {
      if (polys[i].iscontained != 0) {
        continue;
      }

      // List of holes for this top level polygon
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
      std::unique_ptr<std::vector<LinearRing*>> interior(new std::vector<LinearRing*>);
#else
      std::unique_ptr<std::vector<Geometry*>> interior(new std::vector<Geometry*>);
#endif
      for (unsigned j = i + 1; j < totalpolys; ++j) {
        if (polys[j].iscontained == 1 && polys[j].containedbyid == i) {
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
          interior->push_back(polys[j].ring);
#else
          interior->push_back(polys[j].ring);
#endif
        }
      }

#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
      Polygon* poly(gf->createPolygon(polys[i].ring, interior.release()));
#else
      Polygon* poly(gf->createPolygon(polys[i].ring, interior.release()));
#endif
      poly->normalize();
      polygons->push_back(poly);
    }

    // Make a multipolygon
    std::unique_ptr<Geometry> multipoly(gf->createMultiPolygon(polygons.release()));
    if (!multipoly->isValid()) {
      multipoly = std::unique_ptr<Geometry>(multipoly->buffer(0));
    }
    multipoly->normalize();

    if (multipoly->isValid()) {
      wkts.push_back(writer.write(multipoly.get()));
    }
  }

  for (unsigned i = 0; i < totalpolys; ++i) {
    delete (polys[i].polygon);
  }

  delete[](polys);

  return wkts;
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

  spatialite_init(0);

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
  if (!valhalla::mjolnir::load_spatialite(db_handle)) {
    sqlite3_close(db_handle);
    return;
  }

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
  bool has_data;
#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 6
  auto gf = GeometryFactory::create();
#else
  std::unique_ptr<GeometryFactory> gf(new GeometryFactory());
#endif

  try {

    for (const auto& admin : osm_admin_data.admins_) {

      std::unique_ptr<Geometry> geom;
      std::unique_ptr<std::vector<Geometry*>> lines(new std::vector<Geometry*>);
      has_data = true;

      for (const auto memberid : admin.ways()) {

        auto itr = osm_admin_data.way_map.find(memberid);

        // A relation may be included in an extract but it's members may not.
        // Example:  PA extract can contain a NY relation.
        if (itr == osm_admin_data.way_map.end()) {
          has_data = false;
          break;
        }

#if GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR >= 8
        auto coords = std::unique_ptr<CoordinateArraySequence>(new CoordinateArraySequence);
#else
        std::unique_ptr<CoordinateSequence> coords(
            gf->getCoordinateSequenceFactory()->create((size_t)0, (size_t)2));
#endif

        for (const auto ref_id : itr->second) {

          const PointLL ll = osm_admin_data.shape_map.at(ref_id);

          Coordinate c;
          c.x = ll.lng();
          c.y = ll.lat();
          coords->add(c, 0);
        }

        if (coords->getSize() > 1) {
          geom = std::unique_ptr<Geometry>(gf->createLineString(coords.release()));
          lines->push_back(geom.release());
        }

      } // member loop

      if (has_data) {

        std::unique_ptr<Geometry> mline(gf->createMultiLineString(lines.release()));
        std::vector<std::string> wkts = GetWkts(mline);
        std::string name;
        std::string name_en;
        std::string iso;

        for (const auto& wkt : wkts) {

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
          LOG_ERROR("sqlite3_step() Name: " +
                    osm_admin_data.name_offset_map.name(admin.name_index()));
          LOG_ERROR("sqlite3_step() Name:en: " +
                    osm_admin_data.name_offset_map.name(admin.name_en_index()));
          LOG_ERROR("sqlite3_step() Admin Level: " + std::to_string(admin.admin_level()));
          LOG_ERROR("sqlite3_step() Drive on Right: " + std::to_string(admin.drive_on_right()));
          LOG_ERROR("sqlite3_step() Allow Intersection Names: " +
                    std::to_string(admin.allow_intersection_names()));
        }
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
