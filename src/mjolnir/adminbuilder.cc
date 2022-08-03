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

#include "config.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/property_tree/ptree.hpp>

using polar_coordinate_system_t = boost::geometry::cs::geographic<boost::geometry::degree>;
using point_t = boost::geometry::model::d2::point_xy<double, polar_coordinate_system_t>;
using ring_t = boost::geometry::model::ring<point_t>;
using polygon_t = boost::geometry::model::polygon<point_t>;
using multipolygon_t = boost::geometry::model::multi_polygon<polygon_t>;

// in order to work with boost points inside stl containers
namespace std {
template <> struct hash<point_t> {
  inline size_t operator()(const point_t& p) const {
    return (uint64_t(p.x() * 1e7 + 180 * 1e7) << 32) | (uint64_t(p.y() * 1e7 + 90 * 1e7));
  }
};
template <> struct equal_to<point_t> {
  inline bool operator()(const point_t& a, const point_t& b) const {
    return a.x() == b.x() && a.y() == b.y();
  }
};
} // namespace std

// For OSM pbf reader
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

/**
 * Converts a series of a linestrings into one or more polygons (rings) by connecting contiguous
 * ones until a ring is formed
 * @param lines the unmerged lines
 * @param line_lookup a multi map that lets one easily
 * @return
 */
std::vector<std::vector<point_t>> line_merge(std::vector<std::vector<point_t>> lines,
                                             std::unordered_multimap<point_t, size_t> line_lookup) {
  std::vector<std::vector<point_t>> rings;
  std::equal_to<point_t> point_equals;

  // keep going while we have threads to pull
  while (!line_lookup.empty()) {
    // start connecting the first line we have to adjacent ones
    auto& ring = *rings.emplace(rings.end());
    for (auto line_itr = line_lookup.begin(); line_itr != line_lookup.end();
         line_itr = line_lookup.find(ring.back())) {
      // grab the line segment to add
      auto line_index = line_itr->second;
      auto& line = lines[line_index];
      // we can add this line in the forward direction
      if ((ring.empty() && point_equals(line_itr->first, line.front())) ||
          (!ring.empty() && point_equals(ring.back(), line.front()))) {
        ring.insert(ring.end(), std::make_move_iterator(line.begin() + !ring.empty()),
                    std::make_move_iterator(line.end()));
      } // have to add this segment backwards
      else {
        ring.insert(ring.end(), std::make_move_iterator(line.rbegin() + !ring.empty()),
                    std::make_move_iterator(line.rend()));
      }

      // done with this segment and its other end
      line_lookup.erase(line_itr);
      line_itr = line_lookup.find(ring.back());
      assert(line_itr != line_lookup.end());
      if (line_itr->second != line_index)
        ++line_itr;
      assert(line_itr != line_lookup.end());
      line_lookup.erase(line_itr);
    }
    // degenerate rings are ignored, unconnected rings or missing relation members cause this
    if (ring.size() < 4 || !point_equals(ring.front(), ring.back()))
      rings.pop_back();
  }

  return rings;
}

struct polygon_data {
  polygon_t polygon;
  double area;
  bool is_inner;
  size_t outer_idx;
  bool operator<(const polygon_data& p) const {
    return area >= p.area; // sort descending
  }
};

std::string to_wkt(const std::vector<std::vector<point_t>>& rings) {
  // Procces ways into lines or simple polygon list
  std::vector<polygon_data> polys;
  for (const auto& ring : rings) {
    polygon_data pd{};
    boost::geometry::assign_points(pd.polygon, ring);
    boost::geometry::correct(pd.polygon);
    if (!boost::geometry::is_valid(pd.polygon))
      continue;
    pd.area = boost::geometry::area(pd.polygon);
    polys.emplace_back(std::move(pd));
  }

  // if we have anything that wasnt degenerate we can convert it to wkt
  if (polys.empty())
    return "";

  // sort descending by area
  std::sort(polys.begin(), polys.end());

  // we need to figure out which rings are inners of others
  // our strategy is to only ever go one level deep with inners, that is inners of inners
  // will just become outers that happen to be inside the hole of another outer
  for (size_t i = 0; i < polys.size(); ++i) {
    // im already claimed as an inner no need to search
    if (polys[i].is_inner) {
      continue;
    }

    // i may have an inner myself though
    for (size_t j = i + 1; j < polys.size(); ++j) {
      // is j inside i but not the inner of any other rings?
      if (!polys[j].is_inner && boost::geometry::within(polys[j].polygon, polys[i].polygon)) {

        // its possible that j would be the inner of someone who is already my inner, call them k
        // in that case j will be its own outer that lives in the hole that k carves out of i
        bool j_is_inner = true;
        for (size_t k = i + 1; k < j; ++k) {
          if (polys[k].is_inner && polys[k].outer_idx == i &&
              boost::geometry::within(polys[j].polygon, polys[k].polygon)) {
            j_is_inner = false;
            break;
          }
        }

        // if j is not living inside another ring then its now i's inner
        if (j_is_inner) {
          polys[j].is_inner = true;
          polys[j].outer_idx = i;
        }
      }
    }
  }

  // For each top level polygon create a new polygon including any holes
  for (size_t i = 0; i < polys.size(); ++i) {
    if (polys[i].is_inner) {
      continue;
    }

    // List of holes for this top level polygon
    for (size_t j = i + 1; j < polys.size(); ++j) {
      if (polys[j].is_inner && polys[j].outer_idx == i) {
        polys[i].polygon.inners().push_back(polys[j].polygon.outer());
        polys.erase(polys.begin() + j);
      }
    }

    // Make sure the geom is good
    boost::geometry::correct(polys[i].polygon);
  }

  // Make a simple container of multiple polygons
  multipolygon_t stripped;
  stripped.reserve(polys.size());
  for (auto& poly : polys) {
    stripped.push_back(std::move(poly.polygon));
  }

  // Get back some wkt
  std::stringstream ss;
  ss << boost::geometry::wkt(stripped);
  return ss.str();
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
  bool has_data;
  try {
    // for each admin area (relation)
    for (const auto& admin : osm_admin_data.admins) {
      LOG_DEBUG("Building admin: " + osm_admin_data.name_offset_map.name(admin.name_index));

      // have to keep the geom of each member and lookup so we can merge them together
      std::vector<std::vector<point_t>> lines;
      std::unordered_multimap<point_t, size_t> line_lookup;
      has_data = true;

      // get all the individual members of the admin relation merged into one ring
      for (const auto memberid : admin.ways) {
        auto itr = osm_admin_data.way_map.find(memberid);

        // A relation may be included in an extract but it's members may not.
        // Example:  PA extract can contain a NY relation.
        if (itr == osm_admin_data.way_map.end()) {
          has_data = false;
          break;
        }

        // build the line geom
        std::vector<point_t> coords;
        for (const auto ref_id : itr->second) {
          const auto& ll = osm_admin_data.shape_map.at(ref_id);
          coords.push_back(point_t{ll.first, ll.second});
        }

        // remember how to find this line
        if (!coords.empty()) {
          line_lookup.insert({coords.front(), lines.size()});
          line_lookup.insert({coords.back(), lines.size()});
          lines.push_back(std::move(coords));
        }
      }

      // if we had a complete relation (ie we found all members)
      if (has_data) {
        // merge all the members into one or more rings
        auto rings = line_merge(lines, line_lookup);
        if (rings.empty())
          continue;

        // convert those into wkt format so we can put it into sqlite
        // TODO: why was this a vector before, just to find when it was empty?
        auto wkt = to_wkt(rings);
        if (wkt.empty())
          continue;

        std::string name;
        std::string name_en;
        std::string iso;

        // load it into sqlite
        count++;
        sqlite3_reset(stmt);
        sqlite3_clear_bindings(stmt);
        sqlite3_bind_int(stmt, 1, admin.admin_level);

        if (admin.iso_code_index) {
          iso = osm_admin_data.name_offset_map.name(admin.iso_code_index);
          sqlite3_bind_text(stmt, 2, iso.c_str(), iso.length(), SQLITE_STATIC);
        } else {
          sqlite3_bind_null(stmt, 2);
        }

        sqlite3_bind_null(stmt, 3);

        name = osm_admin_data.name_offset_map.name(admin.name_index);
        sqlite3_bind_text(stmt, 4, name.c_str(), name.length(), SQLITE_STATIC);

        if (admin.name_en_index) {
          name_en = osm_admin_data.name_offset_map.name(admin.name_en_index);
          sqlite3_bind_text(stmt, 5, name_en.c_str(), name_en.length(), SQLITE_STATIC);
        } else {
          sqlite3_bind_null(stmt, 5);
        }

        sqlite3_bind_int(stmt, 6, admin.drive_on_right);
        sqlite3_bind_int(stmt, 7, admin.allow_intersection_names);
        sqlite3_bind_text(stmt, 8, wkt.c_str(), wkt.length(), SQLITE_STATIC);
        /* performing INSERT INTO */
        ret = sqlite3_step(stmt);
        if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
          continue;
        }
        LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
        LOG_ERROR("sqlite3_step() Name: " + osm_admin_data.name_offset_map.name(admin.name_index));
        LOG_ERROR("sqlite3_step() Name:en: " +
                  osm_admin_data.name_offset_map.name(admin.name_en_index));
        LOG_ERROR("sqlite3_step() Admin Level: " + std::to_string(admin.admin_level));
        LOG_ERROR("sqlite3_step() Drive on Right: " + std::to_string(admin.drive_on_right));
        LOG_ERROR("sqlite3_step() Allow Intersection Names: " +
                  std::to_string(admin.allow_intersection_names));

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
