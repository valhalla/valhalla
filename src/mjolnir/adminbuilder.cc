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

#include <geos_c.h>

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
 * a simple singleton to wrap geos setup and tear down
 */
struct geos_helper_t {
  static const geos_helper_t& get() {
    static geos_helper_t singleton;
    return singleton;
  }
  template <typename striped_container_t>
  static GEOSCoordSequence* from_striped_container(const striped_container_t& coords) {
    // sadly we dont layout the memory in parallel arrays so we have to copy to geos
    size_t i = 0;
    GEOSCoordSequence* geos_coords = GEOSCoordSeq_create(coords.size(), 2);
    for (const auto& coord : coords) {
      GEOSCoordSeq_setXY(geos_coords, i++, coord.x(), coord.y());
    }
    return geos_coords;
  }
  template <typename striped_container_t>
  static striped_container_t to_striped_container(const GEOSCoordSequence* coords) {
    // sadly we dont layout the memory in parallel arrays so we have to copy from geos
    unsigned int coords_size;
    GEOSCoordSeq_getSize(coords, &coords_size);
    striped_container_t container;
    container.reserve(coords_size);
    for (unsigned int i = 0; i < coords_size; ++i) {
      double x, y;
      GEOSCoordSeq_getXY(coords, i, &x, &y);
      container[i].x(x);
      container[i].y(y);
    }
    return container;
  }

protected:
  static void message_handler(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
  }
  geos_helper_t() {
    initGEOS(message_handler, message_handler);
  }
  ~geos_helper_t() {
    finishGEOS();
  }
};

/**
 * Temporarily convert to a less convenient geos representation in c to get access to a working
 * buffer implementation and then back to boost geometry
 * @param ring   to buffer to fix self intersections
 * @param rings  any resulting rings are output here
 * @return one or more rings
 */
void buffer_ring(const ring_t& ring, std::vector<ring_t>& rings) {
  auto* geos_coords = geos_helper_t::from_striped_container(ring);
  auto* geos_ring = GEOSGeom_createLinearRing(geos_coords);
  auto* buffered = GEOSBuffer(geos_ring, 0, 8);
  auto geom_type = GEOSGeomTypeId(buffered);
  switch (geom_type) {
    case GEOS_POLYGON: {
      auto* coords = GEOSGeom_getCoordSeq(GEOSGetExteriorRing(buffered));
      rings.emplace_back(geos_helper_t::to_striped_container<ring_t>(coords));
      // TODO: this can't have inners right?
      break;
    }
    case GEOS_GEOMETRYCOLLECTION: {
      for (int i = 0; i < GEOSGetNumGeometries(buffered); ++i) {
        auto* geom = GEOSGetGeometryN(buffered, i);
        if (GEOSGeomTypeId(geom) != GEOS_LINEARRING)
          throw std::runtime_error("Unusable geometry type after buffering");
        auto* coords = GEOSGeom_getCoordSeq(geom);
        rings.emplace_back(geos_helper_t::to_striped_container<ring_t>(coords));
      }
      break;
    }
    default:
      throw std::runtime_error("Unusable geometry type after buffering");
  }
  GEOSGeom_destroy(geos_ring);
  GEOSGeom_destroy(buffered);
}

void buffer_polygon(const polygon_t& polygon, multipolygon_t& multipolygon) {
  // TODO:
}

/**
 * Build a look up of ring segments for a given admin either for outers or inners
 * The look up can then be used to merge the segments together into connected rings
 * If a given admin is incomplete (in that its missing members) the function will throw
 * @param admin_data   used to look up ways shape (nodes)
 * @param admin        the admin for which we are building the lookup
 * @param outer        whether or not we should build a lookup for outers or inners
 * @param lines        ring segments that are built as output to be later connected together
 * @param line_lookup  a look up to find ring segments to help in connecting them together
 * @return true if all the members of the relation had corresponding geometry
 */
bool to_segments(const OSMAdminData& admin_data,
                 const OSMAdmin& admin,
                 bool outer,
                 std::vector<ring_t>& lines,
                 std::unordered_multimap<point_t, size_t>& line_lookup) {
  // get all the individual members of the admin relation merged into one ring
  auto roll_itr = admin.roles.begin();
  for (const auto memberid : admin.ways) {
    // skip roles we arent interested in
    if (*(roll_itr++) != outer)
      continue;

    // A relation may be included in an extract but it's members may not
    // Example:  PA extract can contain an NY relation but wont have all its members
    auto w_itr = admin_data.way_map.find(memberid);
    if (w_itr == admin_data.way_map.end())
      return false;

    // build the line geom
    ring_t coords;
    for (const auto node_id : w_itr->second) {
      // although unlikely, we could have the way but not all the nodes
      auto n_itr = admin_data.shape_map.find(node_id);
      if (n_itr == admin_data.shape_map.end())
        return false;
      coords.push_back(point_t{n_itr->second.first, n_itr->second.second});
    }

    // remember how to find this line
    if (!coords.empty()) {
      line_lookup.insert({coords.front(), lines.size()});
      line_lookup.insert({coords.back(), lines.size()});
      lines.push_back(std::move(coords));
    }
  }
  return true;
}

/**
 * Converts a series of a linestrings into one or more polygons (rings) by connecting contiguous
 * ones until a ring is formed
 * @param lines the unmerged lines
 * @param line_lookup a multi map that lets one easily
 * @return zero or more rings
 */
std::vector<ring_t> to_rings(std::vector<ring_t>& lines,
                             std::unordered_multimap<point_t, size_t>& line_lookup) {

  // keep going while we have threads to pull
  std::vector<ring_t> rings;
  std::equal_to<point_t> point_equals;
  while (!line_lookup.empty()) {
    // start connecting the first line we have to adjacent ones
    ring_t ring;
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
      continue;

    // otherwise we try to make sure the ring is not self intersecting etc and correct it if it is
    multipolygon_t buffered;
    boost::geometry::correct(ring);
    // boost::geometry::buffer(ring, buffered, 0);
    buffer_ring(ring, rings);
  }

  return rings;
}

struct polygon_data {
  polygon_t polygon;
  double area;
  bool operator<(const polygon_data& p) const {
    return area < p.area;
  }
};

/**
 * Takes outer and inner rings and combines them first into polygons and finally into a multipolygon
 * @param outers outer rings of polygons
 * @param inners inner rings of polygons
 * @return the multipolygon of the combined outer and inner rings
 */
multipolygon_t to_multipolygon(std::vector<ring_t>& outers, std::vector<ring_t>& inners) {
  // Associate an area with each outer so we can
  std::vector<polygon_data> polys;
  for (auto& outer : outers) {
    polygon_data pd{};
    pd.polygon.outer() = std::move(outer);
    pd.area = boost::geometry::area(pd.polygon);
    polys.emplace_back(std::move(pd));
  }

  // sort descending by area
  std::sort(polys.begin(), polys.end());

  // here we try to figure out which polygon to assign each inner to
  // thankfully it seems second order enclaves arent a problem to worry about in practice
  // previously this mess existed but thankfully no more:
  // https://en.wikipedia.org/wiki/India%E2%80%93Bangladesh_enclaves
  // additionally it seems like the second order enclaves that do still exists (NL and AE)
  // are mapped such that they are outers that live inside the inners (basically multipolygon)
  for (const auto& inner : inners) {
    auto area = boost::geometry::area(inner);
    bool found = false;
    for (auto& poly : polys) {
      // is this the smallest polygon that can contain this inner?
      if (poly.area > area && boost::geometry::within(inner, poly.polygon)) {
        poly.polygon.inners().emplace_back(std::move(inner));
        found = true;
        break;
      }
      if (!found)
        LOG_WARN("Inner found without an outer");
    }
  }

  // Make a simple container of multiple polygons
  multipolygon_t multipolygon;
  multipolygon.reserve(polys.size());
  for (auto& poly : polys) {
    multipolygon_t buffered;
    boost::geometry::correct(poly.polygon);
    // boost::geometry::buffer(poly.polygon, buffered, 0);
    buffer_polygon(poly.polygon, multipolygon);
  }
  return multipolygon;
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
  OSMAdminData admin_data = PBFAdminParser::Parse(pt, input_files);

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

  // initialize geos
  geos_helper_t::get();

  // for each admin area (relation)
  uint32_t count = 0;
  for (const auto& admin : admin_data.admins) {
    auto name = admin_data.name_offset_map.name(admin.name_index);
    LOG_DEBUG("Building admin: " + name);

    // do inners and outers separately
    bool complete = true;
    std::vector<std::vector<ring_t>> outers_inners;
    for (bool outer : {true, false}) {
      // grab the ring segments and a lookup to find them when connecting them
      std::vector<ring_t> lines;
      std::unordered_multimap<point_t, size_t> line_lookup;
      if (!to_segments(admin_data, admin, outer, lines, line_lookup)) {
        complete = false;
        break;
      }
      // connect them into a series of one or more rings
      outers_inners.emplace_back(to_rings(lines, line_lookup));
    }

    // if we didn't have a complete relation (ie some members were missing) we bail
    if (!complete || outers_inners.front().empty()) {
      LOG_WARN("Degenerate admin: " + name);
      continue;
    }

    // convert the rings into multipolygons
    auto multipolygon = to_multipolygon(outers_inners.front(), outers_inners.back());

    // convert that into wkt format so we can put it into sqlite
    std::stringstream ss;
    ss << boost::geometry::wkt(multipolygon);
    auto wkt = ss.str();
    if (wkt.empty())
      continue;

    // load it into sqlite
    count++;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    sqlite3_bind_int(stmt, 1, admin.admin_level);

    std::string iso;
    if (admin.iso_code_index) {
      iso = admin_data.name_offset_map.name(admin.iso_code_index);
      sqlite3_bind_text(stmt, 2, iso.c_str(), iso.length(), SQLITE_STATIC);
    } else {
      sqlite3_bind_null(stmt, 2);
    }

    sqlite3_bind_null(stmt, 3);

    name = admin_data.name_offset_map.name(admin.name_index);
    sqlite3_bind_text(stmt, 4, name.c_str(), name.length(), SQLITE_STATIC);

    std::string name_en;
    if (admin.name_en_index) {
      name_en = admin_data.name_offset_map.name(admin.name_en_index);
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
    LOG_ERROR("sqlite3_step() Name: " + admin_data.name_offset_map.name(admin.name_index));
    LOG_ERROR("sqlite3_step() Name:en: " + admin_data.name_offset_map.name(admin.name_en_index));
    LOG_ERROR("sqlite3_step() Admin Level: " + std::to_string(admin.admin_level));
    LOG_ERROR("sqlite3_step() Drive on Right: " + std::to_string(admin.drive_on_right));
    LOG_ERROR("sqlite3_step() Allow Intersection Names: " +
              std::to_string(admin.allow_intersection_names));
  }

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
