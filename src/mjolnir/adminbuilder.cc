#include "mjolnir/adminbuilder.h"
#include "mjolnir/adminconstants.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/sqlite3.h"

#include <boost/property_tree/ptree.hpp>
#include <geos_c.h>
#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

using geometry_t = std::unique_ptr<GEOSGeometry, decltype(&GEOSGeom_destroy)>;

using namespace valhalla::mjolnir;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

// a simple singleton to wrap geos setup and tear down
struct geos_helper_t {
  static const geos_helper_t& get() {
    static geos_helper_t singleton;
    return singleton;
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
 * @param inners if some kind of self intersection should cause inners to be created we push them here
 */
void buffer_ring(const ring_t& ring, std::vector<ring_t>& rings, std::vector<ring_t>& inners) {
  // for collecting polygons
  auto add = [&](auto* geos_poly) {
    rings.emplace_back(geos_helper_t::to_striped_container<ring_t>(GEOSGetExteriorRing(geos_poly)));
    for (int i = 0; i < GEOSGetNumInteriorRings(geos_poly); ++i) {
      auto* inner = GEOSGetInteriorRingN(geos_poly, i);
      inners.push_back(geos_helper_t::to_striped_container<ring_t>(inner));
    }
  };

  auto* outer_ring = geos_helper_t::from_striped_container(ring);
  auto* geos_poly = GEOSGeom_createPolygon(outer_ring, nullptr, 0);
  auto* buffered = GEOSBuffer(geos_poly, 0, 8);
  GEOSNormalize(buffered);
  auto geom_type = GEOSGeomTypeId(buffered);
  switch (geom_type) {
    case GEOS_POLYGON: {
      add(buffered);
      break;
    }
    case GEOS_MULTIPOLYGON: {
      for (int i = 0; i < GEOSGetNumGeometries(buffered); ++i) {
        auto* geom = GEOSGetGeometryN(buffered, i);
        if (GEOSGeomTypeId(geom) != GEOS_POLYGON)
          throw std::runtime_error("Unusable geometry type after buffering");
        add(geom);
      }
      break;
    }
    default:
      throw std::runtime_error("Unusable geometry type after buffering");
  }

  GEOSGeom_destroy(geos_poly);
  GEOSGeom_destroy(buffered);
}

/**
 * @param polygon       to buffer to fix self intersections
 * @param multipolygon  any resulting polygons are output here
 */
void buffer_polygon(const polygon_t& polygon, multipolygon_t& multipolygon) {
  // for collecting polygons
  auto add = [&](auto* geos_poly) {
    auto& poly = *multipolygon.emplace(multipolygon.end());
    poly.outer() = geos_helper_t::to_striped_container<ring_t>(GEOSGetExteriorRing(geos_poly));
    for (int i = 0; i < GEOSGetNumInteriorRings(geos_poly); ++i) {
      auto* inner = GEOSGetInteriorRingN(geos_poly, i);
      poly.inners().push_back(geos_helper_t::to_striped_container<ring_t>(inner));
    }
  };

  auto* outer_ring = geos_helper_t::from_striped_container(polygon.outer());
  std::vector<GEOSGeometry*> inner_rings;
  inner_rings.reserve(polygon.inners().size());

  // annoying circleci apple clang bug, not reproducible on any other machine..
  // https://github.com/valhalla/valhalla/pull/4500/files#r1445039739
  auto unused_size = std::to_string(polygon.inners().size());

  for (const auto& inner : polygon.inners()) {
    inner_rings.push_back(geos_helper_t::from_striped_container(inner));
  }
  auto* geos_poly = GEOSGeom_createPolygon(outer_ring, &inner_rings.front(), inner_rings.size());
  auto* buffered = GEOSBuffer(geos_poly, 0, 8);
  GEOSNormalize(buffered);
  auto geom_type = GEOSGeomTypeId(buffered);
  switch (geom_type) {
    case GEOS_POLYGON: {
      add(buffered);
      break;
    }
    case GEOS_MULTIPOLYGON: {
      for (int i = 0; i < GEOSGetNumGeometries(buffered); ++i) {
        auto* geom = GEOSGetGeometryN(buffered, i);
        if (GEOSGeomTypeId(geom) != GEOS_POLYGON)
          throw std::runtime_error("Unusable geometry type after buffering");
        add(geom);
      }
      break;
    }
    default:
      throw std::runtime_error("Unusable geometry type after buffering with inners size " +
                               unused_size);
  }
  GEOSGeom_destroy(geos_poly);
  GEOSGeom_destroy(buffered);
}

/**
 * Create line segments from the way members of a given admin either for outers or inners
 * If a given admin is incomplete (eg. missing members) no segments are returned
 * @param admin_data   used to look up ways shape (nodes)
 * @param admin        the admin for which we are building the lookup
 * @param name         the name of the admin handy for error reporting
 * @param outer        whether or not we should build a lookup for outers or inners
 * @returns            the line segments contained in the admins ways
 */
geometry_t to_multilinestring(const OSMAdminData& admin_data,
                              const OSMAdmin& admin,
                              const std::string& name,
                              bool outer) {
  std::vector<GEOSGeometry*> segments;
  segments.reserve(admin.ways.size());
  unsigned int coord_count = 0;

  // get all the individual members of the admin relation merged into one ring
  auto role_itr = admin.roles.begin();
  for (const auto memberid : admin.ways) {
    // skip roles we arent interested in
    if (*(role_itr++) != outer)
      continue;

    // A relation may be included in an extract but it's members may not
    // Example:  PA extract can contain an NY relation but wont have all its members
    auto w_itr = admin_data.way_map.find(memberid);
    if (w_itr == admin_data.way_map.end()) {
      LOG_WARN(name + " (" + std::to_string(admin.id) + ") is missing way member " +
               std::to_string(memberid));
      // since no one owns these we have to deallocate them manually
      std::for_each(segments.begin(), segments.end(), GEOSGeom_destroy);
      return {nullptr, nullptr};
    }

    // build the line geom
    unsigned int i = 0;
    GEOSCoordSequence* sequence = GEOSCoordSeq_create(w_itr->second.size(), 2);
    for (const auto node_id : w_itr->second) {
      // although unlikely, we could have the way but not all the nodes
      auto n_itr = admin_data.shape_map.find(node_id);
      if (n_itr == admin_data.shape_map.end()) {
        LOG_WARN(name + " (" + std::to_string(admin.id) + ") with way member " +
                 std::to_string(memberid) + " is missing node " + std::to_string(node_id));
        // since no one owns these we have to deallocate them manually
        std::for_each(segments.begin(), segments.end(), GEOSGeom_destroy);
        return {nullptr, nullptr};
      }
      GEOSCoordSeq_setX(sequence, i, n_itr->second.first);
      GEOSCoordSeq_setY(sequence, i, n_itr->second.second);
      ++coord_count;
    }
    segments.emplace_back(GEOSGeom_createLineString(sequence));
  }

  // this shouldnt happen
  if (coord_count == 0) {
    LOG_WARN(name + " (" + std::to_string(admin.id) + ") with no usable ways");
    return {nullptr, nullptr};
  }

  // this collection takes ownership of all the unowned geoms we created above
  return {GEOSGeom_createCollection(GEOS_MULTILINESTRING, &segments[0], coord_count),
          GEOSGeom_destroy};
}

struct polygon_data {
  polygon_t polygon;
  polygon_t::inner_container_type postponed_inners;
  double area;
  bool operator<(const polygon_data& p) const {
    return area < p.area;
  }
};

/**
 * Takes outer and inner rings and combines them first into polygons and finally into a multipolygon
 * @param admin_info  a simple pair of name and relation id used for logging
 * @param outers      outer rings of polygons
 * @param inners      inner rings of polygons
 * @return the multipolygon of the combined outer and inner rings
 */
geometry_t to_multipolygon(const std::pair<std::string, uint64_t>& admin_info,
                           geometry_t& outers,
                           geometry_t& inners) {
  // Associate an area with each outer so we can
  std::vector<polygon_data> polys;
  for (auto& outer : outers) {
    polygon_data pd{};
    pd.polygon.outer() = std::move(outer);
    pd.area = boost::geometry::area(pd.polygon);
    polys.emplace_back(std::move(pd));
  }

  // sort ascending by area
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
      if (poly.area > area && boost::geometry::covered_by(inner, poly.polygon)) {
        poly.postponed_inners.emplace_back(std::move(inner));
        found = true;
        break;
      }
    }
    if (!found) {
      LOG_WARN("Inner with no outer " + admin_info.first + " (" + std::to_string(admin_info.second) +
               ") " + " near lat,lon " + std::to_string(inner.front().y()) + "," +
               std::to_string(inner.front().x()));
    }
  }
}

  // Make a simple container of multiple polygons
  multipolygon_t multipolygon;
  multipolygon.reserve(polys.size());
  for (auto& poly : polys) {
    multipolygon_t buffered;
    poly.polygon.inners().swap(poly.postponed_inners);
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
bool BuildAdminFromPBF(const boost::property_tree::ptree& pt,
                       const std::vector<std::string>& input_files) {
  // Bail if bad path
  auto database = pt.get_optional<std::string>("admin");

  if (!database) {
    LOG_ERROR("Admin config info not found. Admins will not be created.");
    return false;
  }

  const std::filesystem::path parent_dir = std::filesystem::path(*database).parent_path();
  if (!std::filesystem::exists(parent_dir) && !std::filesystem::create_directories(parent_dir)) {
    LOG_ERROR("Can't create parent directory " + parent_dir.string());
    return false;
  }

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  OSMAdminData admin_data = PBFAdminParser::Parse(pt, input_files);

  sqlite3_stmt* stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // In-memory database that will be dumped to disk at the end
  auto db = Sqlite3::open(":memory:", SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
  // TODO: these blocks are the same like 20 times or so
  // let's abstract the sqlite commands somewhere
  if (!db) {
    LOG_ERROR("cannot create in-memory database");
    return false;
  }

  /* creating an admin POLYGON table */
  sql = "SELECT InitSpatialMetaData(1); CREATE TABLE admins (";
  sql += "admin_level INTEGER NOT NULL,";
  sql += "iso_code TEXT,";
  sql += "parent_admin INTEGER,";
  sql += "name TEXT NOT NULL,";
  sql += "name_en TEXT,";
  sql += "drive_on_right INTEGER NULL,";
  sql += "allow_intersection_names INTEGER NULL,";
  sql += "default_language TEXT,";
  sql += "supported_languages TEXT)";

  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  /* creating a MULTIPOLYGON Geometry column */
  sql = "SELECT AddGeometryColumn('admins', ";
  sql += "'geom', 4326, 'MULTIPOLYGON', 2)";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  LOG_INFO("Created admin table.");

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

  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  LOG_INFO("Created admin access table.");
  LOG_INFO("Start populating admin tables.");

  /*
   * inserting some MULTIPOLYGONs
   * this time too we'll use a Prepared Statement
   */
  sql = "INSERT INTO admins (admin_level, iso_code, parent_admin, name, name_en, ";
  sql +=
      "drive_on_right, allow_intersection_names, default_language, supported_languages, geom) VALUES (?,?,?,?,?,?,?,?,?,";
  sql += "ST_MakeValid(CastToMulti(GeomFromText(?, 4326))))";

  ret = sqlite3_prepare_v2(db->get(), sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db->get())));
  }
  ret = sqlite3_exec(db->get(), "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  // initialize geos
  geos_helper_t::get();

  // for each admin area (relation)
  [[maybe_unused]] uint32_t count = 0;
  for (const auto& admin : admin_data.admins) {
    std::pair<std::string, uint64_t> admin_info(admin_data.name_offset_map.name(admin.name_index),
                                                admin.id);
    LOG_DEBUG("Building admin: " + admin_info.first);

    // do inners and outers separately
    bool complete = true;
    geometry_t outers{nullptr, GEOSGeom_destroy};
    geometry_t inners{nullptr, GEOSGeom_destroy};
    for (auto oi : {std::make_pair(true, &outers), std::make_pair(false, &inners)}) {
      // grab the ring segments and a lookup to find them when connecting them
      auto multilinestring = to_multilinestring(admin_data, admin, admin_info.first, oi.first);
      if (!multilinestring) {
        complete = false;
        break;
      }
      // connect them into a series of one or more rings
      oi.second->reset(GEOSLineMerge(multilinestring.get()));
    }

    // if we didn't have a complete relation (ie some members were missing) we bail
    if (!complete || !outers) {
      LOG_WARN(admin_info.first + " (" + std::to_string(admin_info.second) +
               ") is degenerate and will be skipped");
      continue;
    }

    // convert the rings into multipolygons
    auto multipolygon = to_multipolygon(admin_info, outers, inners);

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

    sqlite3_bind_text(stmt, 4, admin_info.first.c_str(), admin_info.first.length(), SQLITE_STATIC);

    std::string name_en, default_language;
    if (admin.name_en_index) {
      name_en = admin_data.name_offset_map.name(admin.name_en_index);
      sqlite3_bind_text(stmt, 5, name_en.c_str(), name_en.length(), SQLITE_STATIC);
    } else {
      sqlite3_bind_null(stmt, 5);
    }

    uint32_t level = admin.admin_level;
    if (level == 2 || level == 4)
      sqlite3_bind_int(stmt, 6, admin.drive_on_right);
    else
      sqlite3_bind_null(stmt, 6);

    if (level == 2 || level == 4)
      sqlite3_bind_int(stmt, 7, admin.allow_intersection_names);
    else
      sqlite3_bind_null(stmt, 7);

    if (admin.default_language_index) {
      default_language = admin_data.name_offset_map.name(admin.default_language_index);
      sqlite3_bind_text(stmt, 8, default_language.c_str(), default_language.length(), SQLITE_STATIC);
    } else {
      sqlite3_bind_null(stmt, 8);
    }

    sqlite3_bind_null(stmt, 9);
    sqlite3_bind_text(stmt, 10, wkt.c_str(), wkt.length(), SQLITE_STATIC);

    /* performing INSERT INTO */
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db->get())));
    LOG_ERROR("sqlite3_step() Name: " + admin_data.name_offset_map.name(admin.name_index));
    LOG_ERROR("sqlite3_step() Name:en: " + admin_data.name_offset_map.name(admin.name_en_index));
    LOG_ERROR("sqlite3_step() Admin Level: " + std::to_string(admin.admin_level));
    LOG_ERROR("sqlite3_step() Drive on Right: " + std::to_string(admin.drive_on_right));
    LOG_ERROR("sqlite3_step() Allow Intersection Names: " +
              std::to_string(admin.allow_intersection_names));
    LOG_ERROR("sqlite3_step() Default Language: " +
              admin_data.name_offset_map.name(admin.default_language_index));
  }

  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db->get(), "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Inserted " + std::to_string(count) + " admin areas");

  sql = "SELECT CreateSpatialIndex('admins', 'geom')";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Created spatial index");

  sql = "CREATE INDEX IdxLevel ON admins (admin_level)";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Created Level index");

  sql = "UPDATE admins AS child SET parent_admin = (SELECT parent.rowid from admins";
  sql +=
      " AS parent WHERE parent.admin_level < child.admin_level AND ST_Covers(parent.geom, child.geom) ";
  sql += " ORDER BY parent.admin_level DESC LIMIT 1)"; // if multiple candidates, get the one with the
                                                       // lowest admin_level
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Done updating parent admin");

  sql = "UPDATE admins AS child SET drive_on_right = (SELECT parent.drive_on_right FROM ";
  sql += "admins AS parent WHERE parent.rowid = child.parent_admin) WHERE parent_admin IS NOT NULL;";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Done updating drive on right column.");

  sql = "CREATE INDEX IdxDriveOnRight ON admins (drive_on_right)";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Created Drive On Right index");

  sql =
      "UPDATE admins AS child SET allow_intersection_names = (SELECT parent.allow_intersection_names ";
  sql +=
      "FROM admins AS parent WHERE parent.rowid = child.parent_admin) WHERE parent_admin IS NOT NULL;";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Done updating allow intersection names column.");

  sql = "CREATE INDEX IdxAllowIntersectionNames ON admins (allow_intersection_names)";
  ret = sqlite3_exec(db->get(), sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  LOG_INFO("Created allow intersection names index");

  sql = "update admins set supported_languages = ? ";
  sql += "where (name = ? or name_en = ?) and admin_level = ? ";

  ret = sqlite3_prepare_v2(db->get(), sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db->get())));
  }
  ret = sqlite3_exec(db->get(), "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  for (const auto& languages : kSupportedLanguages) {

    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    sqlite3_bind_text(stmt, 1, languages.second.second.c_str(), languages.second.second.length(),
                      SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, languages.first.c_str(), languages.first.length(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, languages.first.c_str(), languages.first.length(), SQLITE_STATIC);
    sqlite3_bind_int(stmt, 4, (int)languages.second.first);

    /* performing update */
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("Supported Languages: sqlite3_step() error: " + std::string(sqlite3_errmsg(db->get())) +
              ".  Ignore if not using a planet extract or check if there was a name change for " +
              languages.first.c_str());
  }

  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db->get(), "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  LOG_INFO("Done updating supported languages");

  sql = "INSERT into admin_access (admin_id, iso_code, trunk, trunk_link, track, footway, ";
  sql += "pedestrian, bridleway, cycleway, path, motorroad) VALUES (";
  sql += "(select rowid from admins where (name = ? or name_en = ?)), ";
  sql += "(select iso_code from admins where (name = ? or name_en = ?)), ";
  sql += "?, ?, ?, ?, ?, ?, ?, ?, ?)";

  ret = sqlite3_prepare_v2(db->get(), sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db->get())));
  }
  ret = sqlite3_exec(db->get(), "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
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
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db->get())) +
              ".  Ignore if not using a planet extract or check if there was a name change for " +
              access.first.c_str());
  }

  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db->get(), "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }

  LOG_INFO("Writing database to disk.");
  if (std::filesystem::exists(*database)) {
    std::filesystem::remove(*database);
  }

  auto db_on_disk = Sqlite3::open((*database).c_str(), SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
  if (!db_on_disk) {
    LOG_ERROR("cannot open " + (*database));
    return false;
  }

  sqlite3_backup* backup = sqlite3_backup_init(db_on_disk->get(), "main", db->get(), "main");
  if (backup) {
    sqlite3_backup_step(backup, -1);
    sqlite3_backup_finish(backup);
  }
  ret = sqlite3_errcode(db_on_disk->get());
  if (ret != SQLITE_OK) {
    LOG_ERROR("failed to save database to " + (*database));
    return false;
  }

  LOG_INFO("Finished.");

  return true;
}

} // namespace mjolnir

} // namespace valhalla
