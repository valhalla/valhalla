#include <string>
#include <vector>

#include "pbfgraphbuilder.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/graphoptimizer.h"
#include "config.h"

#include <sqlite3.h>
#include <spatialite.h>
/* Need to know which geos version we have to work out which headers to include */
#include <geos/version.h>

#include <geos/geom/GeometryFactory.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/LineString.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/MultiPolygon.h>
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>
#include <geos/util/GEOSException.h>
#include <geos/opLinemerge.h>
using namespace geos::geom;
using namespace geos::io;
using namespace geos::operation::linemerge;

// For OSM pbf reader
using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include <valhalla/midgard/logging.h>

namespace bpo = boost::program_options;
using namespace valhalla::midgard;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
      "pbfadminbuilder " VERSION "\n"
      "\n"
      " Usage: pbfadminbuilder [options] <protocolbuffer_input_file>\n"
      "\n"
      "pbfadminbuilder is a program that creates the route graph from a osm.pbf "
      "extract or osm2pgsql import.  You should use the lua scripts provided for "
      "either method.  The scripts are located in the ./import/osm2pgsql directory.  "
      "Moreover, sample json cofigs are located in ./import/configs directory."
      "\n"
      "\n");

  options.add_options()
          ("help,h", "Print this help message.")
          ("version,v", "Print the version of this software.")
          ("config,c",
              boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
              "Path to the json configuration file.")
              // positional arguments
              ("input_files", boost::program_options::value<std::vector<std::string> >(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
          << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
          << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  if (vm.count("version")) {
    std::cout << "pbfadminbuilder " << VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path))
      return true;
    else
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
  }

  return false;
}

namespace {

struct polygondata
{
  Polygon*        polygon;
  LinearRing*     ring;
  double          area;
  int             iscontained;
  unsigned        containedbyid;
};

int polygondata_comparearea(const void* vp1, const void* vp2)
{
  const polygondata* p1 = (const polygondata*)vp1;
  const polygondata* p2 = (const polygondata*)vp2;

  if (p1->area == p2->area) return 0;
  if (p1->area > p2->area) return -1;
  return 1;
}
} // anonymous namespace

std::vector<std::string> GetWkts(const std::vector<PointLL>& shape) {
  std::vector<std::string> wkts;

  GeometryFactory gf;

  try {

    std::vector<std::string> wkts;
    std::unique_ptr<Geometry> geom;
    std::unique_ptr<std::vector<Geometry*> > lines(new std::vector<Geometry*>);
    std::unique_ptr<CoordinateSequence> coords(gf.getCoordinateSequenceFactory()->create((size_t)0, (size_t)2));

    for (const auto& ll : shape) {

      Coordinate c;
      c.x = ll.lng();
      c.y = ll.lat();
      coords->add(c, 0);
    }

    if (coords->getSize() > 1) {
      geom = std::unique_ptr<Geometry>(gf.createLineString(coords.release()));
      lines->emplace_back(geom.release());
    }

    std::unique_ptr<Geometry> mline (gf.createMultiLineString(lines.release()));
    LineMerger merger;
    merger.add(mline.get());
    std::unique_ptr<std::vector<LineString *> > merged(merger.getMergedLineStrings());
    WKTWriter writer;

    // Procces ways into lines or simple polygon list
    polygondata* polys = new polygondata[merged->size()];

    unsigned totalpolys = 0;
    for (unsigned i=0 ;i < merged->size(); ++i) {
      std::unique_ptr<LineString> pline ((*merged ) [i]);
      if (pline->getNumPoints() > 3 && pline->isClosed()) {
        polys[totalpolys].polygon = gf.createPolygon(gf.createLinearRing(pline->getCoordinates()),0);
        polys[totalpolys].ring = gf.createLinearRing(pline->getCoordinates());
        polys[totalpolys].area = polys[totalpolys].polygon->getArea();
        polys[totalpolys].iscontained = 0;
        polys[totalpolys].containedbyid = 0;
        if (polys[totalpolys].area > 0.0)
          totalpolys++;
        else {
          delete(polys[totalpolys].polygon);
          delete(polys[totalpolys].ring);
        }
      }
    }

    if (totalpolys) {
      qsort(polys, totalpolys, sizeof(polygondata), polygondata_comparearea);

      unsigned toplevelpolygons = 0;
      int istoplevelafterall;

      for (unsigned i=0 ;i < totalpolys; ++i) {
        if (polys[i].iscontained != 0)
          continue;

        toplevelpolygons++;

        for (unsigned j=i+1; j < totalpolys; ++j) {
          // Does polygon[i] contain the smaller polygon[j]?
              if (polys[j].containedbyid == 0 && polys[i].polygon->contains(polys[j].polygon)) {
                // are we in a [i] contains [k] contains [j] situation
                // which would actually make j top level
                istoplevelafterall = 0;
                for (unsigned k=i+1; k < j; ++k) {
                  if (polys[k].iscontained && polys[k].containedbyid == i &&
                      polys[k].polygon->contains(polys[j].polygon)) {
                    istoplevelafterall = 1;
                    break;
                  }
                }
                if (istoplevelafterall == 0) {
                  polys[j].iscontained = 1;
                  polys[j].containedbyid = i;
                }
              }
        }
      }
      // polys now is a list of polygons tagged with which ones are inside each other

      // List of polygons for multipolygon
      std::unique_ptr<std::vector<Geometry*> > polygons(new std::vector<Geometry*>);
      // Make a multipolygon if required
      if (toplevelpolygons > 1)
      {
        std::unique_ptr<Geometry> multipoly(gf.createMultiPolygon(polygons.release()));
        if (!multipoly->isValid()) {
          multipoly = std::unique_ptr<Geometry>(multipoly->buffer(0));
        }
        multipoly->normalize();

        if (multipoly->isValid())
          wkts.emplace_back(writer.write(multipoly.get()));
      }
      // For each top level polygon create a new polygon including any holes
      /* for (unsigned i=0 ;i < totalpolys; ++i) {
            if (polys[i].iscontained != 0) continue;

            // List of holes for this top level polygon
            std::unique_ptr<std::vector<Geometry*> > interior(new std::vector<Geometry*>);
            for (unsigned j=i+1; j < totalpolys; ++j) {
              if (polys[j].iscontained == 1 && polys[j].containedbyid == i)
                interior->emplace_back(polys[j].ring);
            }

            Polygon* poly(gf.createPolygon(polys[i].ring, interior.release()));
            poly->normalize();
            polygons->emplace_back(poly);
          }

          for(unsigned i=0; i<toplevelpolygons; i++) {
            Geometry* poly = dynamic_cast<Geometry*>(polygons->at(i));
            if (!poly->isValid()) {
              poly = dynamic_cast<Geometry*>(poly->buffer(0));
              poly->normalize();
            }
            if (poly->isValid())
              wkts.emplace_back(writer.write(poly));

            delete(poly);
          }*/
    }

    for (unsigned i=0; i < totalpolys; ++i)
      delete(polys[i].polygon);

    delete[](polys);

  }
  catch (std::exception& e)
  {
    LOG_ERROR("Standard exception processing relation: " + std::string(e.what()));
  }
  catch (...)
  {
    LOG_ERROR("Exception caught processing relations.");
  }

  return wkts;
}

/**
 * Build admins from protocol buffer input.
 */
void BuildAdminFromPBF(const boost::property_tree::ptree& pt,
                       const std::vector<std::string>& input_files) {

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  OSMData osmdata = PBFAdminParser::Parse(pt, input_files);

  std::string dir = pt.get<std::string>("admin.admin_dir");
  std::string db_name = pt.get<std::string>("admin.db_name");

  std::string database = dir + "/" +  db_name;

  if (boost::filesystem::exists(database)) {
    boost::filesystem::remove(database);
  }

  spatialite_init(0);

  sqlite3 *db_handle;
  sqlite3_stmt *stmt;
  uint32_t ret;
  char *err_msg = NULL;
  std::string sql;

  ret = sqlite3_open_v2(database.c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + database);
    sqlite3_close(db_handle);
    db_handle = NULL;
    return;
  }

  // loading SpatiaLite as an extension
  sqlite3_enable_load_extension(db_handle, 1);
  sql = "SELECT load_extension('libspatialite.so')";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("load_extension() error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("SpatiaLite loaded as an extension");

  /* creating an admin POLYGON table */
  sql = "SELECT InitSpatialMetaData(); CREATE TABLE admins (";
  sql += "id INTEGER NOT NULL PRIMARY KEY,";
  sql += "name TEXT NOT NULL)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  /* creating a POLYGON Geometry column */
  sql = "SELECT AddGeometryColumn('admins', ";
  sql += "'geom', 4326, 'MULTIPOLYGON', 'XY')";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  LOG_INFO("Created admin table.");

  /*
   * inserting some POLYGONs
   * this time too we'll use a Prepared Statement
   */
  sql = "INSERT INTO admins (id, name, geom) ";
  sql += "VALUES (?, ?, GeomFromText(?, 4326))";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), strlen (sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
  }
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
  }

  uint32_t count = 0;
  uint64_t nodeid;
  bool has_data;

  for (const auto admin : osmdata.admins_) {

    for (size_t i = 0; i < admin.member_count(); i++) {

      const uint64_t &memberid = osmdata.memberids_[admin.member_index() + i];
      const OSMWay* w = osmdata.GetWay(memberid);

      // A relation may be included in an extract but it's members may not.
      // Example:  PA extract can contain a NY relation.
      if (w == nullptr) {
        has_data = false;
        break;
      }

      std::vector<PointLL> shape;
      size_t j = 0;

      bool done = false;
      while (!done) {
        nodeid = osmdata.noderefs[w->noderef_index() + j];
        const OSMNode& osmnode = *osmdata.GetNode(nodeid);

        shape.emplace_back(osmnode.latlng());
        j++;

        if (j == w->node_count())
          done = true;
      }
      std::vector<std::string> wkts = GetWkts(shape);
      for (const auto& wkt : wkts) {

        count++;
        sqlite3_reset (stmt);
        sqlite3_clear_bindings (stmt);
        sqlite3_bind_int (stmt, 1, count);
        sqlite3_bind_text (stmt, 2, admin.name().c_str(), admin.name().length(), SQLITE_STATIC);
        sqlite3_bind_text (stmt, 3, wkt.c_str(), wkt.length(), SQLITE_STATIC);
        /* performing INSERT INTO */
        ret = sqlite3_step (stmt);
        if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
          continue;
        }
        LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
      }
    }
  }

  sqlite3_finalize (stmt);
  ret = sqlite3_exec (db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);

  }
  LOG_INFO("Inserted " + std::to_string(count) + " admin areas");

  sql = "SELECT CreateSpatialIndex('admins', 'geom')";
  ret = sqlite3_exec (db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
    return;
  }

  LOG_INFO("Created spatial index");
  sqlite3_close (db_handle);
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  //check what type of input we are getting
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("mjolnir.logging");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  //we only support protobuf at present
  std::string input_type = pt.get<std::string>("mjolnir.input.type");
  if(input_type == "protocolbuffer"){
    BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);
  }/*else if("postgres"){
    //TODO
    if (v.first == "host")
      host = v.second.get_value<std::string>();
    else if (v.first == "port")
      port = v.second.get_value<unsigned int>();
    else if (v.first == "username")
      username = v.second.get_value<std::string>();
    else if (v.first == "password")
      password = v.second.get_value<std::string>();
    else
      return false;  //unknown value;
  }*/

  return EXIT_SUCCESS;
}

