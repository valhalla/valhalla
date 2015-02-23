#include <string>
#include <vector>
#include <cstdio>
#include <iostream>

#include "pbfgraphbuilder.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/osmadmin.h"
#include "config.h"
#include "mjolnir/graphbuilder.h"

// For OSM pbf reader
using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include <sqlite3.h>
#include <spatialite.h>

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

/**
 * Parse PBF into the supplied data structures
 */
void ParsePBFSaveToDb(const boost::property_tree::ptree& pt,
                const std::vector<std::string>& input_files) {
  PBFAdminParser parser(pt);
  OSMData osmdata = parser.Load(input_files);

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
  sql += "'geom', 4326, 'POLYGON', 'XY')";
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
  std::string geom;
  uint64_t nodeid,old,lastid = 0;
  bool has_data = false, reverse = false;

  for (const auto admin : parser.admins_) {

    lastid = 0;
    has_data = false;
    // setting up values / binding
    geom = "POLYGON((";

    for (size_t i = 0; i < admin.member_count(); i++) {

      const uint64_t &memberid = parser.memberids_[admin.member_index() + i];
      const auto& iter = parser.admin_ways_.find(memberid);

      // A relation may be included in an extract but it's members may not.
      // Example:  PA extract can contain a NY relation.
      if (iter == parser.admin_ways_.end()) {
        has_data = false;
        break;
      }

      const OSMWay &w  = iter->second;

      reverse = false;

      nodeid = osmdata.noderefs[w.noderef_index()];

      size_t j = w.node_count() - 1;

      if (osmdata.noderefs[w.noderef_index()] ==
          osmdata.noderefs[w.noderef_index() + j] && has_data) {

        geom += "))";

        count++;
        sqlite3_reset (stmt);
        sqlite3_clear_bindings (stmt);
        sqlite3_bind_int (stmt, 1, count);
        sqlite3_bind_text (stmt, 2, admin.name().c_str(), admin.name().length(), SQLITE_STATIC);
        sqlite3_bind_text (stmt, 3, geom.c_str(), geom.length(), SQLITE_STATIC);
        /* performing INSERT INTO */
        ret = sqlite3_step (stmt);
        if (ret != SQLITE_DONE && ret != SQLITE_ROW)
          LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));

        std::cout << geom << std::endl;


        geom = "POLYGON((";

        has_data = false;

      }

      j = 0;
      if (lastid != 0 && lastid != nodeid) {
        reverse = true;
        j = w.node_count() - 1;

        if (lastid != osmdata.noderefs[w.noderef_index() + j])
          std::cout << old << " " << memberid << std::endl;

      }

      bool done = false;

      while (!done) {

        nodeid = osmdata.noderefs[w.noderef_index() + j];

        const auto& iter = osmdata.admin_nodes.find(nodeid);

       // OSMNode osmnode = osmdata.GetNode(nodeid);

        // TODO
        if (iter == osmdata.admin_nodes.end())
        {
          has_data = false;
          break;
        }

        lastid = nodeid;

        const auto& osmnode = iter->second;

        const PointLL& ll = osmnode.latlng();

        if (has_data)
          geom += ", ";
        geom += std::to_string(ll.lng()) + " " + std::to_string(ll.lat());
        has_data = true;

        if (reverse) {
          j--;
          if (j == -1)
            done = true;
        } else {
          j++;
          if (j == w.node_count())
            done = true;
        }
      }
      old = memberid;
    }

    geom += "))";

    if (!has_data)
      continue;

    count++;
    sqlite3_reset (stmt);
    sqlite3_clear_bindings (stmt);
    sqlite3_bind_int (stmt, 1, count);
    sqlite3_bind_text (stmt, 2, admin.name().c_str(), admin.name().length(), SQLITE_STATIC);
    sqlite3_bind_text (stmt, 3, geom.c_str(), geom.length(), SQLITE_STATIC);
    /* performing INSERT INTO */
    ret = sqlite3_step (stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW)
      continue;
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));

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

/**
 * Build admins from protocol buffer input.
 */
void BuildAdminFromPBF(const boost::property_tree::ptree& pt,
               const std::vector<std::string>& input_files) {

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  ParsePBFSaveToDb(pt, input_files);
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
  }

  return EXIT_SUCCESS;
}

