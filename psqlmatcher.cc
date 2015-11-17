// -*- mode: c++ -*-

#include <unordered_map>
#include <vector>
#include <cassert>
#include <iostream>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// psql
#include <postgresql/libpq-fe.h>
#include <pqxx/pqxx>

// geos
#include <geos/io/WKBReader.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/Geometry.h>

// sqlite3
#include <sqlite3.h>

// valhalla
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla;

#include "costings.h"
#include "map_matching.h"


namespace {
constexpr float kDefaultSigmaZ = 4.07;
constexpr float kDefaultBeta = 3;
constexpr float kDefaultSquaredSearchRadius = 25 * 25;  // 25 meters
}


using BoundingBox = midgard::AABB2<midgard::PointLL>;
using Sequence = std::vector<Measurement>;


inline std::string
joinbbox(const BoundingBox& bbox)
{
  return std::to_string(bbox.minx())
      + " "
      + std::to_string(bbox.miny())
      + ", "
      + std::to_string(bbox.maxx())
      + " "
      + std::to_string(bbox.maxy());
}


// Convert a linestring's Well-Known Binary to a sequence (a vector of
// measurements)
Sequence to_sequence(std::istringstream& wkb)
{
  geos::geom::GeometryFactory factory;
  geos::io::WKBReader reader(factory);
  auto geometry = reader.read(wkb);
  Sequence sequence;

  auto coords = geometry->getCoordinates();
  for (decltype(coords->size()) idx = 0; idx < coords->size(); idx++) {
    const auto& coord = coords->getAt(idx);
    sequence.emplace_back(PointLL{coord.x, coord.y});
  }

  return sequence;
}


inline Sequence to_sequence(const std::string& wkb)
{
  std::istringstream is(wkb);
  return to_sequence(is);
}


using SequenceId = uint32_t;


// Query all sequences within the bounding box
std::unordered_map<SequenceId, Sequence>
query_sequences(pqxx::connection& conn, const BoundingBox& bbox)
{
  pqxx::work txn(conn);

  auto bbox_clause = txn.quote("BOX(" + joinbbox(bbox) + ")") + "::box2d";
  std::string statement = "SELECT id, ST_AsBinary(path_gm) AS geom FROM sequences WHERE "
                          + bbox_clause + " && path_gm"
                          + " LIMIT " + std::to_string(std::numeric_limits<uint32_t>::max());
  LOG_INFO("Querying: " + statement);

  // Send query
  auto tuples = txn.exec(statement);

  std::unordered_map<SequenceId, Sequence> sequences;
  for (auto it = tuples.begin(); it != tuples.end(); it++) {
    auto sid = it->at("id").as<SequenceId>();
    auto bs = pqxx::binarystring(it->at("geom"));
    sequences[sid] = to_sequence(bs.str());
  }

  return sequences;
}


// Collect all tile IDs
std::vector<uint32_t>
collect_local_tileids(const baldr::TileHierarchy& tile_hierarchy,
                      const std::unordered_set<uint32_t>& excluded_tileids)
{
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;

  std::vector<uint32_t> queue;

  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId tile_id(id, local_level, 0);
    if (baldr::GraphReader::DoesTileExist(tile_hierarchy, tile_id)
        && excluded_tileids.find(id) == excluded_tileids.end()) {
      queue.push_back(tile_id.tileid());
    }
  }

  return queue;
}


// Tell you which tile this sequence belongs to
inline uint32_t
which_tileid(const baldr::TileHierarchy& tile_hierarchy,
             const Sequence& sequence)
{
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto graphid = tile_hierarchy.GetGraphId(sequence[0].lnglat(), local_level);
  if (graphid.Is_Valid()) {
    return graphid.tileid();
  }
  baldr::GraphId id;
  return id.tileid();
}


bool create_tiles_table(sqlite3* db_handle)
{
  char *err_msg;
  std::string sql = "CREATE TABLE IF NOT EXISTS tiles"
                    " (id INTEGER PRIMARY KEY, matched_count INTEGER, total_count INTEGER)";
  auto ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  return true;
}



// <sequence_id, coordinate_index, graphid, graphtype> no invalid
// graphid guaranteed
using Result = std::tuple<SequenceId, uint32_t, baldr::GraphId, GraphType>;


bool create_scores_table(sqlite3* db_handle)
{
  char *err_msg;
  std::string sql = "CREATE TABLE IF NOT EXISTS scores"
                    " (sequence_id INTEGER, coordinate_index INTEGER, graphid BIGINT, graphtype SMALLINT)";
  auto ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  return true;
}


bool read_finished_tiles(sqlite3* db_handle,
                         std::unordered_set<uint32_t>& tileids)
{
  sqlite3_stmt* stmt;
  std::string sql = "SELECT id FROM tiles";
  int ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, nullptr);
  if (SQLITE_OK != ret) {
    sqlite3_finalize(stmt);
    return false;
  }

  do {
    ret = sqlite3_step(stmt);
    if (SQLITE_ROW == ret) {
      int sequence_id = sqlite3_column_int(stmt, 0);
      if (sequence_id >= 0) {
        tileids.insert(static_cast<uint32_t>(sequence_id));
      } else {
        LOG_ERROR("FOUND negative sequence ID which is not good");
      }
    }
    // Try again if busy
  } while (SQLITE_ROW == ret || SQLITE_BUSY == ret);

  if (SQLITE_DONE != ret) {
    LOG_ERROR("Not so successfully: expect SQLITE_DONE returned");
    sqlite3_finalize(stmt);
    return false;
  }

  sqlite3_finalize(stmt);
  return true;
}


bool write_results(sqlite3* db_handle,
                   const std::vector<Result>& results,
                   uint32_t tileid,
                   uint32_t matched_count,
                   uint32_t total_count)
{
  std::string sql;

  sql += "BEGIN;\n";

  // Insert (sequence_id, coordinate_index, graphid, graphtype) into scores
  if (!results.empty()) {
    sql += "INSERT INTO scores VALUES ";
    for (const auto& result : results) {
      sql += "(";
      sql += std::to_string(std::get<0>(result)) + ", "; // sequence_id
      sql += std::to_string(std::get<1>(result)) + ", "; // coordinate_index
      sql += std::to_string(std::get<2>(result)) + ", "; // graphid
      sql += std::to_string(static_cast<uint8_t>(std::get<3>(result))); // graphtype
      sql += "),";
    }
    sql.pop_back();  // Pop out the last comma
    sql += ";\n";
  }

  // Insert (tileid, matched_count, total_count) into tiles
  sql += "INSERT INTO tiles VALUES (";
  sql += std::to_string(tileid) + ", ";
  sql += std::to_string(matched_count) + ", ";
  sql += std::to_string(total_count) + ");\n";

  sql += "END;";

  char *err_msg;
  int ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (SQLITE_OK != ret) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    return false;
  }
  return true;
}


int main(int argc, char *argv[])
{
  //////////////////////////////////
  // Parse arguments
  if (argc < 4) {
    std::cerr << "usage: psqlmatcher CONF_FILE_PATH PSQL_URI SQLITE3_FILE_PATH" << std::endl << std::endl;
    std::cerr << "example: psqlmatcher conf/valhalla.json \"dbname=sequence user=postgres password=secret host=localhost\" results.sqlite3" << std::endl;
    std::cerr << "It will read ALL GPS sequences from the psql database and write results into results.sqlite3." << std::endl;
    return 1;
  }

  std::string config_file_path(argv[1]);
  std::string psql_uri(argv[2]);
  std::string sqlite3_file_path(argv[3]);

  /////////////////////////////////
  // Initialize
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);
  std::shared_ptr<sif::DynamicCost> mode_costing[4] = {
    nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
    nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
    nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
    CreatePedestrianCost(*pt.get_child_optional("costing_options.pedestrian"))
  };
  sif::TravelMode travel_mode = static_cast<sif::TravelMode>(3);
  baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));
  // TODO read them from config
  auto sigma_z = kDefaultSigmaZ;
  auto beta = kDefaultBeta;
  MapMatching mm(sigma_z, beta, reader, mode_costing, travel_mode);

  const auto& tile_hierarchy = reader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  auto tile_size = tiles.TileSize();
  const CandidateGridQuery grid(reader, tile_size/1000, tile_size/1000);
  LOG_INFO("Config: tile size = " + std::to_string(tile_size));
  LOG_INFO("Config: sigma_z = " + std::to_string(sigma_z));
  LOG_INFO("Config: beta = " + std::to_string(beta));

  ////////////////////////
  // Prepare sqlite3 database for writing results
  sqlite3* db_handle;
  int ret = sqlite3_open_v2(sqlite3_file_path.c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr);
  if (SQLITE_OK != ret) {
    LOG_ERROR("failed to open sqlite3 database at " + sqlite3_file_path);
    return 2;
  }
  {
    bool ok = create_tiles_table(db_handle);
    if (!ok) {
      sqlite3_close(db_handle);
      return 2;
    }
  }
  {
    bool ok = create_scores_table(db_handle);
    if (!ok) {
      sqlite3_close(db_handle);
      return 2;
    }
  }

  ////////////////////////
  // Collect tiles
  std::unordered_set<uint32_t> finished_tileids;
  {
    bool ok = read_finished_tiles(db_handle, finished_tileids);
    if (!ok) {
      sqlite3_close(db_handle);
      return 2;
    }
  }
  auto tileids = collect_local_tileids(tile_hierarchy, finished_tileids);
  LOG_INFO("The number of tiles collected: " + std::to_string(tileids.size()) + " (excluded " + std::to_string(finished_tileids.size()) + " finished tiles)");

  //////////////////////
  // For each tile:
  //     1. Query all sequences within bounding box of this tile
  //     2. For each sequence:
  //            Offline match the sequence
  //     3. Insert rows [sequence's id, coordinate's index, edge's GraphId] into sqlite3

  uint64_t stat_matched_count_totally = 0;
  uint64_t stat_measurement_count_totally = 0;

  pqxx::connection conn(psql_uri.c_str());
  for (auto tileid : tileids) {
    std::vector<Result> results;

    uint32_t stat_matched_count_of_tile = 0;
    uint32_t stat_measurement_count_of_tile = 0;
    const auto& bbox = tiles.TileBounds(tileid);
    for (const auto& sequencepair : query_sequences(conn, bbox)) {
      auto sid = sequencepair.first;
      const auto& sequence = sequencepair.second;
      // Too verbose
      // LOG_INFO("Got sequence: id = " + std::to_string(sid) + " size = " + std::to_string(sequence.size()));

      if (sequence.empty()) {
        continue;
      }

      // Skip sequences that don't belong to this tile
      if (which_tileid(tile_hierarchy, sequence) != tileid) {
        continue;
      }

      const auto& match_results = OfflineMatch(mm, grid, sequence, kDefaultSquaredSearchRadius);
      assert(match_results.size() == sequence.size());

      uint32_t stat_matched_count_of_sequence = 0;
      uint32_t coord_idx = 0;
      for (auto result = match_results.cbegin(); result != match_results.cend(); result++, coord_idx++) {
        if (result->graphid().Is_Valid()) {
          results.emplace_back(sid, coord_idx, result->graphid(), result->graphtype());
          stat_matched_count_of_sequence++;
        }
      }

      stat_matched_count_of_tile += stat_matched_count_of_sequence;
      stat_measurement_count_of_tile += sequence.size();
      // Too verbose
      // LOG_INFO("Matched " + std::to_string(stat_matched_count_of_sequence) + "/" + std::to_string(sequence.size()));
    }

    stat_matched_count_totally += stat_matched_count_of_tile;
    stat_measurement_count_totally += stat_measurement_count_of_tile;

    {
      bool ok = write_results(db_handle, results, tileid, stat_matched_count_of_tile, stat_measurement_count_of_tile);
      if (!ok) {
        sqlite3_close(db_handle);
        return 2;
      }
    }
    LOG_INFO("Tile " + std::to_string(tileid) + " wrote " + std::to_string(stat_matched_count_of_tile) + "/" + std::to_string(stat_measurement_count_of_tile) + " points");
  }

  LOG_INFO("============= Summary ==================");
  LOG_INFO("Matched: " + std::to_string(stat_matched_count_totally) + "/" + std::to_string(stat_measurement_count_totally) + " points");

  sqlite3_close(db_handle);
  return 0;
}
