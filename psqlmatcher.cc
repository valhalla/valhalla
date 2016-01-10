// -*- mode: c++ -*-

#include <unordered_map>
#include <vector>
#include <cassert>
#include <iostream>
// For converting network byte order to host byte order
#include <arpa/inet.h>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// psql
#include <postgresql/libpq-fe.h>

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
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>

#include "costings.h"
#include "map_matching.h"

using namespace valhalla;
using namespace mm;


namespace {
constexpr int kSqliteMaxCompoundSelect = 10000;
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
    sequence.emplace_back(PointLL(coord.x, coord.y));
  }
  delete coords;
  delete geometry;

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
query_sequences(PGconn* conn, const BoundingBox& bbox)
{
  auto bbox_clause = "'BOX(" + joinbbox(bbox) + ")'::box2d";
  std::string statement = "SELECT id, ST_AsBinary(path_gm) AS geom FROM sequences WHERE "
                          + bbox_clause + " && path_gm AND NOT ST_IsEmpty(path_gm)"
                          + " LIMIT " + std::to_string(std::numeric_limits<int>::max());
  LOG_INFO("Fetching sequences in BBOX (" + joinbbox(bbox) + ")");

  // Send query
  auto result = PQexecParams(conn, statement.c_str(), 0, NULL, NULL, NULL, NULL, 1);
  if (PQresultStatus(result) != PGRES_TUPLES_OK) {
    PQclear(result);
    throw std::runtime_error(PQresultErrorMessage(result));
  }

  int nrows = PQntuples(result);
  std::unordered_map<SequenceId, Sequence> sequences;
  for (int row = 0; row < nrows; row++) {
    // Sequence Id
    auto size0 = PQgetlength(result, row, 0);
    assert(size0 == 4);
    int* val0 = (int*)PQgetvalue(result, row, 0);
    assert(sizeof(SequenceId) == size0 && sizeof(int) == size0);
    // Integers are stored in network byte order (big-endian), we need
    // to convert it to host byte order (little-endian)
    SequenceId sid = ntohl(*val0);

    // WKB geometry
    auto size1 = PQgetlength(result, row, 1);
    const char* val1 = PQgetvalue(result, row, 1);
    std::string bs(val1, size1);
    // It seems that binary data are stored in host byte order
    // already, no need to reverse it
    // std::reverse(bs.begin(), bs.end());

    sequences.emplace(sid, to_sequence(bs));
  }

  PQclear(result);

  return sequences;
}


// Collect all tile IDs
std::vector<uint32_t>
collect_local_tileids(const baldr::TileHierarchy& tile_hierarchy,
                      const std::unordered_set<uint32_t>& excluded_tileids)
{
  const auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;

  std::vector<uint32_t> queue;

  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId graphid(id, local_level, 0);
    if (baldr::GraphReader::DoesTileExist(tile_hierarchy, graphid)
        && excluded_tileids.find(id) == excluded_tileids.end()) {
      queue.push_back(graphid.tileid());
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
  auto graphid = tile_hierarchy.GetGraphId(sequence.front().lnglat(), local_level);
  if (graphid.Is_Valid()) {
    return graphid.tileid();
  }
  baldr::GraphId id;
  return id.tileid();
}


class SqliteWriteError: public std::runtime_error
{
  using std::runtime_error::runtime_error;
};


inline int
simple_sqlite3_exec(sqlite3* db_handle, const char* const statement)
{
  char *err_msg = NULL;
  int ret = sqlite3_exec(db_handle, statement, NULL, NULL, &err_msg);
  if (SQLITE_OK != ret) {
    std::string message(err_msg);
    sqlite3_free(err_msg);
    throw SqliteWriteError(message + " while executing " + statement);
  }
  // Passing a NULL pointer to sqlite3_free() is harmless
  sqlite3_free(err_msg);
  return ret;
}


void create_tiles_table(sqlite3* db_handle)
{
  std::string sql = "CREATE TABLE IF NOT EXISTS tiles"
                    " (id INTEGER PRIMARY KEY, matched_count INTEGER, total_count INTEGER)";
  simple_sqlite3_exec(db_handle, sql.c_str());
}


void create_scores_table(sqlite3* db_handle)
{
  std::string sql = "CREATE TABLE IF NOT EXISTS scores"
                    " (sequence_id INTEGER, coordinate_index INTEGER, graphid BIGINT, graphtype SMALLINT)";
  simple_sqlite3_exec(db_handle, sql.c_str());
}


void create_routes_table(sqlite3* db_handle)
{
  std::string sql = "CREATE TABLE IF NOT EXISTS routes"
                    " (sequence_id INTEGER, route_index INTEGER, edgeid BIGINT, source FLOAT, target FLOAT)";
  simple_sqlite3_exec(db_handle, sql.c_str());
}


bool read_accomplished_tiles(sqlite3* db_handle,
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
        tileids.insert(static_cast<SequenceId>(sequence_id));
      } else {
        LOG_ERROR("Found negative sequence ID which is not good");
      }
    }
    // Try again if busy
  } while (SQLITE_ROW == ret || SQLITE_BUSY == ret);

  if (SQLITE_DONE != ret) {
    LOG_ERROR("Expect SQLITE_DONE to return, but you got " + std::to_string(ret));
    sqlite3_finalize(stmt);
    return false;
  }

  sqlite3_finalize(stmt);
  return true;
}


// Start transaction
inline void write_begin(sqlite3* db_handle)
{
  std::string sql = "BEGIN";
  simple_sqlite3_exec(db_handle, sql.c_str());
}


// End transaction
void write_end(sqlite3* db_handle)
{
  std::string sql = "END";
  simple_sqlite3_exec(db_handle, sql.c_str());
}


using ResultTuple = std::tuple<SequenceId, uint32_t, baldr::GraphId, GraphType>;


size_t write_results_segment(sqlite3* db_handle,
                             const std::vector<ResultTuple>::const_iterator cbegin,
                             const std::vector<ResultTuple>::const_iterator cend)
{
  size_t written_count = 0;

  if (cbegin == cend) {
    return written_count;
  }

  std::string sql = "INSERT INTO scores VALUES ";
  for (auto result = cbegin; result != cend; result++, written_count++) {
    sql += "(";
    sql += std::to_string(std::get<0>(*result)) + ", "; // sequence_id
    sql += std::to_string(std::get<1>(*result)) + ", "; // coordinate_index
    sql += std::to_string(std::get<2>(*result)) + ", "; // graphid
    sql += std::to_string(static_cast<uint8_t>(std::get<3>(*result))); // graphtype
    sql += "),";
  }
  sql.pop_back();  // Pop out the last comma

  simple_sqlite3_exec(db_handle, sql.c_str());

  return written_count;
}


// Insert (sequence_id, coordinate_index, graphid, graphtype) into scores
size_t write_results(sqlite3* db_handle,
                     size_t write_size,
                     const std::unordered_map<SequenceId, std::vector<MatchResult>>& results)
{
  if (write_size <= 0) {
    throw std::invalid_argument("Expect segment size to be positive " + std::to_string(write_size));
  }

  std::vector<ResultTuple> tuples;
  size_t written_count = 0;

  // Convert match results into tuples
  for (const auto& pair : results) {
    uint32_t coord_idx = 0;
    for (const auto& result : pair.second) {
      if (result.graphid().Is_Valid()) {
        tuples.emplace_back(pair.first, coord_idx, result.graphid(), result.graphtype());
      }
      coord_idx++;
    }
  }

  for (decltype(tuples.size()) i = 0; i < tuples.size(); i += write_size) {
    const auto begin = std::next(tuples.cbegin(), i),
                 end = (i + write_size) < tuples.size()? std::next(begin, write_size) : tuples.cend();
    written_count += write_results_segment(db_handle, begin, end);
  }

  return written_count;
}


using EdgeSegmentTuple = std::tuple<SequenceId, uint32_t, baldr::GraphId, float, float>;


size_t write_routes_segment(sqlite3* db_handle,
                            std::vector<EdgeSegmentTuple>::const_iterator begin,
                            std::vector<EdgeSegmentTuple>::const_iterator end)
{
  size_t written_count = 0;

  if (begin == end) {
    return written_count;
  }

  std::string sql = "INSERT INTO routes VALUES ";
  for (auto tuple = begin; tuple != end; tuple++, written_count++) {
    sql += "(";
    sql += std::to_string(std::get<0>(*tuple)) + ", "; // sequence_id
    sql += std::to_string(std::get<1>(*tuple)) + ", "; // route_index
    sql += std::to_string(std::get<2>(*tuple)) + ", "; // edgeid
    sql += std::to_string(std::get<3>(*tuple)) + ", "; // source
    sql += std::to_string(std::get<4>(*tuple)); // target
    sql += "),";
  }
  sql.pop_back();  // Pop out the last comma

  simple_sqlite3_exec(db_handle, sql.c_str());

  return written_count;
}


// Insert (sequence_id, route_idx, edgeid, source, target) into routes
size_t write_routes(sqlite3* db_handle,
                    size_t write_size,
                    const std::unordered_map<SequenceId, std::vector<EdgeSegment>>& routes)
{
  if (write_size <= 0) {
    throw std::invalid_argument("Expect segment size to be positive " + std::to_string(write_size));
  }

  std::vector<EdgeSegmentTuple> tuples;
  size_t written_count = 0;

  // Convert routes into tuples
  for (const auto& pair : routes) {
    uint32_t route_idx = 0;
    for (const auto& segment : pair.second) {
      tuples.emplace_back(pair.first, route_idx, segment.edgeid, segment.source, segment.target);
      route_idx++;
    }
  }

  for (decltype(tuples.size()) i = 0; i < tuples.size(); i += write_size) {
    const auto begin = std::next(tuples.cbegin(), i),
                 end = (i + write_size) < tuples.size()? std::next(begin, write_size) : tuples.cend();
    written_count += write_routes_segment(db_handle, begin, end);
  }

  return written_count;
}


// Insert (tileid, matched_count, total_count) into tiles
void write_tiles_summary(sqlite3* db_handle,
                         uint32_t tileid,
                         uint32_t matched_count,
                         uint32_t total_count)
{
  std::string sql = "INSERT INTO tiles VALUES (";
  sql += std::to_string(tileid) + ", ";
  sql += std::to_string(matched_count) + ", ";
  sql += std::to_string(total_count) + ");\n";
  simple_sqlite3_exec(db_handle, sql.c_str());
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
  boost::property_tree::ptree config;
  boost::property_tree::read_json(config_file_path, config);

  mm::MapMatcherFactory matcher_factory_(config);
  auto matcher = matcher_factory_.Create(config.get<std::string>("mm.mode"));

  ////////////////////////
  // Prepare sqlite3 database for writing results
  sqlite3* db_handle;
  int ret = sqlite3_open_v2(sqlite3_file_path.c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr);
  if (SQLITE_OK != ret) {
    LOG_ERROR("Failed to open sqlite3 database at " + sqlite3_file_path);
    return 2;
  }

  int write_size = kSqliteMaxCompoundSelect;

#if SQLITE_VERSION_NUMBER < 3008008
  // SQLite under 3.8.8 has limitation on the number of rows in a VALUES clause
  // See https://www.sqlite.org/changes.html
  sqlite3_limit(db_handle, SQLITE_LIMIT_COMPOUND_SELECT, kSqliteMaxCompoundSelect);
  // Read the true value back
  write_size = sqlite3_limit(db_handle, SQLITE_LIMIT_COMPOUND_SELECT, -1);
#endif

  LOG_INFO("Read config write_size = " + std::to_string(write_size));
  if (write_size <= 0) {
    LOG_ERROR("Expect SQLITE_LIMIT_COMPOUND_SELECT to be positive " + std::to_string(write_size));
    return 3;
  }

  create_scores_table(db_handle);
  create_routes_table(db_handle);
  create_tiles_table(db_handle);

  ////////////////////////
  // Collect tiles
  std::unordered_set<uint32_t> accomplished_tileids;
  {
    bool ok = read_accomplished_tiles(db_handle, accomplished_tileids);
    if (!ok) {
      sqlite3_close(db_handle);
      return 2;
    }
  }
  const auto& tile_hierarchy = matcher_factory_.graphreader().GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  auto tileids = collect_local_tileids(tile_hierarchy, accomplished_tileids);
  auto total_tiles = tileids.size() + accomplished_tileids.size();
  if (accomplished_tileids.size() > 0) {
    LOG_INFO("Found " + std::to_string(accomplished_tileids.size()) + " tiles accomplished in " + sqlite3_file_path);
  }
  LOG_INFO("Need to match " + std::to_string(tileids.size()) + " tiles");

  //////////////////////
  // For each tile:
  //     1. Query all sequences within bounding box of this tile
  //     2. For each sequence:
  //            Offline match the sequence
  //     3. Insert rows [sequence's id, coordinate's index, edge's GraphId] into sqlite3

  uint64_t total_matched_results_count = 0;
  uint64_t total_measurement_count = 0;

  auto conn = PQconnectdb(psql_uri.c_str());

  for (auto tileid : tileids) {
    std::unordered_map<SequenceId, std::vector<MatchResult>> results;
    std::unordered_map<SequenceId, std::vector<EdgeSegment>> routes;

    uint32_t measurement_count = 0;
    const auto& bbox = tiles.TileBounds(tileid);
    for (const auto& sequencepair : query_sequences(conn, bbox)) {
      auto sid = sequencepair.first;
      const auto& sequence = sequencepair.second;

      if (sequence.empty()) {
        continue;
      }

      // Skip sequences that don't belong to this tile (it must belong
      // to another tile)
      if (which_tileid(tile_hierarchy, sequence) != tileid) {
        continue;
      }

      const auto& match_results = matcher->OfflineMatch(sequence);
      assert(match_results.size() == sequence.size());

      results.emplace(sid, match_results);
      routes.emplace(sid, mm::ConstructRoute(match_results.begin(), match_results.end()));

      measurement_count += sequence.size();
    }

    // Write into sqlite
    write_begin(db_handle);
    auto matched_results_count = write_results(db_handle, write_size, results);
    write_routes(db_handle, write_size, routes);
    write_tiles_summary(db_handle, tileid, matched_results_count, measurement_count);
    write_end(db_handle);

    accomplished_tileids.insert(tileid).second;

    total_matched_results_count += matched_results_count;
    total_measurement_count += measurement_count;

    LOG_INFO("Matched " + std::to_string(matched_results_count) + "/" + std::to_string(measurement_count) + " points in Tile " + std::to_string(tileid));
    LOG_INFO("So far matched " + std::to_string(total_matched_results_count) + "/" + std::to_string(total_measurement_count)
             + " points and processed "
             + std::to_string(accomplished_tileids.size()) + "/" + std::to_string(total_tiles)
             + " tiles");

    matcher_factory_.ClearCacheIfPossible();
  }

  matcher_factory_.ClearCache();
  PQfinish(conn);
  sqlite3_close(db_handle);

  return 0;
}
