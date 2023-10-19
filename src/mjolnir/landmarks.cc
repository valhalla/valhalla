#include "mjolnir/landmarks.h"
#include "filesystem.h"

#include "baldr/graphreader.h"
#include "midgard/sequence.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"

#include "baldr/location.h"
#include "baldr/pathlocation.h"
#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "mjolnir/graphtilebuilder.h"
#include "sif/nocost.h"

#include <future>
#include <thread>
#include <tuple>

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;
using namespace valhalla::midgard;
using namespace valhalla;

namespace {
// a 25m radius used to associate edges to landmarks, which allows us to only keep the close edges in
// the tight cities
constexpr unsigned long kLandmarkRadius = 25;
// a 75m search cutoff used to associate edges to landmarks, which should allow us to get gas stations
// that are off the road a bit (for parking)
constexpr float kLandmarkSearchCutoff = 75.;
// a slight buffer to add to landmark queries to avoid near misses in the data due to precision
constexpr double kLandmarkQueryBuffer = .000001;

struct landmark_callback : public OSMPBF::Callback {
public:
  landmark_callback(const std::string& db_name) : db_(db_name, false) {
  }
  virtual ~landmark_callback() {
  }

  virtual void
  node_callback(const uint64_t /*osmid*/, double lng, double lat, const OSMPBF::Tags& tags) override {
    auto iter = tags.find("amenity");
    if (iter != tags.cend() && !iter->second.empty()) {
      try {
        auto landmark_type = string_to_landmark_type(iter->second);

        std::string name = "";
        auto it = tags.find("name");
        if (it != tags.cend() && !it->second.empty()) {
          name = it->second;
        }

        // insert parsed landmark directly into database
        db_.insert_landmark(name, landmark_type, lng, lat);
      } catch (...) {}
    }
  }

  virtual void changeset_callback(const uint64_t /*changeset_id*/) override {
    LOG_WARN("landmark changeset callback shouldn't be called!");
  }

  virtual void way_callback(const uint64_t /*osmid*/,
                            const OSMPBF::Tags& /*tags*/,
                            const std::vector<uint64_t>& /*nodes*/) override {
    LOG_WARN("landmark way callback shouldn't be called!");
  }

  virtual void relation_callback(const uint64_t /*osmid*/,
                                 const OSMPBF::Tags& /*tags*/,
                                 const std::vector<OSMPBF::Member>& /*members*/) override {
    LOG_WARN("landmark relation callback shouldn't be called!");
  }

  valhalla::mjolnir::LandmarkDatabase db_;
};

// sort a sequence file to put the edges in the same tile together
bool sort_seq_file(const std::pair<GraphId, uint64_t>& a, const std::pair<GraphId, uint64_t>& b) {
  if (a.first.Tile_Base() == b.first.Tile_Base()) {
    return a.first.id() < b.first.id();
  }
  return a.first.Tile_Base() < b.first.Tile_Base();
}
} // namespace

namespace valhalla {
namespace mjolnir {
// TODO: this can be a utility and be more generic with a few more options, we could make the prepared
//  statements on the fly and retrievable by the caller, then anything in the code base that wants to
//  use sqlite can make use of this utility class. for now its ok to be specific to landmarks though
struct LandmarkDatabase::db_pimpl {
  sqlite3* db;
  sqlite3_stmt* insert_stmt;
  sqlite3_stmt* bounding_box_stmt;
  std::shared_ptr<void> spatial_lite;
  bool vacuum_analyze = false;

  db_pimpl(const std::string& db_name, bool read_only)
      : insert_stmt(nullptr), bounding_box_stmt(nullptr) {
    // create parent directory if it doesn't exist
    const filesystem::path parent_dir = filesystem::path(db_name).parent_path();
    if (!filesystem::exists(parent_dir) && !filesystem::create_directories(parent_dir)) {
      throw std::runtime_error("Can't create parent directory " + parent_dir.string());
    }

    // figure out if we need to create database or can just open it up
    auto flags = read_only ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
    if (!filesystem::exists(db_name)) {
      if (read_only)
        throw std::logic_error("Cannot open sqlite database in read-only mode if it does not exist");
    } else if (!read_only) {
      filesystem::remove(db_name);
      LOG_INFO("deleting existing landmark database " + db_name + ", creating a new one");
    }

    // get a connection to the database
    auto ret = sqlite3_open_v2(db_name.c_str(), &db, flags, NULL);
    if (ret != SQLITE_OK) {
      throw std::runtime_error("Failed to open sqlite database: " + db_name);
    }

    // loading spatiaLite as an extension
    spatial_lite = make_spatialite_cache(db);

    // if the db was empty we need to initialize the schema
    char* err_msg = nullptr;
    if (flags & SQLITE_OPEN_CREATE) {
      // make the table
      const char* table =
          "SELECT InitSpatialMetaData(1); CREATE TABLE IF NOT EXISTS landmarks (id INTEGER PRIMARY KEY, name TEXT, type TEXT)";
      ret = sqlite3_exec(db, table, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite table creation error: " + std::string(err_msg));
      }

      // add geom column
      const char* geom = "SELECT AddGeometryColumn('landmarks', 'geom', 4326, 'POINT', 2)";
      ret = sqlite3_exec(db, geom, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite geom column creation error: " + std::string(err_msg));
      }

      // make the index
      const char* index = "SELECT CreateSpatialIndex('landmarks', 'geom')";
      ret = sqlite3_exec(db, index, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite spatial index creation error: " + std::string(err_msg));
      }

      // prep the insert statement
      const char* insert =
          "INSERT INTO landmarks (name, type, geom) VALUES (?, ?, MakePoint(?, ?, 4326))";
      ret = sqlite3_prepare_v2(db, insert, strlen(insert), &insert_stmt, NULL);
      if (ret != SQLITE_OK)
        throw std::runtime_error("Sqlite prepared insert statement error: " +
                                 std::string(sqlite3_errmsg(db)));
    }

    // prep the select statement
    const char* select =
        "SELECT id, name, type, X(geom), Y(geom) FROM landmarks WHERE ST_Covers(BuildMbr(?, ?, ?, ?, 4326), geom)";
    ret = sqlite3_prepare_v2(db, select, strlen(select), &bounding_box_stmt, NULL);
    if (ret != SQLITE_OK) {
      throw std::runtime_error("Sqlite prepared select statement error: " +
                               std::string(sqlite3_errmsg(db)));
    }
  }
  ~db_pimpl() {
    char* err_msg = nullptr;
    if (vacuum_analyze && sqlite3_exec(db, "VACUUM", NULL, NULL, &err_msg) != SQLITE_OK) {
      sqlite3_free(err_msg);
      LOG_ERROR("Sqlite vacuum error: " + std::string(err_msg));
    }

    if (vacuum_analyze && sqlite3_exec(db, "ANALYZE", NULL, NULL, &err_msg) != SQLITE_OK) {
      sqlite3_free(err_msg);
      LOG_ERROR("Sqlite analyze error: " + std::string(err_msg));
    }

    sqlite3_finalize(insert_stmt);
    sqlite3_finalize(bounding_box_stmt);
    sqlite3_close_v2(db);
  }
  std::string last_error() {
    return std::string(sqlite3_errmsg(db));
  }
};

LandmarkDatabase::LandmarkDatabase(const std::string& db_name, bool read_only)
    : pimpl(new db_pimpl(db_name, read_only)) {
}

void LandmarkDatabase::insert_landmark(const std::string& name,
                                       const LandmarkType& type,
                                       const double lng,
                                       const double lat) {
  auto* insert_stmt = pimpl->insert_stmt;
  if (!insert_stmt)
    throw std::logic_error("Sqlite database connection is read-only");

  sqlite3_reset(insert_stmt);
  sqlite3_clear_bindings(insert_stmt);

  sqlite3_bind_text(insert_stmt, 1, name.c_str(), name.length(), SQLITE_STATIC);
  sqlite3_bind_int(insert_stmt, 2, static_cast<int>(type));
  sqlite3_bind_double(insert_stmt, 3, lng);
  sqlite3_bind_double(insert_stmt, 4, lat);

  LOG_TRACE(sqlite3_expanded_sql(insert_stmt));
  if (sqlite3_step(insert_stmt) != SQLITE_DONE)
    throw std::runtime_error("Sqlite could not insert landmark: " + pimpl->last_error());
  pimpl->vacuum_analyze = true;
}

// get multiple landmarks by their ids
/** TODO: Currently this function dynamically creates query statement based on
 *  the number of provided primary keys.
 *  In the future, we may consider implementing a fix-sized batch retrieval approach, where
 *  multiple landmarks are retrieved in batches using a prepared SQL statement with a fixed
 *  number of placeholders (e.g., 10 question marks) to be filled with corresponding inputs.
 *  If the caller provides more than the fixed number of inputs, the function will automatically
 *  perform multiple batch retrieves.
 */
std::vector<Landmark> LandmarkDatabase::get_landmarks_by_ids(const std::vector<int64_t>& pkeys) {
  // create the sql statement with inputs
  std::string sql = "SELECT id, name, type, X(geom), Y(geom) FROM landmarks WHERE id IN (";
  for (size_t i = 0; i < pkeys.size(); ++i) {
    if (i > 0) {
      sql += ", ";
    }
    sql += std::to_string(static_cast<int>(pkeys[i]));
  }
  sql += ")";

  // callback for the sql query
  auto populate_landmarks = [](void* data, int argc, char** argv, char** col_names) {
    std::vector<Landmark>* landmarks = static_cast<std::vector<Landmark>*>(data);

    int64_t landmark_id = static_cast<int64_t>(std::stoi(argv[0]));
    const char* landmark_name = argv[1];
    int landmark_type = std::stoi(argv[2]);
    double lng = std::stod(argv[3]);
    double lat = std::stod(argv[4]);

    landmarks->emplace_back(
        Landmark(landmark_id, landmark_name, static_cast<LandmarkType>(landmark_type), lng, lat));
    return 0;
  };

  std::vector<Landmark> landmarks;
  char* err_msg = nullptr;
  // execute query
  int ret = sqlite3_exec(pimpl->db, sql.c_str(), populate_landmarks, &landmarks, &err_msg);

  // check for errors in the sql execution
  if (ret != SQLITE_OK) {
    throw std::runtime_error("Sqlite execution error: " + std::string(err_msg));
  }

  return landmarks;
}

std::vector<Landmark> LandmarkDatabase::get_landmarks_by_bbox(const double minlng,
                                                              const double minlat,
                                                              const double maxlng,
                                                              const double maxlat) {
  std::vector<Landmark> landmarks;

  auto* bounding_box_stmt = pimpl->bounding_box_stmt;
  sqlite3_reset(bounding_box_stmt);
  sqlite3_clear_bindings(bounding_box_stmt);

  sqlite3_bind_double(bounding_box_stmt, 1, minlng);
  sqlite3_bind_double(bounding_box_stmt, 2, minlat);
  sqlite3_bind_double(bounding_box_stmt, 3, maxlng);
  sqlite3_bind_double(bounding_box_stmt, 4, maxlat);

  LOG_TRACE(sqlite3_expanded_sql(bounding_box_stmt));

  int ret = sqlite3_step(bounding_box_stmt);
  while (ret == SQLITE_ROW) {
    auto landmark_id = static_cast<int64_t>(sqlite3_column_int64(bounding_box_stmt, 0));
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 1));
    int landmark_type = sqlite3_column_int(bounding_box_stmt, 2);
    double lng = sqlite3_column_double(bounding_box_stmt, 3);
    double lat = sqlite3_column_double(bounding_box_stmt, 4);

    landmarks.emplace_back(
        Landmark(landmark_id, name, static_cast<LandmarkType>(landmark_type), lng, lat));

    ret = sqlite3_step(bounding_box_stmt);
  }

  if (ret != SQLITE_DONE && ret != SQLITE_OK) {
    throw std::runtime_error("Sqlite could not query landmarks in bounding box: " +
                             pimpl->last_error());
  }

  return landmarks;
}

bool BuildLandmarkFromPBF(const boost::property_tree::ptree& pt,
                          const std::vector<std::string>& input_files) {
  // parse pbf to get landmark nodes
  const std::string db_name = pt.get<std::string>("landmarks", "");
  landmark_callback callback(db_name);

  LOG_INFO("Parsing files...");
  // hold open all the files so that if something else (like diff application)
  // needs to mess with them we wont have troubles with inodes changing underneath us
  std::list<std::ifstream> file_handles;
  for (const auto& input_file : input_files) {
    file_handles.emplace_back(input_file, std::ios::binary);
    if (!file_handles.back().is_open()) {
      throw std::runtime_error("Unable to open: " + input_file);
    }
  }

  LOG_INFO("Parsing nodes and storing landmarks...");
  for (auto& file_handle : file_handles) {
    OSMPBF::Parser::parse(file_handle, static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES),
                          callback);
  }

  LOG_INFO("Successfully built landmark database from PBF");
  return true;
}

// Find landmarks in the tiles and the edges correlated to each landmark,
// and return the sequence file name where we wrote the correlations
void FindLandmarkEdges(const boost::property_tree::ptree& pt,
                       const std::vector<GraphId>& tileset,
                       const size_t& thread_number,
                       const size_t& total_threads,
                       std::promise<std::string>& seq_file_name) {
  // Open the database and create a graph reader
  const std::string db_name = pt.get<std::string>("landmarks", "");

  LandmarkDatabase db(db_name, true);
  GraphReader reader(pt);
  // create the sequence file
  std::string file_name = "landmark_dump_" + std::to_string(thread_number);
  midgard::sequence<std::pair<GraphId, uint64_t>> seq_file(file_name, true);

  for (size_t i = 0; i < tileset.size(); ++i) {
    // every i'th thread works on every i'th tile
    if (i % total_threads == thread_number) {
      // get landmarks in the tile
      midgard::AABB2<PointLL> bbox = baldr::TileHierarchy::GetGraphIdBoundingBox(tileset[i]);

      std::vector<Landmark> landmarks = db.get_landmarks_by_bbox(bbox.minx() - kLandmarkQueryBuffer,
                                                                 bbox.miny() - kLandmarkQueryBuffer,
                                                                 bbox.maxx() + kLandmarkQueryBuffer,
                                                                 bbox.maxy() + kLandmarkQueryBuffer);

      // find and collect all nearby path locations for the landmarks
      for (const auto& landmark : landmarks) {
        baldr::Location landmark_location(midgard::PointLL{landmark.lng, landmark.lat},
                                          baldr::Location::StopType::BREAK, 0, 0, kLandmarkRadius);
        landmark_location.search_cutoff_ = kLandmarkSearchCutoff;

        // call loki::Search to get nearby edges to each landmark
        std::unordered_map<valhalla::baldr::Location, PathLocation> result =
            loki::Search({landmark_location}, reader, sif::CreateNoCost({}));

        // we only have one landmark as input so the return size should be no more than one
        if (result.size() > 1) {
          throw std::logic_error(
              "Error occurred in finding nearby edges to a landmark. Result size is " +
              std::to_string(result.size()) + ", but should be one or zero");
        }
        // if the landmark should not be associated with any edge
        if (result.size() == 0) {
          continue;
        }

        std::vector<PathLocation::PathEdge> edges = result.begin()->second.edges;
        // for each edge insert edgeid - landmark_pkey pair into the sequence file
        // TODO: maybe do some filtering and only keep some of the edges it finds? (now we have the
        //  75m search cutoff)
        for (const auto& edge : edges) {
          seq_file.push_back(std::make_pair(edge.id, landmark.id));
        }
      }
    }
  }

  seq_file_name.set_value(file_name);
}

// Update tiles to associate landmarks with edges, return some stats about the numbers of updated
// tiles, edges, and landmarks. NOTE: the input sequence file seq_file is passed by reference, but
// should not be modified by these threads.
void UpdateTiles(midgard::sequence<std::pair<GraphId, uint64_t>>& seq_file,
                 const std::string& tile_dir,
                 const std::string& db_name,
                 const size_t& thread_number,
                 const size_t& total_threads,
                 std::promise<std::tuple<size_t, size_t, size_t>>& stats) {
  // open the database and initialize a unique pointer to graph tile builder
  std::unique_ptr<GraphTileBuilder> tile_builder_ptr = nullptr;

  LandmarkDatabase db(db_name, true);

  // stats to record how many tiles, edges and landmarks are updated
  size_t updated_tiles = 0, updated_edges = 0, updated_landmarks = 0;
  GraphId last_edge, last_tile;

  size_t tile_count = static_cast<size_t>(-1);
  // every i'th thread works on every i'th tile
  for (auto it = seq_file.begin(); it != seq_file.end(); ++it) {
    // if the current tile is not the same as the last one, increase counter by one
    if ((*it).first.Tile_Base() != last_tile) {
      last_tile = (*it).first.Tile_Base();
      tile_count++;
    }
    // decide whether this tile is a "every i'th tile". if not, the thread should skip it
    if (tile_count % total_threads != thread_number) {
      continue;
    }

    // now this pair is on a "every i'th tile". the thread should process it.

    // if this pair is on a new tile, then store the previous tile and move to the new tile
    if (!tile_builder_ptr ||
        tile_builder_ptr->header_builder().graphid().Tile_Base() != (*it).first.Tile_Base()) {
      // store the previously updated tile
      if (tile_builder_ptr) {
        tile_builder_ptr->StoreTileData();
        updated_tiles++;
      }
      // reset the tile builder to this new tile
      tile_builder_ptr.reset(new GraphTileBuilder(tile_dir, (*it).first.Tile_Base(), true));
    }

    // retrieve the landmark to be added
    // TODO: in the future we can do batches of ids, though it will complicate the code it will likely
    // speed up the processing
    const std::vector<Landmark> landmark =
        db.get_landmarks_by_ids({static_cast<int64_t>((*it).second)});
    if (landmark.size() != 1) {
      throw std::logic_error("Incorrect result size " + std::to_string(landmark.size()) +
                             " of retrieved landmarks, which should be 1");
    }
    // add the landmark to the tile
    GraphId edge_id = (*it).first;
    tile_builder_ptr->AddLandmark(edge_id, landmark[0]);

    // update the stats
    updated_landmarks++;
    // a single edge can have multiple landmarks
    // record the number of unique edges updated (pairs with the same edge should appear consecutively
    // in the sequence)
    if (last_edge != (*it).first) {
      updated_edges++;
      last_edge = (*it).first;
    }
  }
  // store the last updated tile
  if (tile_builder_ptr) {
    tile_builder_ptr->StoreTileData();
    updated_tiles++;
  }

  // set the stats
  stats.set_value(std::make_tuple(updated_tiles, updated_edges, updated_landmarks));
}

// Add all landmarks to tiles
bool AddLandmarks(const boost::property_tree::ptree& pt) {
  LOG_INFO("Starting adding landmarks to tiles...");

  const size_t num_threads =
      pt.get<size_t>("mjolnir.concurrency", std::thread::hardware_concurrency());
  const std::string db_name = pt.get_child("mjolnir").get<std::string>("landmarks_db", "");

  // get tile access
  baldr::GraphReader reader(pt.get_child("mjolnir"));

  // get all tile ids and sort the tiles in descending order by size to balance the threads
  // TODO: it is possible in a global tileset that we have coverage only at level 2 for some places
  // and we'd still like to get landmarks there. we'll probably need to fix this.
  auto tileset = reader.GetTileSet(1);
  std::vector<GraphId> vec_tileset(tileset.begin(),
                                   tileset.end()); // turn the unordered_set into a vector for sorting

  std::sort(vec_tileset.begin(), vec_tileset.end(), [&](const auto id_a, const auto id_b) {
    return reader.GetGraphTile(id_a)->header()->nodecount() >
           reader.GetGraphTile(id_b)->header()->nodecount();
  });

  LOG_INFO("Finding landmarks and their correlated edges...");

  std::vector<std::shared_ptr<std::thread>> threads(num_threads);
  std::vector<std::promise<std::string>> sequence_file_names(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    threads[i].reset(new std::thread(FindLandmarkEdges, std::cref(pt.get_child("mjolnir")),
                                     std::cref(vec_tileset), i, num_threads,
                                     std::ref(sequence_file_names[i])));
  }

  // join all the threads and collect the sequence file names
  for (auto& thread : threads) {
    thread->join();
  }

  std::vector<std::string> seq_names{};
  seq_names.reserve(sequence_file_names.size());
  for (std::promise<std::string>& s : sequence_file_names) {
    seq_names.push_back(s.get_future().get());
  }

  LOG_INFO("Sorting landmark edge pairs by tile...");

  // concatenate all sequence files and sort the merged sequence file
  std::string merged_seq_file = seq_names.back();
  seq_names.pop_back();

  std::ofstream seq_file(merged_seq_file, std::ios_base::binary | std::ios_base::app);
  for (std::string& s : seq_names) {
    std::ifstream seq(s, std::ios_base::binary);
    seq_file << seq.rdbuf();
  }
  seq_file.close();

  midgard::sequence<std::pair<GraphId, uint64_t>> merged_sequence_file(merged_seq_file, false);
  merged_sequence_file.sort(sort_seq_file);

  LOG_INFO("Updating tiles...");

  // re-open the thread pool to update tiles
  std::vector<std::promise<std::tuple<size_t, size_t, size_t>>> stats_info(
      num_threads); // tiles, edges, landmarks

  const std::string tile_dir = reader.tile_dir();
  for (size_t i = 0; i < num_threads; ++i) {
    // assume the data size that each thread processes doesn't affect performance a lot
    threads[i].reset(new std::thread(UpdateTiles, std::ref(merged_sequence_file), tile_dir, db_name,
                                     i, num_threads, std::ref(stats_info[i])));
  }

  for (auto& thread : threads) {
    thread->join();
  }

  // collect and log the stats
  size_t tiles = 0, edges = 0, landmarks = 0;
  for (std::promise<std::tuple<size_t, size_t, size_t>>& s : stats_info) {
    std::tuple<size_t, size_t, size_t> data = s.get_future().get();
    tiles += std::get<0>(data);
    edges += std::get<1>(data);
    landmarks += std::get<2>(data);
  }

  LOG_INFO("Updated " + std::to_string(tiles) + " unique tiles, " + std::to_string(edges) +
           " unique directed edges, and wrote " + std::to_string(landmarks) +
           " landmarks (including repeated ones)");

  return true;
}

} // end namespace mjolnir
} // end namespace valhalla
