#include "mjolnir/landmark_builder.h"
#include "filesystem.h"

#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"
#include <tuple>

namespace {
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
        auto landmark_type = valhalla::mjolnir::string_to_landmark_type(iter->second);

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

    landmarks->emplace_back(landmark_id, landmark_name, static_cast<LandmarkType>(landmark_type), lng,
                            lat);
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

std::vector<Landmark> LandmarkDatabase::get_landmarks_by_bbox(const double minLat,
                                                              const double minLong,
                                                              const double maxLat,
                                                              const double maxLong) {
  std::vector<Landmark> landmarks;

  auto* bounding_box_stmt = pimpl->bounding_box_stmt;
  sqlite3_reset(bounding_box_stmt);
  sqlite3_clear_bindings(bounding_box_stmt);

  sqlite3_bind_double(bounding_box_stmt, 1, minLong);
  sqlite3_bind_double(bounding_box_stmt, 2, minLat);
  sqlite3_bind_double(bounding_box_stmt, 3, maxLong);
  sqlite3_bind_double(bounding_box_stmt, 4, maxLat);

  LOG_TRACE(sqlite3_expanded_sql(bounding_box_stmt));

  int ret = sqlite3_step(bounding_box_stmt);
  while (ret == SQLITE_ROW) {
    auto landmark_id = static_cast<int64_t>(sqlite3_column_int64(bounding_box_stmt, 0));
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 1));
    int landmark_type = sqlite3_column_int(bounding_box_stmt, 2);
    double lng = sqlite3_column_double(bounding_box_stmt, 3);
    double lat = sqlite3_column_double(bounding_box_stmt, 4);

    landmarks.emplace_back(landmark_id, name, static_cast<LandmarkType>(landmark_type), lng, lat);

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

  return true;
}

} // end namespace mjolnir
} // end namespace valhalla
