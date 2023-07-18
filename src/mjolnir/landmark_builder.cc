#include "mjolnir/landmark_builder.h"
#include "filesystem.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"
#include <boost/property_tree/ptree.hpp>

namespace {
struct landmark_callback : public OSMPBF::Callback {
public:
  landmark_callback(const std::string& db_name) : db_(db_name, false) {
  }
  virtual ~landmark_callback() {
  }

  virtual void
  node_callback(const uint64_t /*osmid*/, double lng, double lat, const OSMPBF::Tags& tags) override {
    valhalla::mjolnir::Landmark landmark;

    auto iter = tags.find("amenity");
    if (iter != tags.cend() && !iter->second.empty()) {
      // store landmark nodes that belong to LandmarkType only
      landmark.type = valhalla::mjolnir::string_to_landmark_type(iter->second);
      if (landmark.type == valhalla::mjolnir::LandmarkType::null) {
        return;
      }

      auto it = tags.find("name");
      if (it != tags.cend() && !it->second.empty()) {
        landmark.name = it->second;
      }

      landmark.lng = lng;
      landmark.lat = lat;

      // insert parsed landmark directly into database
      db_.insert_landmark(landmark);
    }
  }

  virtual void changeset_callback(const uint64_t changeset_id) override {
  }

  virtual void way_callback(const uint64_t /*osmid*/,
                            const OSMPBF::Tags& /*tags*/,
                            const std::vector<uint64_t>& /*nodes*/) override {
    LOG_WARN("way callback shouldn't be called!");
  }

  virtual void relation_callback(const uint64_t /*osmid*/,
                                 const OSMPBF::Tags& /*tags*/,
                                 const std::vector<OSMPBF::Member>& /*members*/) override {
    LOG_WARN("relation callback shouldn't be called!");
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

  db_pimpl(const std::string& db_name, bool read_only) : insert_stmt(nullptr) {
    // figure out if we need to create it or can just open it up
    auto flags = read_only ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE;
    if (!filesystem::exists(db_name)) {
      LOG_INFO("database doesn't exist: " + db_name + ", creating now");
      if (read_only)
        throw std::logic_error("Cannot open sqlite database in read-only mode if it does not exist");
      flags |= SQLITE_OPEN_CREATE;
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
          "SELECT InitSpatialMetaData(1); CREATE TABLE IF NOT EXISTS landmarks (name TEXT, type TEXT)";
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
    }

    if (flags & SQLITE_OPEN_READWRITE) {
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
        "SELECT name, type, X(geom), Y(geom) FROM landmarks WHERE ST_Covers(BuildMbr(?, ?, ?, ?, 4326), geom)";
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

void LandmarkDatabase::insert_landmark(const Landmark& landmark) {
  auto* insert_stmt = pimpl->insert_stmt;
  if (!insert_stmt)
    throw std::logic_error("Sqlite database connection is read-only");

  sqlite3_reset(insert_stmt);
  sqlite3_clear_bindings(insert_stmt);

  if (landmark.name != "") {
    sqlite3_bind_text(insert_stmt, 1, landmark.name.c_str(), landmark.name.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(insert_stmt, 1);
  }

  if (landmark.type != LandmarkType::null) {
    sqlite3_bind_int(insert_stmt, 2, static_cast<int>(landmark.type));
  } else {
    sqlite3_bind_null(insert_stmt, 2);
  }

  sqlite3_bind_double(insert_stmt, 3, landmark.lng);
  sqlite3_bind_double(insert_stmt, 4, landmark.lat);

  LOG_TRACE(sqlite3_expanded_sql(insert_stmt));
  if (sqlite3_step(insert_stmt) != SQLITE_DONE)
    throw std::runtime_error("Sqlite could not insert landmark: " + pimpl->last_error());
  pimpl->vacuum_analyze = true;
}

std::vector<Landmark> LandmarkDatabase::get_landmarks_in_bounding_box(const double minLat,
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
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 0));

    LandmarkType type = LandmarkType::null;
    if (sqlite3_column_type(bounding_box_stmt, 1) != SQLITE_NULL) {
      type = static_cast<LandmarkType>(sqlite3_column_int(bounding_box_stmt, 1));
    }

    double lng = sqlite3_column_double(bounding_box_stmt, 2);
    double lat = sqlite3_column_double(bounding_box_stmt, 3);

    landmarks.emplace_back(Landmark{name ? name : "", type, lng, lat});

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
  // parse config to get landmark database
  auto db = pt.get_optional<std::string>("landmarks");
  if (!db) {
    LOG_ERROR("Landmarks config info not found. Landmarks builder will not be created.");
    return false;
  }

  const filesystem::path parent_dir = filesystem::path(*db).parent_path();
  if (!filesystem::exists(parent_dir) && !filesystem::create_directories(parent_dir)) {
    LOG_ERROR("Can't create parent directory " + parent_dir.string());
    return false;
  }

  // parse pbf to get landmark nodes
  const std::string db_name = pt.get<std::string>("landmarks");
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
