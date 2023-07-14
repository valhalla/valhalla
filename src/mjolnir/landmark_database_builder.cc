#include "mjolnir/landmark_database_builder.h"
#include "filesystem.h"
#include "mjolnir/util.h"

#include "mjolnir/osmpbfparser.h"

namespace valhalla {
namespace mjolnir {
<<<<<<< HEAD
=======
const std::string landmark_db = "landmarks.db";

static const std::unordered_map<std::string, LandmarkType> str_type_map = {
    {"parking", LandmarkType::parking},
    {"bench", LandmarkType::bench},
    {"parking_space", LandmarkType::parking_space},
    {"place_of_worship", LandmarkType::place_of_worship},
    {"restaurant", LandmarkType::restaurant},
    {"waste_basket", LandmarkType::waste_basket},
    {"bicycle_parking", LandmarkType::bicycle_parking},
    {"fast_food", LandmarkType::fast_food},
    {"cafe", LandmarkType::cafe},
    {"fuel", LandmarkType::fuel},
    {"shelter", LandmarkType::shelter},
    {"recycling", LandmarkType::recycling},
    {"toilets", LandmarkType::toilets},
    {"bank", LandmarkType::bank},
    {"pharmacy", LandmarkType::pharmacy},
};

LandmarkType string_to_landmark_type(const std::string& landmark_type_str) {
  auto it = str_type_map.find(landmark_type_str);
  if (it != str_type_map.end()) {
    return it->second;
  }
  // default value if landmark type is not found or empty
  return LandmarkType::NA;
}

void LandmarkDatabase::connect_database() {
  if (!open_database()) {
    throw std::runtime_error("Cannot open database");
    return;
  }
>>>>>>> 5bb48a1b9 (add BuildLandmarkFromPBF function: parse landmarks from pbf and store them in database)

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

  if (landmark.type != LandmarkType::NA) {
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

    LandmarkType type = LandmarkType::NA;
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

// Task 2: parse landmarks
// anonymous namespace?
struct landmark_callback : public OSMPBF::Callback {
public:
  landmark_callback(std::vector<Landmark>& landmarks) : landmarks_(landmarks) {
  }
  virtual ~landmark_callback() {
  }

  virtual void
  node_callback(const uint64_t /*osmid*/, double lng, double lat, const OSMPBF::Tags& tags) override {
    Landmark landmark;

    for (const auto& tag : tags) {
      if (tag.first == "amenity") {
        // if amenity is empty will return LandmarkType::NA
        landmark.type = string_to_landmark_type(tag.second);
      }
      if (tag.first == "name" && !tag.second.empty()) {
        landmark.name = tag.second;
      }
      landmark.lng = lng;
      landmark.lat = lat;
    }

    landmarks_.push_back(std::move(landmark));
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

  std::vector<Landmark>& landmarks_;
};

bool BuildLandmarkFromPBF(const std::vector<std::string>& input_files) {
  // parse nodes in pbf to get landmarks
  std::vector<Landmark> landmarks{};

  landmark_callback callback(landmarks);

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

  LOG_INFO("Parsing nodes...");
  for (auto& file_handle : file_handles) {
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }

  // store landmarks in database
  LandmarkDatabase db(landmark_db, false);

  for (const auto& landmark : landmarks) {
    if (!db.insert_landmark(landmark)) {
      LOG_ERROR("cannot insert landmark");
      return false;
    }
  }

  return true;
}

} // end namespace mjolnir
} // end namespace valhalla
