#include "mjolnir/sqlite3.h"
#include "midgard/logging.h"

#include <sqlite3.h>
// needs to be after sqlite include
#include <spatialite.h>

#include <filesystem>
#include <mutex>

namespace {

struct sqlite_singleton_t {
  static const sqlite_singleton_t& get_instance() {
    static sqlite_singleton_t s;
    return s;
  }

private:
  sqlite_singleton_t() {
    // Disable sqlite3 internal memory tracking (results in a high-contention mutex, and we don't care
    // about marginal sqlite memory usage).
    sqlite3_config(SQLITE_CONFIG_MEMSTATUS, false);
  }
};

struct spatialite_singleton_t {
  static const spatialite_singleton_t& get_instance() {
    static spatialite_singleton_t s;
    return s;
  }

private:
  spatialite_singleton_t() {
    spatialite_initialize();
  }
  ~spatialite_singleton_t() {
    spatialite_shutdown();
  }
};

} // namespace

namespace valhalla {
namespace mjolnir {

std::optional<Sqlite3> Sqlite3::open(const std::string& path, int flags) {
  if (path.empty()) {
    return {};
  }

  if (!(flags & SQLITE_OPEN_CREATE) && !std::filesystem::exists(path)) {
    return {};
  }

  sqlite_singleton_t::get_instance();

  sqlite3* db = nullptr;
  uint32_t ret = sqlite3_open_v2(path.c_str(), &db, flags, nullptr);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + path);
    sqlite3_close(db);
    return {};
  }

  sqlite3_extended_result_codes(db, 1);
  spatialite_singleton_t::get_instance();

  void* conn = spatialite_alloc_connection();
  spatialite_init_ex(db, conn, 0);

  return Sqlite3(db, conn);
}

Sqlite3::~Sqlite3() {
  if (!db) {
    return; // No-op for moved-out objects.
  }

  // Sadly, `spatialite_cleanup_ex` calls `xmlCleanupParser()` (via `free_internal_cache()`) which is
  // not thread-safe and may cause a crash on double-free if called from multiple threads.
  // This static mutex works around the issue until the spatialite library is fixed:
  // - https://www.gaia-gis.it/fossil/libspatialite/tktview/855ef62a68b9ac6e500b54883707b2876c390c01
  // For full "double free" issue details follow https://github.com/valhalla/valhalla/issues/4904
  static std::mutex spatialite_mutex;
  {
    std::lock_guard<std::mutex> lock(spatialite_mutex);
    spatialite_cleanup_ex(spatialite);
  }
  sqlite3_close(db);
}

} // namespace mjolnir
} // namespace valhalla
