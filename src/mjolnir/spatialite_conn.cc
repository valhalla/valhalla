#include "mjolnir/spatialite_conn.h"

namespace {
// we call initialize once the first time its accessed and we shutdown once when it goes out of scope
// (end of program)
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
std::shared_ptr<void> make_spatialite_cache(sqlite3* handle) {
  if (!handle) {
    return nullptr;
  }

  spatialite_singleton_t::get_instance();
  void* conn = spatialite_alloc_connection();
  spatialite_init_ex(handle, conn, 0);
  return std::shared_ptr<void>(conn, [](void* c) { spatialite_cleanup_ex(c); });
}

} // namespace mjolnir
} // namespace valhalla
