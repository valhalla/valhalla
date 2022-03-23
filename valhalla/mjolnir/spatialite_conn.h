#ifndef VALHALLA_MJOLNIR_SPATIALITE_CONN_H_
#define VALHALLA_MJOLNIR_SPATIALITE_CONN_H_

#include <memory>
#include <sqlite3.h>

#include <spatialite.h>

namespace valhalla {
namespace mjolnir {

// wrap the connection in a shared pointer so its automatically cleaned up when no longer used
std::shared_ptr<void> make_spatialite_cache(sqlite3* handle);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_SPATIALITE_CONN_H_
