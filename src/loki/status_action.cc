#include "loki/worker.h"
#include "proto/status.pb.h"

namespace valhalla {
namespace loki {
void loki_worker_t::status(Api& request) const {
  baldr::graph_tile_ptr tile = nullptr;
  for (const auto& tile_id : reader->GetTileSet(2)) {
    if (reader->DoesTileExist(tile_id)) {
      tile = reader->GetGraphTile(tile_id);
      break;
    }
  }

  auto* status = request.mutable_status();
  status->set_has_tiles(!tile ? false : true);
  status->set_has_admins(!tile ? false : tile->header()->admincount() > 0);
  status->set_has_timezones(!tile ? false : tile->node(0)->timezone() > 0);
  status->set_has_live_traffic(reader->DoTrafficTilesExist());

#ifdef HAVE_HTTP
  // if we are in the process of shutting down we signal that here
  // should react by draining traffic (though they are likely doing this as they are usually the ones
  // who sent us the request to shutdown)
  if (prime_server::draining() || prime_server::shutting_down()) {
    throw valhalla_exception_t{102};
  }
#endif
}
} // namespace loki
} // namespace valhalla
