#include "baldr/tilehierarchy.h"
#include "config.h"
#include "loki/worker.h"
#include "proto/status.pb.h"

namespace {
auto get_graphtile(const std::shared_ptr<valhalla::baldr::GraphReader>& reader) {
  graph_tile_ptr tile = nullptr;
  for (const auto& tile_id : reader->GetTileSet()) {
    tile = reader->GetGraphTile(tile_id);
    if (tile->id().level() < valhalla::baldr::TileHierarchy::GetTransitLevel().level &&
        tile->header()->nodecount() > 0) {
      break;
    }
  }
  return tile;
}
} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::status(Api& request) const {

  auto* status = request.mutable_status();

  // only return more info if explicitly asked for (can be very expensive)
  // bail if we wont be getting extra info
  if (!request.options().verbose() || !allow_verbose)
    return;

  // get _some_ tile
  const static baldr::graph_tile_ptr tile = get_graphtile(reader);

  if (connectivity_map) {
    status->set_bbox(connectivity_map->to_geojson(2));
  }

  status->set_has_tiles(static_cast<bool>(tile));
  status->set_has_admins(tile && tile->header()->admincount() > 0);
  status->set_has_timezones(tile && tile->node(0)->timezone() > 0);
  status->set_has_live_traffic(reader->HasLiveTraffic());
  status->set_version(VALHALLA_VERSION);

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
