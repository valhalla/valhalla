#include "baldr/tilehierarchy.h"
#include "config.h"
#include "filesystem.h"
#include "loki/worker.h"
#include "proto/status.pb.h"

using namespace valhalla::baldr;

namespace {

auto get_graphtile(const std::shared_ptr<GraphReader>& reader) {
  graph_tile_ptr tile = nullptr;
  for (const auto& tile_id : reader->GetTileSet()) {
    tile = reader->GetGraphTile(tile_id);
    if (tile->id().level() < TileHierarchy::GetTransitLevel().level &&
        tile->header()->nodecount() > 0) {
      break;
    }
  }
  return tile;
}

time_t get_tileset_last_modified(const std::shared_ptr<GraphReader>& reader) {
  auto path = reader->GetTileSetLocation();
  try {
    return std::chrono::system_clock::to_time_t(filesystem::last_write_time(path));
  } catch (...) {}
  return 0;
}

} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::status(Api& request) const {
#ifdef ENABLE_SERVICES
  // if we are in the process of shutting down we signal that here
  // should react by draining traffic (though they are likely doing this as they are usually the ones
  // who sent us the request to shutdown)
  if (prime_server::draining() || prime_server::shutting_down()) {
    throw valhalla_exception_t{102};
  }
#endif

  // info that's always returned
  auto* status = request.mutable_status();
  status->set_version(VALHALLA_VERSION);
  status->set_tileset_last_modified(get_tileset_last_modified(reader));
  for (const auto& action : actions) {
    auto* action_pbf = status->mutable_available_actions()->Add();
    *action_pbf = Options_Action_Enum_Name(action);
  }

  // only return more info if explicitly asked for (can be very expensive)
  if (!request.options().verbose() || !allow_verbose)
    return;

  // get _some_ tile
  const static baldr::graph_tile_ptr tile = get_graphtile(reader);

  if (connectivity_map) {
    status->set_bbox(connectivity_map->to_geojson(2));
    const bool has_transit_tiles =
        connectivity_map->level_color_exists(TileHierarchy::GetTransitLevel().level);
    status->set_has_transit_tiles(has_transit_tiles);
  } else {
    const static bool has_transit_tiles = !reader->GetTileSet(3).empty();
    status->set_has_transit_tiles(has_transit_tiles);
  }

  status->set_has_tiles(static_cast<bool>(tile));
  status->set_has_admins(tile && tile->header()->admincount() > 0);
  status->set_has_timezones(tile && tile->node(0)->timezone() > 0);
  status->set_has_live_traffic(reader->HasLiveTraffic());
  status->set_osm_changeset(tile ? tile->header()->dataset_id() : 0);
}
} // namespace loki
} // namespace valhalla
