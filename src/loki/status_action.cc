#include <sys/stat.h>
#include <sys/types.h>
#ifndef WIN32
#include <unistd.h>
#else
#define stat _stat
#endif

#include "baldr/tilehierarchy.h"
#include "config.h"
#include "filesystem.h"
#include "loki/worker.h"
#include "proto/status.pb.h"

namespace {
#ifdef _WIN32
#define MTIME(st_stat) st_stat.st_mtime
#elif __APPLE__
#define MTIME(st_stat) st_stat.st_mtime
#else
#define MTIME(st_stat) st_stat.st_mtim.tv_sec
#endif

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

int get_tile_age(const boost::property_tree::ptree& conf) {
  // prefer tile_dir mtime over tile_extract
  struct stat s;
  for (const auto& m : {"tile_dir", "tile_extract"}) {
    auto p = conf.get_optional<std::string>(m);
    if (p && stat(p.get().c_str(), &s) == 0) {
      return MTIME(s);
    }
  }

  return 0;
}

} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::status(Api& request) const {

  auto* status = request.mutable_status();
  status->set_version(VALHALLA_VERSION);
  status->set_tile_age(get_tile_age(config.get_child("mjolnir")));

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
