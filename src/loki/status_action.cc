#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <sstream>

#include "baldr/tilehierarchy.h"
#include "loki/worker.h"
#include "proto/status.pb.h"

namespace valhalla {
namespace loki {
void loki_worker_t::status(Api& request) const {
  // get _some_ tile
  static baldr::graph_tile_ptr tile = nullptr;
  if (!tile) {
    for (const auto& tile_id : reader->GetTileSet()) {
      tile = reader->GetGraphTile(tile_id);
      if (tile->id().level() < baldr::TileHierarchy::GetTransitLevel().level &&
          tile->header()->nodecount() > 0) {
        break;
      }
    }
  }

  // bounding box geojson string from connectivity map
  static std::string bbox = "";
  if (bbox == "" && connectivity_map) {
    bbox = connectivity_map->to_geojson(2);
  }

  auto* status = request.mutable_status();
  status->set_bbox(bbox);
  status->set_has_tiles(!tile ? false : true);
  status->set_has_admins(!tile ? false : tile->header()->admincount() > 0);
  status->set_has_timezones(!tile ? false : tile->node(0)->timezone() > 0);
  status->set_has_live_traffic(reader->HasLiveTraffic());
  status->set_actions(action_str);

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
