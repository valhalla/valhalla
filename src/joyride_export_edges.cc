// joyride_export_edges
//
// Walk every drivable edge in the base Valhalla tileset and emit one CSV
// line per edge:
//
//   edge_id,centroid_lon,centroid_lat,freeflow_kmh,constrained_kmh
//
// where edge_id is Valhalla's GraphId serialized as "level/tile/index".
// Used by the Joyride dryness-rebake pipeline to associate each Valhalla
// edge with its weather grid cell and baseline speeds.

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cstdio>
#include <cstdint>

using namespace valhalla::baldr;

int main(int argc, char** argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <valhalla.json>\n", argv[0]);
    return 1;
  }

  boost::property_tree::ptree cfg;
  boost::property_tree::read_json(argv[1], cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    if (!tile) continue;

    const uint32_t n = tile->header()->directededgecount();
    for (uint32_t i = 0; i < n; ++i) {
      GraphId eid = tile_id;
      eid.set_id(i);
      const DirectedEdge* e = tile->directededge(i);
      if (!(e->forwardaccess() & kAutoAccess)) continue;
      if (e->is_shortcut()) continue;

      auto shape = tile->edgeinfo(e).shape();
      if (shape.empty()) continue;
      const auto& mid = shape[shape.size() / 2];

      // Baseline speeds. If the tile lacks predicted-speed data, GetSpeed
      // falls back to e->speed(). In either case we only need a stable
      // OSM-derived baseline to preserve in our rewritten CSV.
      float freeflow    = static_cast<float>(tile->GetSpeed(e, kFreeFlowMask, 0));
      float constrained = static_cast<float>(tile->GetSpeed(e, kConstrainedFlowMask, 0));
      if (freeflow    <= 0.0f) freeflow    = static_cast<float>(e->speed());
      if (constrained <= 0.0f) constrained = freeflow;

      std::printf("%u/%u/%u,%.6f,%.6f,%.1f,%.1f\n",
                  eid.level(), eid.tileid(), eid.id(),
                  mid.lng(), mid.lat(),
                  freeflow, constrained);
    }
  }
  return 0;
}
