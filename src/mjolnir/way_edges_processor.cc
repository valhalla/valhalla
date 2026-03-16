#include "mjolnir/way_edges_processor.h"
#include "baldr/edgeinfo.h"

#include <fstream>

namespace valhalla {
namespace mjolnir {

namespace {
void write_way_edges(const std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>& ways_edges,
                     const std::string& fname) {
  std::ofstream ways_file;
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);

  for (const auto& way : ways_edges) {
    ways_file << way.first;
    for (auto edge : way.second) {
      ways_file << "," << (uint32_t)edge.forward << "," << (uint64_t)edge.edgeid;
    }
    ways_file << std::endl;
  }
  ways_file.close();
}
} // namespace

std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>
collect_way_edges(baldr::GraphReader& reader, const std::string& filename) {
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  // Iterate through all tiles
  for (auto edge_id : reader.GetTileSet()) {
    // If tile does not exist, skip
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }

    // Trim reader if over-committed
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    baldr::graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const baldr::DirectedEdge* edge = tile->directededge(edge_id);

      // Skip transit, connection, and shortcut edges
      if (edge->IsTransitLine() || edge->use() == baldr::Use::kTransitConnection ||
          edge->use() == baldr::Use::kEgressConnection ||
          edge->use() == baldr::Use::kPlatformConnection || edge->is_shortcut()) {
        continue;
      }

      // Skip if the edge does not allow auto use
      if (!(edge->forwardaccess() & baldr::kAutoAccess)) {
        continue;
      }

      // Get the way Id and store edge information
      uint64_t wayid = tile->edgeinfo(edge).wayid();
      ways_edges[wayid].push_back({edge->forward(), edge_id});
    }
  }
  if (!filename.empty()) {
    write_way_edges(ways_edges, filename);
  }

  return ways_edges;
}
} // namespace mjolnir
} // namespace valhalla
