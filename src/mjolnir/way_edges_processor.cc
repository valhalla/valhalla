#include "mjolnir/way_edges_processor.h"
#include "baldr/edgeinfo.h"

#include <fstream>
// #include <iostream>
namespace valhalla {
namespace mjolnir {

namespace {
void write_way_edges(const std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>& ways_edges,
                     const std::string& fname) {
  std::ofstream ways_file;
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);

  for (const auto& way : ways_edges) {
    ways_file << way.first;

    bool isFirst = true;
    for (auto edge : way.second) {
      if (isFirst) {
        ways_file << "," << (uint32_t)edge.forward << "|" << (uint64_t)edge.edgeid << "|" << edge.length << "|" << (uint32_t)edge.shortcut;
        isFirst = false;
      }
      else {
        ways_file << "|" << (uint32_t)edge.forward << "|" << (uint64_t)edge.edgeid << "|" << edge.length << "|" << (uint32_t)edge.shortcut;
      }
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
          edge->use() == baldr::Use::kPlatformConnection) {
        continue;
      }

      // Skip if the edge does not allow auto use
      if (!(edge->forwardaccess() & baldr::kAutoAccess)) {
        continue;
      }

      if (edge->is_shortcut()) {
        auto edges = reader.RecoverShortcut(edge_id);
        for (auto sub_edge_id : edges) {
          baldr::graph_tile_ptr sub_edge_tile = reader.GetGraphTile(sub_edge_id);
          const baldr::DirectedEdge* sub_edge = sub_edge_tile->directededge(sub_edge_id);

          uint64_t wayid = tile->edgeinfo(sub_edge).wayid();
          ways_edges[wayid].push_back({edge->forward(), edge_id, edge->length(), true});
        }
      } else {
        // Get the way Id and store edge information
        uint64_t wayid = tile->edgeinfo(edge).wayid();
        ways_edges[wayid].push_back({edge->forward(), edge_id, edge->length(), false});
      }
    }
  }
  if (!filename.empty()) {
    write_way_edges(ways_edges, filename);
  }

  return ways_edges;
}
} // namespace mjolnir
} // namespace valhalla
