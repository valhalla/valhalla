#include "mjolnir/way_edges_processor.h"
#include "baldr/edgeinfo.h"

#include <osmium/handler.hpp>
#include <osmium/io/file.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/io/reader_with_progress_bar.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

#include <fstream>

namespace {

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

using way_edges_map_t = std::unordered_map<uint64_t, std::vector<way_edge>>;

class WayEdgesSorter : public osmium::handler::Handler {
public:
  WayEdgesSorter(GraphReader& reader, way_edges_map_t& mapping)
      : reader_(reader), edge_mapping_(mapping) {
  }

  void way(const osmium::Way& way) {
    auto it = edge_mapping_.find(static_cast<uint64_t>(way.id()));
    if (it == edge_mapping_.end() || it->second.empty()) {
      return;
    }
    const auto& edges = it->second;

    const auto& way_nodes = way.nodes();
    if (way_nodes.size() < 2) {
      return;
    }

    // get the way's node IDs into a vector
    std::vector<uint64_t> way_node_ids;
    way_node_ids.reserve(way_nodes.size());
    for (const auto& nr : way_nodes) {
      way_node_ids.push_back(static_cast<uint64_t>(nr.ref()));
    }

    // closed loop ways could be broken up so that we end up with multiple
    // edges that share the same start node
    // note: size_t is the index into the edge_ids vector
    std::unordered_map<uint64_t, std::vector<std::pair<size_t, std::vector<uint64_t>>>> by_start_node;
    by_start_node.reserve(edges.size());

    // collect the node IDs per edge and throw if node IDs
    // are not available on any edge
    for (size_t i = 0; i < edges.size(); ++i) {
      auto eid = edges[i].edgeid;
      auto tile = reader_.GetGraphTile(eid);

      if (!tile) {
        throw tile_gone_error_t("collect_way_edges_sorted failed", eid);
      }

      const auto* de = tile->directededge(eid.id());
      auto ei = tile->edgeinfo(de);
      auto nids = ei.osm_node_ids();
      if (nids.empty()) {
        throw std::runtime_error("Way " + std::to_string(way.id()) + ": edge " +
                                 std::to_string(eid.value) + " has no stored node IDs");
      }
      const uint64_t start = nids.front();
      by_start_node[start].emplace_back(i, nids);
    }
    using vec_t = typename way_edges_map_t::mapped_type;
    vec_t ordered;
    ordered.reserve(edges.size());

    size_t cursor = 0;
    const size_t last = way_node_ids.size() - 1;

    // move through the way's node ID list
    while (cursor < last) {
      const uint64_t start_node = way_node_ids[cursor];

      // find our candidate by start node
      auto cand_it = by_start_node.find(start_node);
      if (cand_it == by_start_node.end() || cand_it->second.empty()) {
        throw std::runtime_error("Way " + std::to_string(way.id()) +
                                 ": no forward edge starts at node " + std::to_string(start_node) +
                                 " (position " + std::to_string(cursor) + ")");
      }

      auto& candidates = cand_it->second;

      // find the edge whose node ids match from the cursor onwards
      size_t chosen = std::numeric_limits<size_t>::max();
      for (size_t s = 0; s < candidates.size(); ++s) {
        const auto& edge_nodes = candidates[s].second;
        if (cursor + edge_nodes.size() > way_node_ids.size()) {
          continue;
        }

        // if the edge's nodes exactly match the upcoming way nodes
        // we're done
        if (std::equal(edge_nodes.begin(), edge_nodes.end(), way_node_ids.begin() + cursor)) {
          chosen = s;
          break;
        }
      }

      // didn't find a candidate
      if (chosen == std::numeric_limits<size_t>::max()) {
        throw std::runtime_error(
            "Way " + std::to_string(way.id()) + ": no edge at node " + std::to_string(start_node) +
            " matches the way's node sequence from position " + std::to_string(cursor));
      }

      const auto& picked = edges[candidates[chosen].first];
      ordered.emplace_back(
          way_edge{.edgeid = picked.edgeid, .length = picked.length, .forward = true});

      // this edge's end node is the next edges start node
      const size_t advance = candidates[chosen].second.size() - 1;

      // remove the picked candidate from our list of candidates
      std::swap(candidates[chosen], candidates.back());
      candidates.pop_back();

      cursor += advance;
    }

    if (ordered.size() != edges.size()) {
      throw std::runtime_error("Way " + std::to_string(way.id()) + ": ordered " +
                               std::to_string(ordered.size()) + " of " +
                               std::to_string(edges.size()) + " edges");
    }

    // finally, walk the ordered edges in reverse order and append their
    // evil twins
    for (size_t i = ordered.size(); i-- > 0;) {
      auto opp_id = reader_.GetOpposingEdgeId(ordered[i].edgeid);
      auto* opp_de = reader_.directededge(opp_id);
      if (opp_de->forward()) {
        throw std::runtime_error("Expected backward oriented edge for Way " +
                                 std::to_string(way.id()));
      }

      ordered.emplace_back(way_edge{.edgeid = opp_id, .length = opp_de->length(), .forward = false});
    }

    it->second = std::move(ordered);
  }

protected:
  GraphReader& reader_;
  way_edges_map_t& edge_mapping_;
};
} // namespace

namespace valhalla {
namespace mjolnir {

namespace {
void write_way_edges(const std::unordered_map<uint64_t, std::vector<way_edge>>& ways_edges,
                     const std::string& fname,
                     bool include_length = false) {
  std::ofstream ways_file;
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);

  for (const auto& way : ways_edges) {
    ways_file << way.first;
    for (const auto& edge : way.second) {
      ways_file << "," << (uint32_t)edge.forward << "," << (uint64_t)edge.edgeid;
      if (include_length) {
        ways_file << "," << edge.length;
      }
    }
    ways_file << std::endl;
  }
  ways_file.close();
}
} // namespace

std::unordered_map<uint64_t, std::vector<way_edge>> collect_way_edges(baldr::GraphReader& reader,
                                                                      const std::string& filename,
                                                                      uint32_t access_mask,
                                                                      bool skip_backward) {
  std::unordered_map<uint64_t, std::vector<way_edge>> ways_edges;

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
      if ((skip_backward && !edge->forward()) || edge->IsTransitLine() ||
          edge->use() == baldr::Use::kTransitConnection ||
          edge->use() == baldr::Use::kEgressConnection ||
          edge->use() == baldr::Use::kPlatformConnection || edge->is_shortcut()) {
        continue;
      }

      // Skip if the edge is filtered by access
      if (!(edge->forwardaccess() & access_mask)) {
        continue;
      }

      // Get the way Id and store edge information
      uint64_t wayid = tile->edgeinfo(edge).wayid();
      ways_edges[wayid].emplace_back(
          way_edge{.edgeid = edge_id, .length = edge->length(), .forward = edge->forward()});
    }
  }

  if (!filename.empty()) {
    write_way_edges(ways_edges, filename);
  }

  return ways_edges;
}

std::unordered_map<uint64_t, std::vector<way_edge>>
collect_way_edges_sorted(const std::string& osm_filename,
                         baldr::GraphReader& reader,
                         bool include_length,
                         const std::string& filename,
                         uint32_t access_mask) {
  // skip the backward edges in the first sweep, we will get them in once
  // we've sorted the forward edges
  auto ways_edges = collect_way_edges(reader, filename, access_mask, true);

  LOG_INFO("Collected ways to edges mapping, sorting edges...");

  const osmium::io::File input_file{osm_filename};
  osmium::io::ReaderWithProgressBar osm_reader{true, input_file, osmium::osm_entity_bits::way};

  WayEdgesSorter handler(reader, ways_edges);
  osmium::apply(osm_reader, handler);

  if (!filename.empty()) {
    write_way_edges(ways_edges, filename, include_length);
  }

  return ways_edges;
}
} // namespace mjolnir
} // namespace valhalla
