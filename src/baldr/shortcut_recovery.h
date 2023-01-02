#pragma once

#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
// TODO: break this out into a private header
// a static cache for shortcut recovery that we can optionally pre-fill
struct shortcut_recovery_t {
protected:
  /**
   * Constructs a shortcut cache from a graphreaders tileset by recovering all shortcuts. If the
   * graphreader passed in is null nothing is cached and revoery will happen on the fly
   * @param reader
   */
  shortcut_recovery_t(valhalla::baldr::GraphReader* reader,
                      bool shortcut_to_edge_cache,
                      bool edge_to_shortcut_cache)
      : shortcut_to_edge_cache_enabled(shortcut_to_edge_cache),
        edge_to_shortcut_cache_enabled(edge_to_shortcut_cache) {
    // do nothing if the reader is no good or both directions are disabled
    if (!reader || (!shortcut_to_edge_cache && !edge_to_shortcut_cache)) {
      LOG_INFO("Shortcut recovery cache disabled");
      return;
    }
    if (shortcut_to_edge_cache)
      LOG_INFO("Shortcut to edge recovery cache enabled");
    if (edge_to_shortcut_cache)
      LOG_INFO("Edge to shortcut recovery cache enabled");

    // completely skip the levels that dont have shortcuts
    for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
      // we dont get shortcuts on level 2 and up
      if (level.level > 1)
        continue;
      // for each tile
      for (auto tile_id : reader->GetTileSet(level.level)) {
        // cull cache if we are over allocated
        if (reader->OverCommitted())
          reader->Trim();
        // this shouldnt fail but garbled files could cause it
        auto tile = reader->GetGraphTile(tile_id);
        assert(tile);
        // for each edge in the tile
        for (const auto& edge : tile->GetDirectedEdges()) {
          // skip non-shortcuts or the shortcut is one we wont use
          if (!edge.shortcut())
            continue;
          auto shortcut_id = tile->header()->graphid();
          shortcut_id.set_id(&edge - tile->directededge(0));
          // skip already found opposing edges
          if (shortcuts.find(shortcut_id) != shortcuts.end())
            continue;
          // recover the shortcut and make a copy for opposing direction
          auto recovered = recover_shortcut(*reader, shortcut_id);
          decltype(recovered) opp_recovered = recovered;
          std::reverse_copy(recovered.cbegin(), recovered.cend(), opp_recovered.begin());
          // save some stats
          bool failed = recovered.front() == shortcut_id;
          unrecovered += failed;
          superseded += failed ? 0 : recovered.size();
          // cache it even if it failed (no point in trying the same thing twice)
          if (edge_to_shortcut_cache) {
            for (auto recovered_edge : recovered) {
              superseded_edges.emplace(recovered_edge, shortcut_id);
            }
          }
          if (shortcut_to_edge_cache) {
            shortcuts.emplace(shortcut_id, std::move(recovered));
          }

          // its cheaper to get the opposing without crawling the graph
          auto opp_tile = tile;
          auto opp_id = reader->GetOpposingEdgeId(shortcut_id, opp_tile);
          if (!opp_id.Is_Valid())
            continue; // dont store edges which arent in our tileset

          for (auto& id : opp_recovered) {
            id = reader->GetOpposingEdgeId(id, opp_tile);
            if (!id.Is_Valid()) {
              opp_recovered = {opp_id};
              break;
            }
          }
          // stats
          failed = opp_recovered.front() == opp_id;
          unrecovered += failed;
          superseded += failed ? 0 : opp_recovered.size();
          // cache it even if it failed (no point in trying the same thing twice)
          if (edge_to_shortcut_cache) {
            for (auto recovered_edge : opp_recovered) {
              superseded_edges.emplace(recovered_edge, opp_id);
            }
          }
          if (shortcut_to_edge_cache) {
            shortcuts.emplace(opp_id, std::move(opp_recovered));
          }
        }
      }
    }

    LOG_INFO(std::to_string(shortcuts.size()) + " shortcuts recovered as " +
             std::to_string(superseded) + " superseded edges. " + std::to_string(unrecovered) +
             " shortcuts could not be recovered.");
  }

  /**
   * Recovers the edges comprising a shortcut edge.
   * @param reader       The GraphReader for graph data access
   * @param  shortcutid  GraphId of the shortcut edge.
   * @return Returns the GraphIds of the directed edges this shortcut represents.
   */
  std::vector<valhalla::baldr::GraphId>
  recover_shortcut(valhalla::baldr::GraphReader& reader,
                   const valhalla::baldr::GraphId& shortcut_id) const {
    using namespace valhalla::baldr;
    // grab the shortcut edge
    auto tile = reader.GetGraphTile(shortcut_id);
    assert(tile);
    const DirectedEdge* shortcut = tile->directededge(shortcut_id);

    // bail if this isnt a shortcut
    if (!shortcut->is_shortcut()) {
      return {shortcut_id};
    }

    // loop over the edges leaving its begin node and find the superseded edge
    GraphId begin_node = reader.edge_startnode(shortcut_id);
    if (!begin_node)
      return {shortcut_id};

    // loop over the edges leaving its begin node and find the superseded edge
    std::vector<GraphId> edges;
    for (const DirectedEdge& de : tile->GetDirectedEdges(begin_node.id())) {
      if (shortcut->shortcut() & de.superseded()) {
        edges.push_back(tile->header()->graphid());
        edges.back().set_id(&de - tile->directededge(0));
        break;
      }
    }

    // bail if we couldnt find it
    if (edges.empty()) {
      LOG_TRACE("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
                " | no superseded edge");
      return {shortcut_id};
    }

    // seed the edge walking with the first edge
    const DirectedEdge* current_edge = tile->directededge(edges.back());
    uint32_t accumulated_length = current_edge->length();

    // walk edges until we find the same ending node as the shortcut
    while (current_edge->endnode() != shortcut->endnode()) {
      // get the node at the end of the last edge we added
      const NodeInfo* node = reader.GetEndNode(current_edge, tile);
      if (!node)
        return {shortcut_id};
      auto node_index = node - tile->node(0);

      // check the edges leaving this node to see if we can find the one that is part of the shortcut
      current_edge = nullptr;
      for (const DirectedEdge& edge : tile->GetDirectedEdges(node_index)) {
        // are they the same enough that its part of the shortcut
        // NOTE: this fails in about .05% of cases where there are two candidates and its not clear
        // which edge is the right one. looking at shortcut builder its not obvious how this is
        // possible as it seems to terminate a shortcut if more than one edge pair can be
        // contracted... NOTE: because we change the speed of the edge in graph enhancer we cant use
        // speed as a reliable determining factor
        if (begin_node != edge.endnode() && !edge.is_shortcut() &&
            edge.forwardaccess() == shortcut->forwardaccess() &&
            edge.reverseaccess() == shortcut->reverseaccess() && edge.sign() == shortcut->sign() &&
            edge.use() == shortcut->use() && edge.classification() == shortcut->classification() &&
            edge.roundabout() == shortcut->roundabout() && edge.link() == shortcut->link() &&
            edge.toll() == shortcut->toll() && edge.destonly() == shortcut->destonly() &&
            edge.unpaved() == shortcut->unpaved() && edge.surface() == shortcut->surface() &&
            edge.use() != Use::kConstruction /*&& edge.speed() == shortcut->speed()*/) {
          // we are going to keep this edge
          edges.emplace_back(tile->header()->graphid());
          edges.back().set_id(&edge - tile->directededge(0));
          // and keep expanding from the end of it
          current_edge = &edge;
          begin_node = tile->header()->graphid();
          begin_node.set_id(node_index);
          accumulated_length += edge.length();
          break;
        }
      }

      // if we didnt add an edge or we went over the length we failed
      if (current_edge == nullptr || accumulated_length > shortcut->length()) {
        LOG_TRACE("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
                  " | accumulated_length: " + std::to_string(accumulated_length) +
                  " | shortcut_length: " + std::to_string(shortcut->length()));
        return {shortcut_id};
      }
    }

    // we somehow got to the end via a shorter path
    if (accumulated_length < shortcut->length()) {
      LOG_TRACE("Unable to recover shortcut for edgeid (accumulated_length < shortcut->length()) " +
                std::to_string(shortcut_id) +
                " | accumulated_length: " + std::to_string(accumulated_length) +
                " | shortcut_length: " + std::to_string(shortcut->length()));
      return {shortcut_id};
    }

    // these edges make up this shortcut
    return edges;
  }

  /**
   * Finds the shortcut that supersedes the given edge.
   * @param reader   The GraphReader for graph data access
   * @param id       GraphId of the edge.
   * @return Returns the GraphId of the shortcut that supersedes the edge
   *         or an invalid GraphId if the edge is not part of any shortcut.
   */
  valhalla::baldr::GraphId find_shortcut(valhalla::baldr::GraphReader& reader,
                                         const valhalla::baldr::GraphId& id) const {
    using namespace valhalla::baldr;
    // Lambda to get continuing edge at a node. Skips the specified edge Id
    // transition edges, shortcut edges, and transit connections. Returns
    // nullptr if more than one edge remains or no continuing edge is found.
    auto continuing_edge = [](const graph_tile_ptr& tile, const GraphId& edgeid,
                              const NodeInfo* nodeinfo) {
      uint32_t idx = nodeinfo->edge_index();
      const DirectedEdge* continuing_edge = static_cast<const DirectedEdge*>(nullptr);
      const DirectedEdge* directededge = tile->directededge(idx);
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, idx++) {
        if (idx == edgeid.id() || directededge->is_shortcut() ||
            directededge->use() == Use::kTransitConnection ||
            directededge->use() == Use::kEgressConnection ||
            directededge->use() == Use::kPlatformConnection) {
          continue;
        }
        if (continuing_edge != nullptr) {
          return static_cast<const DirectedEdge*>(nullptr);
        }
        continuing_edge = directededge;
      }
      return continuing_edge;
    };

    // No shortcuts on the local level or transit level.
    if (id.level() >= TileHierarchy::levels().back().level) {
      return {};
    }

    // If this edge is a shortcut return this edge Id
    graph_tile_ptr tile = reader.GetGraphTile(id);
    const DirectedEdge* directededge = tile->directededge(id);
    if (directededge->is_shortcut()) {
      return id;
    }

    // Walk backwards along the opposing directed edge until a shortcut
    // beginning is found or to get the continuing edge until a node that starts
    // the shortcut is found or there are 2 or more other regular edges at the
    // node.
    GraphId edgeid = id;
    const NodeInfo* node = nullptr;
    const DirectedEdge* cont_de = nullptr;
    while (true) {
      // Get the continuing directed edge. Initial case is to use the opposing
      // directed edge.
      cont_de = (node == nullptr) ? reader.GetOpposingEdge(id) : continuing_edge(tile, edgeid, node);
      if (cont_de == nullptr) {
        return {};
      }

      // Get the end node and end node tile
      GraphId endnode = cont_de->endnode();
      if (cont_de->leaves_tile()) {
        tile = reader.GetGraphTile(endnode.Tile_Base());
      }
      node = tile->node(endnode);

      // Get the opposing edge Id and its directed edge
      uint32_t idx = node->edge_index() + cont_de->opp_index();
      edgeid = {endnode.tileid(), endnode.level(), idx};
      directededge = tile->directededge(edgeid);
      if (directededge->superseded()) {
        // Get the shortcut edge Id that supersedes this edge
        uint32_t idx = node->edge_index() + (directededge->superseded() - 1);
        return GraphId(endnode.tileid(), endnode.level(), idx);
      }
    }
    return {};
  }

  // a place to cache the recovered shortcuts
  std::unordered_map<valhalla::baldr::GraphId, std::vector<valhalla::baldr::GraphId>> shortcuts;
  // a place to cache the shortcut membership
  std::unordered_map<valhalla::baldr::GraphId, valhalla::baldr::GraphId> superseded_edges;
  // a place to keep some stats about the recovery
  size_t unrecovered;
  size_t superseded;

  bool shortcut_to_edge_cache_enabled;
  bool edge_to_shortcut_cache_enabled;

public:
  /**
   * returns a static instance of the cache after prefilling it. if on the first call,
   * the reader is nullptr then the cache will not be filled and recovery will be on the fly
   *
   * @param reader       the reader used to initialize the cache the first time
   * @return a filled cache mapping shortcuts to superceeded edges
   */
  static shortcut_recovery_t& get_instance(valhalla::baldr::GraphReader* reader = nullptr,
                                           bool shortcut_to_edge_cache = true,
                                           bool edge_to_shortcut_cache = true) {
    static shortcut_recovery_t cache{reader, shortcut_to_edge_cache, edge_to_shortcut_cache};
    return cache;
  }

  /**
   * returns the list of graphids of the edges superceded by the provided shortcut. saddly because we
   * may have to recover the shortcut on the fly we cannot return const reference here
   *
   * @param shortcut_id   the shortcuts edge id
   * @return the list of superceded edges
   */
  std::vector<valhalla::baldr::GraphId> get(const valhalla::baldr::GraphId& shortcut_id,
                                            valhalla::baldr::GraphReader& reader) const {
    // in the case that we didnt fill the cache we fallback to recovering on the fly
    if (!shortcut_to_edge_cache_enabled) {
      return recover_shortcut(reader, shortcut_id);
    }

    auto itr = shortcuts.find(shortcut_id);
    if (itr != shortcuts.cend())
      return itr->second;

    return {};
  }

  /**
   * returns the graphid of the shortcut that supersedes the provided edge.
   *
   * @param shortcut_id   the shortcuts edge id
   * @return the list of superceded edges
   */
  valhalla::baldr::GraphId get_shortcut(const valhalla::baldr::GraphId& edge_id,
                                        valhalla::baldr::GraphReader& reader) const {
    // in the case that we didnt fill the cache we fallback to recovering on the fly
    if (!edge_to_shortcut_cache_enabled) {
      return find_shortcut(reader, edge_id);
    }

    auto itr = superseded_edges.find(edge_id);
    if (itr != superseded_edges.cend())
      return itr->second;

    return {};
  }
};

} // namespace
