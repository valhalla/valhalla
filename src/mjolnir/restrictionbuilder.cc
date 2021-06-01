#include "mjolnir/restrictionbuilder.h"
#include "mjolnir/complexrestrictionbuilder.h"
#include "mjolnir/dataquality.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmrestriction.h"

#include <future>
#include <queue>
#include <set>
#include <thread>
#include <unordered_set>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "baldr/timedomain.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct EdgeId {
  uint64_t way_id;
  GraphId graph_id;
};

GraphId GetOpposingEdge(GraphReader& reader,
                        std::mutex& lock,
                        const valhalla::baldr::graph_tile_ptr& tile,
                        GraphId node,
                        const DirectedEdge* edge) {
  GraphId end_node = edge->endnode();
  auto end_node_tile = tile;
  if (end_node_tile->id() != end_node.Tile_Base()) {
    lock.lock();
    end_node_tile = reader.GetGraphTile(end_node);
    lock.unlock();
  }
  const NodeInfo* nodeinfo = end_node_tile->node(end_node);
  auto way_id = tile->edgeinfo(edge).wayid();

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  GraphId opp_id(end_node.tileid(), end_node.level(), nodeinfo->edge_index());
  const DirectedEdge* opp_edge = end_node_tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, opp_edge++, ++opp_id) {
    if (opp_edge->use() == Use::kTransitConnection || opp_edge->use() == Use::kEgressConnection ||
        opp_edge->use() == Use::kPlatformConnection) {
      continue;
    }
    if (opp_edge->endnode() == node && opp_edge->classification() == edge->classification() &&
        opp_edge->length() == edge->length() &&
        ((opp_edge->link() && edge->link()) || (opp_edge->use() == edge->use())) &&
        way_id == end_node_tile->edgeinfo(opp_edge).wayid()) {
      return opp_id;
    }
  }
  PointLL ll = nodeinfo->latlng(end_node_tile->header()->base_ll());
  LOG_ERROR("Opposing directed edge not found at LL= " + std::to_string(ll.lat()) + "," +
            std::to_string(ll.lng()));
  return {};
}

bool ExpandFromNode(GraphReader& reader,
                    std::mutex& lock,
                    uint32_t access,
                    bool forward,
                    GraphId& last_node,
                    std::unordered_set<GraphId>& visited_nodes,
                    std::vector<EdgeId>& edge_ids,
                    const std::vector<uint64_t>& way_ids,
                    size_t way_id_index,
                    const graph_tile_ptr& prev_tile,
                    GraphId prev_node,
                    GraphId current_node);

bool IsEdgeAllowed(const DirectedEdge* de, uint32_t access, bool forward) {
  bool accessible = (forward ? de->forwardaccess() : de->reverseaccess()) & access;
  return accessible &&
         !(de->IsTransitLine() || de->is_shortcut() || de->use() == Use::kTransitConnection ||
           de->use() == Use::kEgressConnection || de->use() == Use::kPlatformConnection);
}

bool ExpandFromNodeInner(GraphReader& reader,
                         std::mutex& lock,
                         uint32_t access,
                         bool forward,
                         GraphId& last_node,
                         std::unordered_set<GraphId>& visited_nodes,
                         std::vector<EdgeId>& edge_ids,
                         const std::vector<uint64_t>& way_ids,
                         size_t way_id_index,
                         const graph_tile_ptr& tile,
                         GraphId prev_node,
                         GraphId current_node,
                         const NodeInfo* node_info) {
  uint64_t way_id = way_ids[way_id_index];

  for (size_t j = 0; j < node_info->edge_count(); ++j) {
    GraphId edge_id(tile->id().tileid(), tile->id().level(), node_info->edge_index() + j);
    const DirectedEdge* de = tile->directededge(edge_id);

    if (de->endnode() != prev_node && IsEdgeAllowed(de, access, forward)) {
      auto edge_info = tile->edgeinfo(de);
      if (edge_info.wayid() == way_id) {
        edge_ids.push_back({way_id, edge_id});

        bool found;
        // expand with the next way_id
        found = ExpandFromNode(reader, lock, access, forward, last_node, visited_nodes, edge_ids,
                               way_ids, way_id_index + 1, tile, current_node, de->endnode());
        if (found)
          return true;

        if (visited_nodes.find(de->endnode()) == visited_nodes.end()) {
          visited_nodes.insert(de->endnode());

          // expand with the same way_id
          found = ExpandFromNode(reader, lock, access, forward, last_node, visited_nodes, edge_ids,
                                 way_ids, way_id_index, tile, current_node, de->endnode());
          if (found)
            return true;

          visited_nodes.erase(de->endnode());
        }

        edge_ids.pop_back();
      }
    }
  }
  return false;
}

// The function does depth-first-search to convert way_ids to the edge_ids
//
//
// pseudo-code
//  def DepthFirstSearch(way_id, node):
//   if not way_id:
//     # we matched all way ids
//     return true
//   for edge in edges(node): # edges = directed_edges + transition_node_edges
//     if edge.way_id == way_id:
//       if DepthFirstSearch(next(way_id), edge.end_node):
//         return true
//       if DepthFirstSearch(way_id, edge.end_node):
//         return true
//    return false
bool ExpandFromNode(GraphReader& reader,
                    std::mutex& lock,
                    uint32_t access,
                    bool forward,
                    GraphId& last_node,
                    std::unordered_set<GraphId>& visited_nodes,
                    std::vector<EdgeId>& edge_ids,
                    const std::vector<uint64_t>& way_ids,
                    size_t way_id_index,
                    const graph_tile_ptr& prev_tile,
                    GraphId prev_node,
                    GraphId current_node) {
  if (way_id_index == way_ids.size()) {
    // assign last node to use it for the reverse search later
    last_node = current_node;
    return true;
  }

  auto tile = prev_tile;
  if (tile->id() != current_node.Tile_Base()) {
    lock.lock();
    tile = reader.GetGraphTile(current_node);
    lock.unlock();
  }

  auto node_info = tile->node(current_node);

  bool found;
  // expand from the current node
  found = ExpandFromNodeInner(reader, lock, access, forward, last_node, visited_nodes, edge_ids,
                              way_ids, way_id_index, tile, prev_node, current_node, node_info);
  if (found)
    return true;

  // expand from the transition nodes
  for (size_t k = 0; k < node_info->transition_count(); ++k) {
    const NodeTransition* trans = tile->transition(node_info->transition_index() + k);

    graph_tile_ptr trans_tile = tile;
    if (trans_tile->id() != trans->endnode().Tile_Base()) {
      lock.lock();
      trans_tile = reader.GetGraphTile(trans->endnode());
      lock.unlock();
    }

    found = ExpandFromNodeInner(reader, lock, access, forward, last_node, visited_nodes, edge_ids,
                                way_ids, way_id_index, trans_tile, prev_node, trans->endnode(),
                                trans_tile->node(trans->endnode()));
    if (found)
      return true;
  }

  return false;
}

std::vector<GraphId> GetGraphIds(GraphId& start_node,
                                 GraphReader& reader,
                                 std::mutex& lock,
                                 const std::vector<uint64_t>& way_ids,
                                 uint32_t access,
                                 bool forward) {
  lock.lock();
  graph_tile_ptr tile = reader.GetGraphTile(start_node);
  lock.unlock();

  std::unordered_set<GraphId> visited_nodes{start_node};
  std::vector<EdgeId> edge_ids;
  ExpandFromNode(reader, lock, access, forward, start_node, visited_nodes, edge_ids, way_ids, 0, tile,
                 GraphId(), start_node);
  if (edge_ids.empty())
    return {};

  // ignore duplicated way_ids in the prefix so [1, 1, 1, 2, 54] => [1, 2, 54]
  auto it = std::find_if(edge_ids.begin() + 1, edge_ids.end(),
                         [&](const EdgeId& id) { return id.way_id != edge_ids.front().way_id; });
  --it;
  std::vector<GraphId> res;
  res.reserve(edge_ids.end() - it);
  for (; it != edge_ids.end(); ++it) {
    res.push_back(it->graph_id);
  }
  return res;
}

ComplexRestrictionBuilder CreateComplexRestriction(const OSMRestriction& restriction,
                                                   const GraphId& from,
                                                   const GraphId& to,
                                                   const std::vector<GraphId>& vias) {
  ComplexRestrictionBuilder complex_restriction;
  complex_restriction.set_from_id(from);
  complex_restriction.set_via_list(vias);
  complex_restriction.set_to_id(to);
  complex_restriction.set_type(restriction.type());
  complex_restriction.set_modes(restriction.modes());

  TimeDomain td = TimeDomain(restriction.time_domain());
  if (td.td_value()) {
    complex_restriction.set_begin_day_dow(td.begin_day_dow());
    complex_restriction.set_begin_hrs(td.begin_hrs());
    complex_restriction.set_begin_mins(td.begin_mins());
    complex_restriction.set_begin_month(td.begin_month());
    complex_restriction.set_begin_week(td.begin_week());
    complex_restriction.set_dow(td.dow());
    complex_restriction.set_dt(true);
    complex_restriction.set_dt_type(td.type());
    complex_restriction.set_end_day_dow(td.end_day_dow());
    complex_restriction.set_end_hrs(td.end_hrs());
    complex_restriction.set_end_mins(td.end_mins());
    complex_restriction.set_end_month(td.end_month());
    complex_restriction.set_end_week(td.end_week());
  }

  return complex_restriction;
};

struct Result {
  uint32_t forward_restrictions_count;
  uint32_t reverse_restrictions_count;
  std::vector<ComplexRestrictionBuilder> restrictions;
  std::unordered_set<GraphId> part_of_restriction;
};

void HandleOnlyRestrictionProperties(const std::vector<Result>& results,
                                     const boost::property_tree::ptree& config) {
  std::unordered_map<GraphId, std::vector<const ComplexRestrictionBuilder*>> restrictions;
  std::unordered_map<GraphId, std::vector<GraphId>> part_of_restriction;
  for (const auto& res : results) {
    for (const auto& restriction : res.restrictions) {
      restrictions[restriction.to_graphid().Tile_Base()].push_back(&restriction);
    }
    for (const auto& edge_id : res.part_of_restriction) {
      part_of_restriction[edge_id.Tile_Base()].push_back(edge_id);
    }
  }

  GraphReader reader(config);
  for (const auto& i : restrictions) {
    GraphId tile_id = i.first;
    auto tile = reader.GetGraphTile(tile_id);
    if (!tile)
      continue;

    GraphTileBuilder tile_builder(reader.tile_dir(), tile_id, true);
    for (auto restriction : i.second) {
      tile_builder.AddForwardComplexRestriction(*restriction);
      DirectedEdge& edge = tile_builder.directededge_builder(restriction->to_graphid().id());
      edge.set_end_restriction(edge.end_restriction() | restriction->modes());
    }
    tile_builder.StoreTileData();
  }

  for (const auto& i : part_of_restriction) {
    GraphId tile_id = i.first;
    auto tile = reader.GetGraphTile(tile_id);
    if (!tile)
      continue;

    GraphTileBuilder tile_builder(reader.tile_dir(), tile_id, true);
    for (GraphId edge_id : i.second) {
      DirectedEdge& edge = tile_builder.directededge_builder(edge_id.id());
      edge.complex_restriction(true);
    }
    tile_builder.StoreTileData();
  }
}

void build(const std::string& complex_restriction_from_file,
           const std::string& complex_restriction_to_file,
           const boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue,
           std::mutex& lock,
           std::promise<Result>& result) {
  sequence<OSMRestriction> complex_restrictions_from(complex_restriction_from_file, false);
  sequence<OSMRestriction> complex_restrictions_to(complex_restriction_to_file, false);

  GraphReader reader(hierarchy_properties);
  Result stats;

  // Iterate through the tiles in the queue and perform enhancements
  while (true) {
    // Get the next tile Id from the queue and get writeable and readable
    // tile. Lock while we access the tile queue and get the tile.
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    tilequeue.pop();

    // Get a readable tile. If the tile is empty, skip it. Empty tiles are
    // added where ways go through a tile but no end not is within the tile.
    // This allows creation of connectivity maps using the tile set,
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    if (!tile) {
      lock.unlock();
      continue;
    }

    // Tile builder - serialize in existing tile
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, true);
    lock.unlock();

    std::unordered_multimap<GraphId, ComplexRestrictionBuilder> forward_tmp_cr;
    std::unordered_multimap<GraphId, ComplexRestrictionBuilder> reverse_tmp_cr;

    size_t forward_count = 0, reverse_count = 0;

    GraphId prevNode;
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      // Go through directed edges and "enhance" directed edge attributes
      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge = tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        if (directededge.IsTransitLine() || directededge.is_shortcut() ||
            directededge.use() == Use::kTransitConnection ||
            directededge.use() == Use::kEgressConnection ||
            directededge.use() == Use::kPlatformConnection) {
          continue;
        }
        auto edge_info = tilebuilder.edgeinfo(&directededge);
        //    |      |       |
        //    |      |  to   |
        // ---O------O---x---O---
        //       way c       |
        //                   |x via   way b
        //       way a       |
        // ---O------O---x---O---
        //    |      | from  |
        //    |      |       |
        // only want to store where edges are marked with a x

        // Starting with the "from" wayid.  If we are on the edge where the endnode has the via,
        // save the edgeid as the "from" and walk the vias until we reach the "to" wayid.  Save all
        // these edges as the complex restriction.  Note:  we may have to transition up or down to
        // other hierarchy levels as needed at endnodes.

        if (directededge.start_restriction()) {
          OSMRestriction target_res{edge_info.wayid()}; // this is our from way id
          sequence<OSMRestriction>::iterator res_it =
              complex_restrictions_from.find(target_res,
                                             [](const OSMRestriction& a, const OSMRestriction& b) {
                                               return a.from() < b.from();
                                             });
          OSMRestriction restriction{};
          while (res_it != complex_restrictions_from.end() &&
                 (restriction = *res_it).from() == edge_info.wayid()) {
            GraphId currentNode = directededge.endnode();

            std::vector<uint64_t> res_way_ids;
            res_way_ids.push_back(restriction.from());

            for (const auto& v : restriction.vias()) {
              res_way_ids.push_back(v);
            }

            // if via = restriction.to then don't add to the res_way_ids vector.  This happens
            // when we have a restriction:<type> with a via as a node in the osm data.
            if (restriction.vias().size() == 1 && restriction.vias().at(0) != restriction.to()) {
              res_way_ids.push_back(restriction.to());
            } else if (restriction.vias().size() > 1) {
              res_way_ids.push_back(restriction.to());
            }

            // walk in the forward direction.
            std::vector<GraphId> tmp_ids =
                GetGraphIds(currentNode, reader, lock, res_way_ids, restriction.modes(), true);

            // now that we have the tile and currentNode walk in the reverse direction as this is
            // really what needs to be stored in this tile.
            if (tmp_ids.size()) {
              std::reverse(res_way_ids.begin(), res_way_ids.end());
              auto tmp_ids =
                  GetGraphIds(currentNode, reader, lock, res_way_ids, restriction.modes(), false);

              auto AddReverseRestriction = [&](const std::vector<GraphId>& tmp_ids) {
                std::vector<GraphId> vias(tmp_ids.begin() + 1, tmp_ids.end() - 1);

                if (vias.size() > kMaxViasPerRestriction) {
                  LOG_WARN("Tried to exceed max vias per restriction(forward).  Way: " +
                           std::to_string(tmp_ids.at(0)));
                  return;
                }

                // flip the vias because we walk backwards from the search direction
                // using the predecessor edges in thor.
                std::reverse(vias.begin(), vias.end());
                GraphId from = tmp_ids.back();
                GraphId to = tmp_ids.front();

                if (restriction.type() >= RestrictionType::kOnlyRightTurn &&
                    restriction.type() <= RestrictionType::kOnlyStraightOn) {
                  if (to.Tile_Base() == tile_id) {
                    DirectedEdge& edge = tilebuilder.directededge_builder(to.id());
                    edge.complex_restriction(true);
                  } else {
                    stats.part_of_restriction.insert(to);
                  }
                }

                ComplexRestrictionBuilder complex_restriction =
                    CreateComplexRestriction(restriction, from, to, vias);

                // determine if we need to add this complex restriction or not.
                // basically we do not want any dups.
                bool bfound = false;
                const auto res = reverse_tmp_cr.equal_range(to);
                if (res.first != reverse_tmp_cr.end()) {
                  for (auto r = res.first; r != res.second; ++r) {
                    if (complex_restriction == r->second) {
                      bfound = true;
                      break;
                    }
                  }
                }
                if (!bfound) { // no dups.
                  reverse_tmp_cr.emplace(to, complex_restriction);
                  tilebuilder.AddReverseComplexRestriction(complex_restriction);
                  reverse_count++;
                }
              };

              if (tmp_ids.size() > 1 && tmp_ids.back().Tile_Base() == tile_id) {
                if (restriction.type() >= RestrictionType::kOnlyRightTurn &&
                    restriction.type() <= RestrictionType::kOnlyStraightOn) {
                  while (tmp_ids.size() > 1) {
                    auto last_edge_id = tmp_ids.front();
                    auto last_tile = tile;
                    if (last_tile->id() != last_edge_id.Tile_Base()) {
                      lock.lock();
                      last_tile = reader.GetGraphTile(last_edge_id);
                      lock.unlock();
                    }
                    auto last_de = last_tile->directededge(last_edge_id);
                    auto end_node = last_de->endnode();
                    auto end_node_tile = last_tile;
                    if (end_node_tile->id() != end_node.Tile_Base()) {
                      lock.lock();
                      end_node_tile = reader.GetGraphTile(end_node);
                      lock.unlock();
                    }

                    for (size_t i = 0; i < end_node_tile->node(end_node)->edge_count(); ++i) {
                      GraphId next_edge_id(end_node_tile->id().tileid(), end_node_tile->id().level(),
                                           end_node_tile->node(end_node)->edge_index() + i);
                      auto de = end_node_tile->directededge(next_edge_id);
                      auto opp_id = GetOpposingEdge(reader, lock, end_node_tile, end_node, de);
                      if (opp_id != last_edge_id && IsEdgeAllowed(de, restriction.modes(), true)) {
                        tmp_ids.front() = opp_id;
                        AddReverseRestriction(tmp_ids);
                      }
                    }

                    for (const auto& trans : end_node_tile->GetNodeTransitions(end_node)) {
                      auto to_node = trans.endnode();
                      lock.lock();
                      auto to_tile = reader.GetGraphTile(to_node);
                      lock.unlock();
                      auto to_node_info = to_tile->node(to_node);
                      GraphId next_edge_id(to_tile->id().tileid(), to_tile->id().level(),
                                           to_node_info->edge_index());
                      for (size_t i = 0; i < to_node_info->edge_count(); ++i, ++next_edge_id) {
                        auto de = to_tile->directededge(next_edge_id);
                        auto opp_id = GetOpposingEdge(reader, lock, to_tile, to_node, de);
                        if (opp_id != last_edge_id && IsEdgeAllowed(de, restriction.modes(), true)) {
                          tmp_ids.front() = opp_id;
                          AddReverseRestriction(tmp_ids);
                        }
                      }
                    }
                    tmp_ids.erase(tmp_ids.begin());
                  }
                } else {
                  AddReverseRestriction(tmp_ids);
                }
              }
            }
            ++res_it;
          }
        }

        if (directededge.end_restriction()) {
          OSMRestriction target_to_res{edge_info.wayid()}; // this is our from way id
          sequence<OSMRestriction>::iterator res_to_it =
              complex_restrictions_to.find(target_to_res,
                                           [](const OSMRestriction& a, const OSMRestriction& b) {
                                             return a.from() < b.from();
                                           });
          OSMRestriction restriction_to{};
          // is this edge the end of a restriction?
          while (res_to_it != complex_restrictions_to.end() &&
                 (restriction_to = *res_to_it).from() == edge_info.wayid()) {

            OSMRestriction target_res{restriction_to.to()}; // this is our from way id
            OSMRestriction restriction{};
            sequence<OSMRestriction>::iterator res_it =
                complex_restrictions_from.find(target_res,
                                               [](const OSMRestriction& a, const OSMRestriction& b) {
                                                 return a.from() < b.from();
                                               });
            while (res_it != complex_restrictions_from.end() &&
                   (restriction = *res_it).from() == restriction_to.to()) {

              GraphId currentNode = directededge.endnode();

              std::vector<uint64_t> res_way_ids;
              res_way_ids.push_back(restriction.to());

              std::vector<uint64_t> temp_vias = restriction.vias();
              std::reverse(temp_vias.begin(), temp_vias.end());

              // if via = restriction.to then don't add to the res_way_ids vector.  This
              // happens
              // when we have a restriction:<type> with a via as a node in the osm data.
              if (restriction.vias().size() > 1 ||
                  (restriction.vias().size() == 1 && restriction.vias().at(0) != restriction.to())) {
                for (const auto& v : temp_vias) {
                  res_way_ids.push_back(v);
                }
              }

              res_way_ids.push_back(restriction.from());

              // walk in the forward direction (reverse in relation to the restriction)
              std::vector<GraphId> tmp_ids =
                  GetGraphIds(currentNode, reader, lock, res_way_ids, restriction.modes(), false);

              // now that we have the tile and currentNode walk in the reverse
              // direction(forward in relation to the restriction) as this is really what
              // needs to be stored in this tile.
              if (tmp_ids.size()) {
                std::reverse(res_way_ids.begin(), res_way_ids.end());
                tmp_ids =
                    GetGraphIds(currentNode, reader, lock, res_way_ids, restriction.modes(), true);

                if (tmp_ids.size() > 1 && tmp_ids.back().Tile_Base() == tile_id) {
                  auto addForwardRestriction = [&](const std::vector<GraphId>& tmp_ids) {
                    std::vector<GraphId> vias(tmp_ids.begin() + 1, tmp_ids.end() - 1);

                    if (vias.size() > kMaxViasPerRestriction) {
                      LOG_WARN("Tried to exceed max vias per restriction(reverse).  Way: " +
                               std::to_string(tmp_ids.at(0)));
                      return;
                    }

                    std::reverse(vias.begin(), vias.end());
                    GraphId from = tmp_ids.front();
                    GraphId to = tmp_ids.back();
                    ComplexRestrictionBuilder complex_restriction =
                        CreateComplexRestriction(restriction, from, to, vias);

                    // determine if we need to add this complex restriction or not.
                    // basically we do not want any dups.
                    bool bfound = false;
                    const auto res = forward_tmp_cr.equal_range(from);
                    if (res.first != forward_tmp_cr.end()) {
                      for (auto r = res.first; r != res.second; ++r) {
                        if (complex_restriction == r->second) {
                          bfound = true;
                          break;
                        }
                      }
                    }
                    if (!bfound) { // no dups.
                      forward_tmp_cr.emplace(from, complex_restriction);

                      // happens if we got while processing only_* restriction
                      if (complex_restriction.to_graphid().Tile_Base() != tile_id) {
                        stats.restrictions.push_back(std::move(complex_restriction));
                      } else {
                        DirectedEdge& edge = tilebuilder.directededge_builder(to.id());
                        edge.set_end_restriction(edge.end_restriction() | restriction.modes());

                        tilebuilder.AddForwardComplexRestriction(complex_restriction);
                        forward_count++;
                      }
                    }
                  };

                  if (restriction.type() < RestrictionType::kOnlyRightTurn ||
                      restriction.type() > RestrictionType::kOnlyStraightOn) {
                    addForwardRestriction(tmp_ids);
                  } else {
                    while (tmp_ids.size() > 1) {
                      GraphId last_edge_id = *tmp_ids.rbegin();
                      GraphId pre_last_edge_id = *std::next(tmp_ids.rbegin());

                      auto pre_last_tile = tile;
                      if (pre_last_edge_id.Tile_Base() != pre_last_tile->id()) {
                        lock.lock();
                        pre_last_tile = reader.GetGraphTile(pre_last_edge_id);
                        lock.unlock();
                      }
                      auto pre_last_edge = pre_last_tile->directededge(pre_last_edge_id);

                      auto end_node = pre_last_edge->endnode();
                      auto next_tile = pre_last_tile;
                      if (end_node.Tile_Base() != next_tile->id()) {
                        lock.lock();
                        next_tile = reader.GetGraphTile(end_node);
                        lock.unlock();
                      }
                      auto node_info = next_tile->node(end_node);
                      GraphId edge_id(next_tile->id().tileid(), next_tile->id().level(),
                                      node_info->edge_index());
                      for (size_t i = 0; i < node_info->edge_count(); ++i, ++edge_id) {
                        auto de = next_tile->directededge(edge_id);
                        if (edge_id != last_edge_id && IsEdgeAllowed(de, restriction.modes(), true)) {
                          tmp_ids.back() = edge_id;
                          addForwardRestriction(tmp_ids);
                        }
                      }
                      for (const auto& trans : next_tile->GetNodeTransitions(node_info)) {
                        auto to_node = trans.endnode();
                        lock.lock();
                        auto to_tile = reader.GetGraphTile(to_node);
                        lock.unlock();
                        auto to_node_info = to_tile->node(to_node);
                        GraphId edge_id(to_tile->id().tileid(), to_tile->id().level(),
                                        to_node_info->edge_index());
                        for (size_t i = 0; i < to_node_info->edge_count(); ++i, ++edge_id) {
                          auto de = to_tile->directededge(edge_id);
                          if (edge_id != last_edge_id &&
                              IsEdgeAllowed(de, restriction.modes(), true)) {
                            tmp_ids.back() = edge_id;
                            addForwardRestriction(tmp_ids);
                          }
                        }
                      }
                      tmp_ids.pop_back();
                    }
                  }
                }
              }
              ++res_it;
            }
            res_to_it++;
          }
        }
      }
    }
    stats.forward_restrictions_count += forward_count;
    stats.reverse_restrictions_count += reverse_count;

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData();

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Trim();
    }
    lock.unlock();
  }

  // Send back the statistics
  result.set_value(stats);
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void RestrictionBuilder::Build(const boost::property_tree::ptree& pt,
                               const std::string& complex_from_restrictions_file,
                               const std::string& complex_to_restrictions_file) {

  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);
  for (auto tl = TileHierarchy::levels().rbegin(); tl != TileHierarchy::levels().rend(); ++tl) {
    // Create a randomized queue of tiles to work from
    std::deque<GraphId> tempqueue;
    auto level_tiles = reader.GetTileSet(tl->level);
    for (const auto& tile_id : level_tiles) {
      tempqueue.emplace_back(tile_id);
    }
    std::random_device rd;
    std::shuffle(tempqueue.begin(), tempqueue.end(), std::mt19937(rd()));
    std::queue<GraphId> tilequeue(tempqueue);

    // An atomic object we can use to do the synchronization
    std::mutex lock;
    // A place to hold worker threads and their results, exceptions or otherwise

    std::vector<std::shared_ptr<std::thread>> threads(
        std::max(static_cast<unsigned int>(1),
                 pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency())));
    // Hold the results (DataQuality/stats) for the threads
    std::vector<std::promise<Result>> promises(threads.size());

    // Start the threads
    LOG_INFO("Adding Restrictions at level " + std::to_string(tl->level));
    for (size_t i = 0; i < threads.size(); ++i) {
      threads[i].reset(new std::thread(build, std::cref(complex_from_restrictions_file),
                                       std::cref(complex_to_restrictions_file),
                                       std::cref(hierarchy_properties), std::ref(tilequeue),
                                       std::ref(lock), std::ref(promises[i])));
    }

    // Wait for them to finish up their work
    for (auto& thread : threads) {
      thread->join();
    }

    std::vector<Result> results;
    for (auto& p : promises) {
      // If something bad went down this will rethrow it
      try {
        results.push_back(p.get_future().get());
      } catch (const std::exception& e) {
        LOG_ERROR(e.what());
        throw e;
      }
    }

    HandleOnlyRestrictionProperties(results, hierarchy_properties);

    uint32_t forward_restrictions_count = 0;
    uint32_t reverse_restrictions_count = 0;

    for (const auto& stat : results) {
      forward_restrictions_count += stat.forward_restrictions_count + stat.restrictions.size();
      reverse_restrictions_count += stat.reverse_restrictions_count;
    }
    LOG_INFO("--Forward restrictions added: " + std::to_string(forward_restrictions_count));
    LOG_INFO("--Reverse restrictions added: " + std::to_string(reverse_restrictions_count));
  }
  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
