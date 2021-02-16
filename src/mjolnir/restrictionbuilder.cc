#include "mjolnir/restrictionbuilder.h"
#include "mjolnir/complexrestrictionbuilder.h"
#include "mjolnir/dataquality.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmrestriction.h"

#include <future>
#include <queue>
#include <set>
#include <thread>

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

bool ExpandFromNode(GraphReader& reader,
                    std::mutex& lock,
                    GraphId& last_node,
                    std::unordered_set<GraphId>& visited_nodes,
                    std::vector<EdgeId>& edge_ids,
                    const std::vector<uint64_t>& way_ids,
                    size_t way_id_index,
                    const graph_tile_ptr& prev_tile,
                    GraphId prev_node,
                    GraphId current_node);

bool ExpandFromNodeInner(GraphReader& reader,
                         std::mutex& lock,
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

    if (de->endnode() != prev_node &&
        !(de->IsTransitLine() || de->is_shortcut() || de->use() == Use::kTransitConnection ||
          de->use() == Use::kEgressConnection || de->use() == Use::kPlatformConnection)) {
      auto edge_info = tile->edgeinfo(de->edgeinfo_offset());
      if (edge_info.wayid() == way_id) {
        edge_ids.push_back({way_id, edge_id});

        bool found;
        // expand with the next way_id
        found = ExpandFromNode(reader, lock, last_node, visited_nodes, edge_ids, way_ids,
                               way_id_index + 1, tile, current_node, de->endnode());
        if (found)
          return true;

        if (visited_nodes.find(de->endnode()) == visited_nodes.end()) {
          visited_nodes.insert(de->endnode());

          // expand with the same way_id
          found = ExpandFromNode(reader, lock, last_node, visited_nodes, edge_ids, way_ids,
                                 way_id_index, tile, current_node, de->endnode());
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
  found = ExpandFromNodeInner(reader, lock, last_node, visited_nodes, edge_ids, way_ids, way_id_index,
                              tile, prev_node, current_node, node_info);
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

    found = ExpandFromNodeInner(reader, lock, last_node, visited_nodes, edge_ids, way_ids,
                                way_id_index, trans_tile, prev_node, trans->endnode(),
                                trans_tile->node(trans->endnode()));
    if (found)
      return true;
  }

  return false;
}

std::vector<GraphId> GetGraphIds(GraphId& start_node,
                                 GraphReader& reader,
                                 std::mutex& lock,
                                 const std::vector<uint64_t>& way_ids) {
  lock.lock();
  graph_tile_ptr tile = reader.GetGraphTile(start_node);
  lock.unlock();

  std::unordered_set<GraphId> visited_nodes{start_node};
  std::vector<EdgeId> edge_ids;
  ExpandFromNode(reader, lock, start_node, visited_nodes, edge_ids, way_ids, 0, tile, GraphId(),
                 start_node);
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

void build(const std::string& complex_restriction_from_file,
           const std::string& complex_restriction_to_file,
           const boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue,
           std::mutex& lock,
           std::promise<DataQuality>& result) {
  sequence<OSMRestriction> complex_restrictions_from(complex_restriction_from_file, false);
  sequence<OSMRestriction> complex_restrictions_to(complex_restriction_to_file, false);

  GraphReader reader(hierarchy_properties);
  DataQuality stats;

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

    uint32_t id = tile_id.tileid();
    GraphId prevNode;
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      // Go through directed edges and "enhance" directed edge attributes
      const DirectedEdge* edges = tilebuilder.directededges(nodeinfo.edge_index());
      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge = tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        if (directededge.IsTransitLine() || directededge.is_shortcut() ||
            directededge.use() == Use::kTransitConnection ||
            directededge.use() == Use::kEgressConnection ||
            directededge.use() == Use::kPlatformConnection) {
          continue;
        }
        auto e_offset = tilebuilder.edgeinfo(directededge.edgeinfo_offset());
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

          OSMRestriction target_res{e_offset.wayid()}; // this is our from way id
          OSMRestriction restriction{};
          sequence<OSMRestriction>::iterator res_it =
              complex_restrictions_from.find(target_res,
                                             [](const OSMRestriction& a, const OSMRestriction& b) {
                                               return a.from() < b.from();
                                             });
          while (res_it != complex_restrictions_from.end() &&
                 (restriction = *res_it).from() == e_offset.wayid() && restriction.vias().size()) {

            if (restriction.type() < RestrictionType::kOnlyRightTurn ||
                restriction.type() > RestrictionType::kOnlyStraightOn) {

              GraphId currentNode = directededge.endnode();

              std::vector<uint64_t> res_way_ids;
              res_way_ids.push_back(e_offset.wayid());

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
              std::vector<GraphId> tmp_ids = GetGraphIds(currentNode, reader, lock, res_way_ids);

              // now that we have the tile and currentNode walk in the reverse direction as this is
              // really what needs to be stored in this tile.
              if (tmp_ids.size()) {

                res_way_ids.clear();
                res_way_ids.push_back(restriction.to());

                std::vector<uint64_t> temp_vias = restriction.vias();
                std::reverse(temp_vias.begin(), temp_vias.end());

                // if via = restriction.to then don't add to the res_way_ids vector.  This happens
                // when we have a restriction:<type> with a via as a node in the osm data.
                if (restriction.vias().size() == 1 && restriction.vias().at(0) != restriction.to()) {
                  for (const auto& v : temp_vias) {
                    res_way_ids.push_back(v);
                  }
                } else if (restriction.vias().size() > 1) {
                  for (const auto& v : temp_vias) {
                    res_way_ids.push_back(v);
                  }
                }

                res_way_ids.push_back(e_offset.wayid());
                tmp_ids = GetGraphIds(currentNode, reader, lock, res_way_ids);

                if (tmp_ids.size() && tmp_ids.back().Tile_Base() == tile_id) {

                  std::vector<GraphId> vias;
                  std::copy(tmp_ids.begin() + 1, tmp_ids.end() - 1, std::back_inserter(vias));

                  if (vias.size() > kMaxViasPerRestriction) {
                    LOG_WARN("Tried to exceed max vias per restriction(forward).  Way: " +
                             std::to_string(tmp_ids.at(0)));
                    res_it++;
                    continue;
                  }

                  // flip the vias because we walk backwards from the search direction
                  // using the predecessor edges in thor.
                  std::reverse(vias.begin(), vias.end());

                  ComplexRestrictionBuilder complex_restriction;
                  complex_restriction.set_from_id(tmp_ids.at(tmp_ids.size() - 1));
                  complex_restriction.set_via_list(vias);
                  complex_restriction.set_to_id(tmp_ids.at(0));
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

                  // determine if we need to add this complex restriction or not.
                  // basically we do not want any dups.
                  bool bfound = false;
                  const auto res = reverse_tmp_cr.equal_range(tmp_ids.at(0));
                  if (res.first != reverse_tmp_cr.end()) {
                    for (auto r = res.first; r != res.second; ++r) {
                      if (complex_restriction == r->second) {
                        bfound = true;
                        break;
                      }
                    }
                  }
                  if (!bfound) { // no dups.
                    reverse_tmp_cr.emplace(tmp_ids.at(0), complex_restriction);
                    tilebuilder.AddReverseComplexRestriction(complex_restriction);
                    reverse_count++;
                  }
                }
              }
            }
            res_it++;
          }
        }

        if (directededge.end_restriction()) {

          OSMRestriction target_to_res{e_offset.wayid()}; // this is our from way id
          OSMRestriction restriction_to{};
          sequence<OSMRestriction>::iterator res_to_it =
              complex_restrictions_to.find(target_to_res,
                                           [](const OSMRestriction& a, const OSMRestriction& b) {
                                             return a.from() < b.from();
                                           });
          // is this edge the end of a restriction?
          while (res_to_it != complex_restrictions_to.end() &&
                 (restriction_to = *res_to_it).from() == e_offset.wayid()) {

            OSMRestriction target_res{restriction_to.to()}; // this is our from way id
            OSMRestriction restriction{};
            sequence<OSMRestriction>::iterator res_it =
                complex_restrictions_from.find(target_res,
                                               [](const OSMRestriction& a, const OSMRestriction& b) {
                                                 return a.from() < b.from();
                                               });
            while (res_it != complex_restrictions_from.end() &&
                   (restriction = *res_it).from() == restriction_to.to() &&
                   restriction.vias().size()) {

              if (restriction.type() < RestrictionType::kOnlyRightTurn ||
                  restriction.type() > RestrictionType::kOnlyStraightOn) {

                GraphId currentNode = directededge.endnode();

                std::vector<uint64_t> res_way_ids;
                res_way_ids.push_back(restriction.to());

                std::vector<uint64_t> temp_vias = restriction.vias();
                std::reverse(temp_vias.begin(), temp_vias.end());

                // if via = restriction.to then don't add to the res_way_ids vector.  This
                // happens
                // when we have a restriction:<type> with a via as a node in the osm data.
                if (restriction.vias().size() == 1 && restriction.vias().at(0) != restriction.to()) {
                  for (const auto& v : temp_vias) {
                    res_way_ids.push_back(v);
                  }
                } else if (restriction.vias().size() > 1) {
                  for (const auto& v : temp_vias) {
                    res_way_ids.push_back(v);
                  }
                }

                res_way_ids.push_back(restriction_to.to());

                // walk in the forward direction (reverse in relation to the restriction)
                std::vector<GraphId> tmp_ids = GetGraphIds(currentNode, reader, lock, res_way_ids);

                // now that we have the tile and currentNode walk in the reverse
                // direction(forward in relation to the restriction) as this is really what
                // needs to be stored in this tile.
                if (tmp_ids.size()) {

                  res_way_ids.clear();
                  res_way_ids.push_back(restriction_to.to());

                  for (const auto& v : restriction.vias()) {
                    res_way_ids.push_back(v);
                  }

                  // if via = restriction.to then don't add to the res_way_ids vector.  This
                  // happens when we have a restriction:<type> with a via as a node in the osm
                  // data.
                  if (restriction.vias().size() == 1 &&
                      restriction.vias().at(0) != restriction.to()) {
                    res_way_ids.push_back(restriction.to());
                  } else if (restriction.vias().size() > 1) {
                    res_way_ids.push_back(restriction.to());
                  }

                  tmp_ids = GetGraphIds(currentNode, reader, lock, res_way_ids);

                  if (tmp_ids.size() && tmp_ids.back().Tile_Base() == tile_id) {
                    std::vector<GraphId> vias;
                    std::copy(tmp_ids.begin() + 1, tmp_ids.end() - 1, std::back_inserter(vias));

                    if (vias.size() > kMaxViasPerRestriction) {
                      LOG_WARN("Tried to exceed max vias per restriction(reverse).  Way: " +
                               std::to_string(tmp_ids.at(0)));
                      res_it++;
                      continue;
                    }

                    std::reverse(vias.begin(), vias.end());
                    ComplexRestrictionBuilder complex_restriction;
                    complex_restriction.set_from_id(tmp_ids.at(0));
                    complex_restriction.set_via_list(vias);
                    complex_restriction.set_to_id(tmp_ids.at(tmp_ids.size() - 1));
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

                    // determine if we need to add this complex restriction or not.
                    // basically we do not want any dups.
                    bool bfound = false;
                    const auto res = forward_tmp_cr.equal_range(tmp_ids.at(0));
                    if (res.first != forward_tmp_cr.end()) {
                      for (auto r = res.first; r != res.second; ++r) {
                        if (complex_restriction == r->second) {
                          bfound = true;
                          break;
                        }
                      }
                    }
                    if (!bfound) { // no dups.
                      forward_tmp_cr.emplace(tmp_ids.at(0), complex_restriction);
                      tilebuilder.AddForwardComplexRestriction(complex_restriction);
                      forward_count++;
                    }
                  }
                }
              }
              res_it++;
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
    std::vector<std::promise<DataQuality>> results(threads.size());

    // Start the threads
    LOG_INFO("Adding Restrictions at level " + std::to_string(tl->level));
    for (size_t i = 0; i < threads.size(); ++i) {
      threads[i].reset(new std::thread(build, std::cref(complex_from_restrictions_file),
                                       std::cref(complex_to_restrictions_file),
                                       std::cref(hierarchy_properties), std::ref(tilequeue),
                                       std::ref(lock), std::ref(results[i])));
    }

    // Wait for them to finish up their work
    for (auto& thread : threads) {
      thread->join();
    }

    uint32_t forward_restrictions_count = 0;
    uint32_t reverse_restrictions_count = 0;

    for (auto& result : results) {
      // If something bad went down this will rethrow it
      try {
        const auto& stat = result.get_future().get();
        forward_restrictions_count += stat.forward_restrictions_count;
        reverse_restrictions_count += stat.reverse_restrictions_count;
      } catch (const std::exception& e) {
        LOG_ERROR(e.what());
        throw e;
      }
    }
    LOG_INFO("--Forward restrictions added: " + std::to_string(forward_restrictions_count));
    LOG_INFO("--Reverse restrictions added: " + std::to_string(reverse_restrictions_count));
  }
  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
