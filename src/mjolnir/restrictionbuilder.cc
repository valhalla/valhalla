#include "mjolnir/restrictionbuilder.h"
#include "mjolnir/complexrestrictionbuilder.h"
#include "mjolnir/dataquality.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmrestriction.h"

#include <future>
#include <queue>
#include <set>
#include <thread>

#include <boost/filesystem/operations.hpp>

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

// Function to replace wayids with graphids.  Will transition up and down the hierarchy as needed.
std::deque<GraphId> GetGraphIds(GraphId& n_graphId,
                                GraphReader& reader,
                                GraphId& tileid,
                                std::mutex& lock,
                                std::vector<uint64_t> res_way_ids) {

  std::deque<GraphId> graphids;

  lock.lock();
  const GraphTile* endnodetile = reader.GetGraphTile(n_graphId);
  lock.unlock();

  const NodeInfo* n_info = endnodetile->node(n_graphId);
  bool bBeginFound = false;
  uint64_t begin_wayid = 0;
  uint64_t end_wayid = 0;

  std::set<GraphId> visited_set;

  GraphId prev_Node, avoidId;
  GraphId currentNode = n_graphId;

  for (uint32_t i = 1; i < res_way_ids.size(); i++) {

    begin_wayid = res_way_ids.at(i - 1);
    end_wayid = res_way_ids.at(i);

    uint32_t j = 0;
    while (j < n_info->edge_count()) {

      const DirectedEdge* de = endnodetile->directededge(n_info->edge_index() + j);

      GraphId g_id(endnodetile->id().tileid(), endnodetile->id().level(), n_info->edge_index() + j);

      if (de->edgeinfo_offset() != 0 && de->endnode() != prev_Node && g_id != avoidId &&
          !(de->IsTransitLine() || de->is_shortcut() || de->use() == Use::kTransitConnection ||
            de->use() == Use::kEgressConnection || de->use() == Use::kPlatformConnection)) {
        // get the edge info offset
        auto current_offset = endnodetile->edgeinfo(de->edgeinfo_offset());
        if (end_wayid == current_offset.wayid()) {
          // only save the last graphid as the others are not needed.
          if (i == 1 && graphids.size() > 1) {
            std::deque<GraphId> tmp(graphids.end() - 1, graphids.end());
            graphids = tmp;
          }

          prev_Node = currentNode;
          graphids.push_back(g_id);

          currentNode = de->endnode();
          // get the new tile if needed.
          if (endnodetile->id() != currentNode.Tile_Base()) {
            lock.lock();
            endnodetile = reader.GetGraphTile(currentNode);
            lock.unlock();
          }

          // get new end node and start over.
          n_info = endnodetile->node(currentNode);
          break;
        } else if (begin_wayid == current_offset.wayid()) {
          if (visited_set.find(currentNode) == visited_set.end()) {
            prev_Node = currentNode;
            visited_set.emplace(currentNode);
            graphids.push_back(g_id);

            currentNode = de->endnode();
            // get the new tile if needed.
            if (endnodetile->id() != currentNode.Tile_Base()) {
              lock.lock();
              endnodetile = reader.GetGraphTile(currentNode);
              lock.unlock();
            }

            // get new end node and start over.
            n_info = endnodetile->node(currentNode);
            j = 0;
            bBeginFound = true;
            continue;
          }
        }
      }

      // if we made it here we need to check transition edges.
      if (j + 1 == n_info->edge_count()) {
        if (n_info->transition_count() > 0) {
          bool bfound = false;
          uint32_t k = 0;
          while (k < n_info->transition_count()) {
            // only look at transition edges from this end node
            const GraphTile* temp_endnodetile = endnodetile;
            // Handle transitions - expand from the end node each transition
            const NodeTransition* trans = endnodetile->transition(n_info->transition_index() + k);

            if (temp_endnodetile->id() != trans->endnode().Tile_Base()) {
              lock.lock();
              temp_endnodetile = reader.GetGraphTile(trans->endnode());
              lock.unlock();
            }

            currentNode = trans->endnode();
            const NodeInfo* tmp_n_info = temp_endnodetile->node(currentNode);

            // look for the end way id.
            uint32_t l = 0;
            while (l < tmp_n_info->edge_count()) {

              const DirectedEdge* de = temp_endnodetile->directededge(tmp_n_info->edge_index() + l);

              GraphId g_id(temp_endnodetile->id().tileid(), temp_endnodetile->id().level(),
                           tmp_n_info->edge_index() + l);

              // only look at non transition edges.
              if (de->edgeinfo_offset() != 0 && de->endnode() != prev_Node && g_id != avoidId &&
                  !(de->IsTransitLine() || de->is_shortcut() ||
                    de->use() == Use::kTransitConnection || de->use() == Use::kEgressConnection ||
                    de->use() == Use::kPlatformConnection)) {
                auto current_offset = temp_endnodetile->edgeinfo(de->edgeinfo_offset());

                if (end_wayid == current_offset.wayid()) {
                  // only save the last graphid as the others are not needed.
                  if (i == 1 && graphids.size() > 1) {
                    std::deque<GraphId> tmp(graphids.end() - 1, graphids.end());
                    graphids = tmp;
                  }

                  prev_Node = currentNode;
                  graphids.push_back(g_id);

                  currentNode = de->endnode();
                  endnodetile = temp_endnodetile;

                  // get the new tile if needed.
                  if (endnodetile->id() != currentNode.Tile_Base()) {
                    lock.lock();
                    endnodetile = reader.GetGraphTile(currentNode);
                    lock.unlock();
                  }

                  // get new end node and start over.
                  n_info = endnodetile->node(currentNode);
                  bfound = true;
                  break; // while (l < tmp_n_info->edge_count())
                }
              }
              l++;
            }

            if (bfound) {
              break; // while (k < n_info->transition_count())
            }
            k++;
          }
          if (!bfound) { // bad restriction or on another level
            if (bBeginFound && !avoidId.Is_Valid()) {
              avoidId = graphids.at(0);
              graphids.clear();
              bBeginFound = false;
              visited_set.clear();
              i = 0; // start over avoiding the first graphid
                     //(i.e., we walked the graph in the wrong direction)
              lock.lock();
              endnodetile = reader.GetGraphTile(n_graphId);
              lock.unlock();
              n_info = endnodetile->node(n_graphId);
              currentNode = n_graphId;
              prev_Node = GraphId();
            } else {
              graphids.clear();
              return graphids;
            }
          }
          break; // while (j < n_info->edge_count())
        } else if (n_info->transition_count() == 0) {
          // We need to make sure we did not walk in the wrong direction along a huge wayid that has
          // no transitions.
          if (bBeginFound && !avoidId.Is_Valid()) {
            avoidId = graphids.at(0);
            graphids.clear();
            bBeginFound = false;
            visited_set.clear();
            i = 0; // start over avoiding the first graphid
                   //(i.e., we walked the graph in the wrong direction)
            lock.lock();
            endnodetile = reader.GetGraphTile(n_graphId);
            lock.unlock();
            n_info = endnodetile->node(n_graphId);
            currentNode = n_graphId;
            prev_Node = GraphId();
          } else {
            graphids.clear();
            return graphids;
          }
          break; // while (j < n_info->edge_count())
        }
      } // if (j + 1 == n_info->edge_count())
      j++;
    }
  }

  if (!bBeginFound) { // happens when opp edge is found and via a this node.
    graphids.clear();
  }

  n_graphId = currentNode;
  tileid = endnodetile->id();
  return graphids;
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

    // Get a readable tile.If the tile is empty, skip it. Empty tiles are
    // added where ways go through a tile but no end not is within the tile.
    // This allows creation of connectivity maps using the tile set,
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    if (tile->header()->nodecount() == 0) {
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

              GraphId currentNode = GraphId(tile->id().tileid(), tile->id().level(), i);
              GraphId tileid = tile->id();

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
              std::deque<GraphId> tmp_ids =
                  GetGraphIds(currentNode, reader, tileid, lock, res_way_ids);

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
                tmp_ids = GetGraphIds(currentNode, reader, tileid, lock, res_way_ids);

                if (tmp_ids.size()) {

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
                  complex_restriction.set_via_count(vias.size());
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

                GraphId currentNode = GraphId(tile->id().tileid(), tile->id().level(), i);
                GraphId tileid = tile->id();

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
                std::deque<GraphId> tmp_ids =
                    GetGraphIds(currentNode, reader, tileid, lock, res_way_ids);

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

                  tmp_ids = GetGraphIds(currentNode, reader, tileid, lock, res_way_ids);

                  if (tmp_ids.size()) {
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
                    complex_restriction.set_via_count(vias.size());
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

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void RestrictionBuilder::Build(const boost::property_tree::ptree& pt,
                               const std::string& complex_from_restrictions_file,
                               const std::string& complex_to_restrictions_file) {

  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);
  auto level = TileHierarchy::levels().rbegin();
  for (; level != TileHierarchy::levels().rend(); ++level) {
    // Create a randomized queue of tiles to work from
    auto tile_level = level->second;
    std::deque<GraphId> tempqueue;
    auto level_tiles = reader.GetTileSet(tile_level.level);
    for (const auto& tile_id : level_tiles) {
      tempqueue.emplace_back(tile_id);
    }
    std::random_shuffle(tempqueue.begin(), tempqueue.end());
    std::queue<GraphId> tilequeue(tempqueue);

    // An atomic object we can use to do the synchronization
    std::mutex lock;
    // A place to hold worker threads and their results, exceptions or otherwise

    std::vector<std::shared_ptr<std::thread>> threads(
        std::max(static_cast<unsigned int>(1),
                 pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency())));
    // Hold the results (DataQuality/stats) for the threads
    std::vector<std::promise<DataQuality>> results(threads.size());

    // Start the threads
    LOG_INFO("Adding Restrictions at level " + std::to_string(tile_level.level));
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
