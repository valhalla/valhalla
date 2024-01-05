
#include "mjolnir/graphvalidator.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"

#include <boost/format.hpp>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
#include <numeric>
#include <ostream>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct HGVRestrictionTypes {
  bool hazmat;
  bool axle_load;
  bool height;
  bool length;
  bool weight;
  bool width;
};

// Get the GraphId of the opposing edge.
uint32_t GetOpposingEdgeIndex(const GraphId& startnode,
                              DirectedEdge& edge,
                              uint64_t wayid,
                              const graph_tile_ptr& tile,
                              const graph_tile_ptr& end_tile,
                              std::set<uint32_t>& problem_ways,
                              uint32_t& dupcount,
                              std::string& endnodeiso,
                              const uint32_t transit_level) {
  if (!end_tile) {
    LOG_WARN("End tile invalid.");
    return kMaxEdgesPerNode;
  }
  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  const NodeInfo* nodeinfo = end_tile->node(endnode.id());
  bool sametile = (startnode.tileid() == endnode.tileid());

  // The following can happen for transit nodes that do not connect to osm data
  // and have no transit lines.  This can happen when we are using a subset of
  // transit data.
  if (nodeinfo->edge_count() == 0) {
    LOG_DEBUG("End node has no connections " + std::to_string(endnode.tileid()) + "," +
              std::to_string(endnode.level()) + "," + std::to_string(endnode.id()));
    return kMaxEdgesPerNode;
  }

  // Set the end node iso.  Used for country crossings.
  endnodeiso = end_tile->admin(nodeinfo->admin_index())->country_iso();

  // Set the deadend flag on the edge.
  bool deadend = nodeinfo->intersection() == IntersectionType::kDeadEnd;
  edge.set_deadend(deadend);

  // Get the directed edges and return when the end node matches
  // the specified node and length / wayId, shape, use, and/or transit
  // attributes matches. Check for duplicates
  constexpr uint32_t absurd_index = 777777;
  uint32_t opp_index = absurd_index;
  const DirectedEdge* directededge = end_tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // Reject edge if access does not match or the edge does not point
    // back to the startnode
    if (directededge->endnode() != startnode ||
        edge.forwardaccess() != directededge->reverseaccess() ||
        edge.reverseaccess() != directededge->forwardaccess()) {
      continue;
    }

    // Transit connections. Match opposing edge if same way Id
    if (edge.use() == Use::kTransitConnection && directededge->use() == Use::kTransitConnection &&
        wayid == end_tile->edgeinfo(directededge).wayid()) {
      opp_index = i;
      continue;
    }
    if (edge.use() == Use::kTransitConnection || directededge->use() == Use::kTransitConnection) {
      continue;
    }
    if ((edge.use() == Use::kPlatformConnection && directededge->use() == Use::kPlatformConnection) ||
        (edge.use() == Use::kEgressConnection && directededge->use() == Use::kEgressConnection)) {
      auto shape1 = tile->edgeinfo(&edge).shape();
      auto shape2 = end_tile->edgeinfo(directededge).shape();
      if (shapes_match(shape1, shape2)) {
        opp_index = i;
        continue;
      }
    }

    // After this point should just have regular edges, shortcut edges, and transit lines.
    if (startnode.level() == transit_level) {
      // Transit level - handle transit lines
      if (edge.IsTransitLine() && directededge->IsTransitLine()) {
        // For a transit edge the line Id must match
        if (edge.lineid() == directededge->lineid()) {
          if (opp_index != absurd_index) {
            LOG_ERROR("Multiple transit edges have the same line Id = " +
                      std::to_string(edge.lineid()));
            dupcount++;
          }
          opp_index = i;
        }
      }
    } else {
      // Regular edges and shortcut edges remain. Lengths and shortcut
      // flag must match
      if (edge.length() != directededge->length() ||
          edge.is_shortcut() != directededge->is_shortcut()) {
        continue;
      }

      bool match = false;
      uint64_t wayid2 = 0;
      if (edge.is_shortcut()) {
        // Shortcut edges - use must match (or both are links)
        if ((directededge->link() && edge.link()) || (directededge->use() == edge.use())) {
          match = true;
        }
      } else {
        // Regular edges - match wayids and edge info offset (if in same tile)
        // or shape (if not in same tile)
        wayid2 = end_tile->edgeinfo(directededge).wayid();
        if (wayid == wayid2) {
          if (sametile && edge.edgeinfo_offset() == directededge->edgeinfo_offset()) {
            match = true;
          } else {
            auto shape1 = tile->edgeinfo(&edge).shape();
            auto shape2 = end_tile->edgeinfo(directededge).shape();
            if (shapes_match(shape1, shape2)) {
              match = true;
            }
          }
        }
      }

      // Set opposing index if match found
      if (match) {
        // Check if multiple edges match - log any duplicates
        if (opp_index != absurd_index && startnode.level() != transit_level) {
          if (edge.is_shortcut()) {
            std::vector<std::string> names = tile->edgeinfo(&edge).GetNames();
            std::string name = (names.size() > 0) ? names[0] : "unnamed";
            LOG_DEBUG("Duplicate shortcut for " + name +
                      " at LL = " + std::to_string(tile->get_node_ll(endnode).lat()) + "," +
                      std::to_string(tile->get_node_ll(endnode).lng()));
          } else {
            LOG_DEBUG("Potential duplicate: wayids " + std::to_string(wayid) + " and " +
                      std::to_string(wayid2) + " level = " + std::to_string(startnode.level()) +
                      " sametile = " + std::to_string(sametile));
            problem_ways.insert(wayid);
            problem_ways.insert(wayid2);
          }
          dupcount++;
        }

        // Set the internal intersection flag if matching opposing edge is
        // marked as an internal intersection edge
        if (directededge->internal()) {
          edge.set_internal(true);
        }
        opp_index = i;
      }
    }
  }

  // No matching opposing edge found - log error cases
  if (opp_index == absurd_index) {
    if (edge.use() == Use::kTransitConnection || edge.use() == Use::kEgressConnection ||
        edge.use() == Use::kPlatformConnection) {
      // Log error - no opposing edge for a transit connection
      LOG_ERROR("No opposing transit/egress/platform connection edge: endstop = " +
                std::to_string(nodeinfo->stop_index()) + " has " +
                std::to_string(nodeinfo->edge_count()));
    } else if (edge.IsTransitLine()) {
      // TODO - add this when opposing transit edges with unique line Ids
      // are present
      /*LOG_ERROR("No opposing transit edge: endstop = " +
               std::to_string(nodeinfo->stop_index()) + " has " +
               std::to_string(nodeinfo->edge_count())); */
    } else if (startnode.level() != transit_level) {
      PointLL ll = end_tile->get_node_ll(endnode);
      if (edge.is_shortcut()) {
        LOG_ERROR(
            (boost::format(
                 "No opposing shortcut edge at LL=%1%,%2% Length = %3% Startnode %4% EndNode %5%") %
             ll.lat() % ll.lng() % edge.length() % startnode % edge.endnode())
                .str());
      } else {
        LOG_ERROR((boost::format("No opposing edge at LL=%1%,%2% Length = %3% Startnode %4% "
                                 "EndNode %5% WayID %6% EdgeInfoOffset %7%") %
                   ll.lat() % ll.lng() % edge.length() % startnode % edge.endnode() % wayid %
                   edge.edgeinfo_offset())
                      .str());
      }

      uint32_t n = 0;
      directededge = end_tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
        if (edge.is_shortcut() == directededge->is_shortcut()) {
          LOG_WARN((boost::format("    Length = %1% Endnode: %2% WayId = %3% EdgeInfoOffset = %4%") %
                    directededge->length() % directededge->endnode() %
                    end_tile->edgeinfo(directededge).wayid() % directededge->edgeinfo_offset())
                       .str());
          n++;
        }
      }
      if (n == 0) {
        if (edge.is_shortcut()) {
          LOG_WARN("No Shortcut edges found from end node");
        } else {
          LOG_WARN("No regular edges found from end node");
        }
      }
    } else {
      LOG_ERROR("No match found - unhandled case");
    }
    return kMaxEdgesPerNode;
  }
  return opp_index;
}

using tweeners_t = GraphTileBuilder::tweeners_t;
void validate(
    const boost::property_tree::ptree& pt,
    std::deque<GraphId>& tilequeue,
    std::mutex& lock,
    std::promise<std::tuple<std::vector<uint32_t>, std::vector<std::vector<float>>, tweeners_t>>&
        result) {
  // Our local copy of edges binned to tiles that they pass through (dont start or end in)
  tweeners_t tweeners;
  // Local Graphreader
  GraphReader graph_reader(pt.get_child("mjolnir"));
  // Get some things we need throughout
  auto numLevels = TileHierarchy::levels().size() + 1; // To account for transit
  auto transit_level = TileHierarchy::GetTransitLevel().level;

  // vector to hold densities for each level
  std::vector<std::vector<float>> densities(numLevels);

  // Array to hold duplicates
  std::vector<uint32_t> duplicates(numLevels, 0);

  // Vector to hold problem ways
  std::set<uint32_t> problem_ways;

  // Check for more tiles
  while (true) {
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    // Get the next tile Id
    GraphId tile_id = tilequeue.front();
    tilequeue.pop_front();
    lock.unlock();

    // Point tiles to the set we need for current level
    const auto& tiles = tile_id.level() == TileHierarchy::GetTransitLevel().level
                            ? TileHierarchy::levels().back().tiles
                            : TileHierarchy::levels()[tile_id.level()].tiles;
    auto level = tile_id.level();
    auto tileid = tile_id.tileid();

    // Get the tile
    GraphTileBuilder tilebuilder(graph_reader.tile_dir(), tile_id, false);

    // Update nodes and directed edges as needed
    std::vector<NodeInfo> nodes;
    std::vector<DirectedEdge> directededges;

    // Get this tile
    lock.lock();
    graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);
    lock.unlock();

    // Iterate through the nodes and the directed edges
    uint32_t dupcount = 0;
    float roadlength = 0.0f;
    uint32_t nodecount = tilebuilder.header()->nodecount();
    GraphId node = tile_id;
    for (uint32_t i = 0; i < nodecount; i++, ++node) {
      // The node we will modify
      NodeInfo nodeinfo = tilebuilder.node(i);
      auto ni = tile->node(i);

      // Validate signs
      if (ni->named_intersection()) {
        if (tile->GetSigns(i, true).size() == 0) {
          LOG_ERROR("Node marked as having signs but none found");
        }
      }

      std::string begin_node_iso = tile->admin(nodeinfo.admin_index())->country_iso();

      // Go through directed edges and validate/update data
      uint32_t idx = ni->edge_index();
      GraphId edgeid(node.tileid(), node.level(), idx);
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++, idx++, ++edgeid) {
        auto de = tile->directededge(idx);

        // Validate signs
        if (de->sign()) {
          if (tile->GetSigns(idx).size() == 0) {
            LOG_ERROR("Directed edge marked as having signs but none found");
          }
        }

        // Validate lane connectivity
        if (de->laneconnectivity()) {
          if (tile->GetLaneConnectivity(idx).size() == 0) {
            LOG_ERROR(
                "Directed edge marked as having lane connectivity but none found ; tile level = " +
                std::to_string(tile_id.level()));
          }
        }

        // Validate access restrictions. TODO - should check modes as well
        uint32_t ar_modes = de->access_restriction();
        if (ar_modes) {
          // since only truck restrictions exist, we can still get all restrictions
          auto res = tile->GetAccessRestrictions(idx, kAllAccess);
          if (res.size() == 0) {
            LOG_ERROR(
                "Directed edge marked as having access restriction but none found ; tile level = " +
                std::to_string(tile_id.level()));
          } else {
            for (auto r : res) {
              if (r.edgeindex() != idx) {
                LOG_ERROR("Access restriction edge index does not match idx");
              }
            }
          }
        }

        // The edge we will modify
        DirectedEdge& directededge = tilebuilder.directededge(nodeinfo.edge_index() + j);

        // Road Length and some variables for statistics
        if (!directededge.shortcut()) {
          roadlength += directededge.length();
        }

        // Check if end node is in a different tile
        graph_tile_ptr endnode_tile = tile;
        if (tile_id != directededge.endnode().Tile_Base()) {
          directededge.set_leaves_tile(true);

          // Get the end node tile
          lock.lock();
          endnode_tile = graph_reader.GetGraphTile(directededge.endnode());
          lock.unlock();
          // make sure this is set to false as access tag logic could of set this to true.
        } else {
          directededge.set_leaves_tile(false);
        }

        // Set the opposing edge index and get the country ISO at the end
        // node. Set the deadend flag and internal flag (if the opposing
        // edge is internal then make sure this edge is as well)
        std::string end_node_iso;
        uint64_t wayid = tile->edgeinfo(&directededge).wayid();
        uint32_t opp_index =
            GetOpposingEdgeIndex(node, directededge, wayid, tile, endnode_tile, problem_ways,
                                 dupcount, end_node_iso, transit_level);
        directededge.set_opp_index(opp_index);
        if (directededge.use() == Use::kTransitConnection ||
            directededge.use() == Use::kEgressConnection ||
            directededge.use() == Use::kPlatformConnection || directededge.bss_connection()) {
          directededge.set_opp_local_idx(opp_index);
        }

        // Mark a country crossing if country ISO codes do not match
        if (!begin_node_iso.empty() && !end_node_iso.empty() && begin_node_iso != end_node_iso) {
          directededge.set_ctry_crossing(true);
        }

        // Validate the complex restriction settings. If no restrictions
        // are found that end at this directed edge, set the end restriction
        // modes to 0.
        if (de->end_restriction()) {
          uint32_t modes = 0;
          for (uint32_t mode = 1; mode < kAllAccess; mode *= 2) {
            if ((de->end_restriction() & mode) &&
                tile->GetRestrictions(true, edgeid, mode).size() > 0) {
              modes |= mode;
            }
          }
          directededge.set_end_restriction(modes);
        }
        if (de->start_restriction()) {
          uint32_t modes = 0;
          for (uint32_t mode = 1; mode < kAllAccess; mode *= 2) {
            if ((de->start_restriction() & mode) &&
                tile->GetRestrictions(false, edgeid, mode).size() > 0) {
              modes |= mode;
            }
          }
          directededge.set_start_restriction(modes);
        }

        // Add the directed edge to the local list
        directededges.emplace_back(std::move(directededge));
      }

      // Add the node to the list
      nodes.emplace_back(std::move(nodeinfo));
    }

    // Add density to return class. Approximate the tile area square km
    AABB2<PointLL> bb = tiles.TileBounds(tileid);
    float area = ((bb.maxy() - bb.miny()) * kMetersPerDegreeLat * kKmPerMeter) *
                 ((bb.maxx() - bb.minx()) *
                  DistanceApproximator<PointLL>::MetersPerLngDegree(bb.Center().y()) * kKmPerMeter);
    float density = (roadlength * 0.0005f) / area;
    densities[level].push_back(density);

    // Set the relative road density within this tile.
    uint32_t relative_density;
    if (tile_id.level() == 0) {
      relative_density = static_cast<uint32_t>(density * 100.0f);
    } else if (tile_id.level() == 1) {
      relative_density = static_cast<uint32_t>(density * 20.0f);
    } else {
      relative_density = static_cast<uint32_t>(density * 2.0f);
    }
    tilebuilder.header_builder().set_density(relative_density);

    // Bin the edges
    auto bins = GraphTileBuilder::BinEdges(tile, tweeners);

    // Write the new tile
    lock.lock();
    tilebuilder.Update(nodes, directededges);

    // Write the bins to it
    if (tile->header()->graphid().level() == TileHierarchy::levels().back().level) {
      auto reloaded = GraphTile::Create(graph_reader.tile_dir(), tile_id);
      GraphTileBuilder::AddBins(graph_reader.tile_dir(), reloaded, bins);
    }

    // Check if we need to clear the tile cache
    if (graph_reader.OverCommitted()) {
      graph_reader.Trim();
    }
    lock.unlock();

    // Add possible duplicates to return class
    duplicates[level] += dupcount;
  }

  // TODO - output problem ways - this could be a useful list!
  /*    for (auto w : problem_ways) {
        LOG_INFO("Problem Way: " + std::to_string(w));
      }*/

  // Fill promise with return data
  result.set_value(std::make_tuple(std::move(duplicates), std::move(densities), std::move(tweeners)));
}

// take tweeners from different tiles' perspectives and merge into a single tweener
// per tile that needs to update its bins
void merge(const tweeners_t& in, tweeners_t& out) {
  for (const auto& t : in) {
    // shove it in
    auto inserted = out.insert(t);
    // had this tile already
    if (!inserted.second) {
      // so have to merge
      for (size_t c = 0; c < kBinCount; ++c) {
        auto& bin = inserted.first->second[c];
        bin.insert(bin.end(), t.second[c].cbegin(), t.second[c].cend());
      }
    }
  }
}

// crack open tiles and bin edges that pass through them but dont end or begin in them
void bin_tweeners(const std::string& tile_dir,
                  tweeners_t::iterator& start,
                  const tweeners_t::iterator& end,
                  uint64_t dataset_id,
                  std::mutex& lock) {
  // go while we have tiles to update
  while (true) {
    lock.lock();
    if (start == end) {
      lock.unlock();
      break;
    }
    // grab this tile and its extra bin edges
    const auto& tile_bin = *start;
    ++start;
    lock.unlock();

    // some tiles are just there because edges' shapes passes through them (no edges/nodes, just bins)
    // if that's the case we need to make a tile to store the spatial index (binned edges) there
    auto tile = GraphTile::Create(tile_dir, tile_bin.first);
    if (!tile) {
      GraphTileBuilder empty(tile_dir, tile_bin.first, false);
      empty.header_builder().set_dataset_id(dataset_id);
      empty.StoreTileData();
      tile = GraphTile::Create(tile_dir, tile_bin.first);
    }

    // keep the extra binned edges
    GraphTileBuilder::AddBins(tile_dir, tile, tile_bin.second);
  }
}
} // namespace

namespace valhalla {
namespace mjolnir {

void GraphValidator::Validate(const boost::property_tree::ptree& pt) {
  LOG_INFO("Validating, finishing and binning tiles...");
  auto hierarchy_properties = pt.get_child("mjolnir");
  std::string tile_dir = hierarchy_properties.get<std::string>("tile_dir");

  // Create a randomized queue of tiles (at all levels) to work from
  std::deque<GraphId> tilequeue;
  GraphReader reader(pt.get_child("mjolnir"));
  auto tileset = reader.GetTileSet();
  for (const auto& id : tileset) {
    tilequeue.emplace_back(id);
  }
  // fixed seed for reproducible tile build
  std::shuffle(tilequeue.begin(), tilequeue.end(), std::mt19937(3));

  // Remember what the dataset id is in case we have to make some tiles
  graph_tile_ptr first_tile = GraphTile::Create(tile_dir, *tilequeue.begin());
  assert(tilequeue.size() && first_tile);
  auto dataset_id = first_tile->header()->dataset_id();

  // An mutex we can use to do the synchronization
  std::mutex lock;

  // Setup threads
  std::vector<std::shared_ptr<std::thread>> threads(
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency())));

  // Setup promises
  std::list<
      std::promise<std::tuple<std::vector<uint32_t>, std::vector<std::vector<float>>, tweeners_t>>>
      results;

  // Spawn the threads
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(validate, std::cref(pt), std::ref(tilequeue), std::ref(lock),
                                 std::ref(results.back())));
  }

  // Wait for threads to finish
  for (auto& thread : threads) {
    thread->join();
  }
  // Get the promise from the future
  std::vector<uint32_t> duplicates(TileHierarchy::levels().size(), 0);
  std::vector<std::vector<float>> densities(3);
  tweeners_t tweeners;
  for (auto& result : results) {
    auto data = result.get_future().get();
    // Total up duplicates for each level
    for (uint8_t i = 0; i < TileHierarchy::levels().size(); ++i) {
      duplicates[i] += std::get<0>(data)[i];
      for (auto& d : std::get<1>(data)[i]) {
        densities[i].push_back(d);
      }
    }
    // keep track of tweeners
    merge(std::get<2>(data), tweeners);
  }
  LOG_INFO("Finished");

  // run a pass to add the edges that binned to tweener tiles
  LOG_INFO("Binning inter-tile edges...");
  auto start = tweeners.begin();
  auto end = tweeners.end();
  for (auto& thread : threads) {
    thread.reset(new std::thread(bin_tweeners, std::cref(tile_dir), std::ref(start), std::cref(end),
                                 dataset_id, std::ref(lock)));
  }
  for (auto& thread : threads) {
    thread->join();
  }
  LOG_INFO("Finished");

  // print dupcount and find densities
  for (uint8_t level = 0; level < TileHierarchy::levels().size(); level++) {
    // Print duplicates info for level
    LOG_WARN((boost::format("Possible duplicates at level: %1% = %2%") % std::to_string(level) %
              duplicates[level])
                 .str());
    if (densities[level].empty()) {
      continue;
    }
#ifdef LOGGING_LEVEL_DEBUG
    // Get the average density and the max density
    float max_density = 0.0f;
    float sum = 0.0f;
    for (auto& density : densities[level]) {
      if (density > max_density) {
        max_density = density;
      }
      sum += density;
    }
    LOG_DEBUG("Average density = " + std::to_string(sum / densities[level].size()) +
              " max = " + std::to_string(max_density));
#endif
  }
}
} // namespace mjolnir
} // namespace valhalla
