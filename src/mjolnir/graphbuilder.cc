
#include "mjolnir/graphbuilder.h"
#include "mjolnir/util.h"
#include "mjolnir/node_expander.h"
#include "mjolnir/ferry_connections.h"
#include "mjolnir/linkclassification.h"

#include <future>
#include <utility>
#include <thread>
#include <set>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/graphreader.h>

#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

/**
 * we need the nodes to be sorted by graphid and then by osmid to make a set of tiles
 * we also need to then update the egdes that pointed to them
 *
 */
std::map<GraphId, size_t> SortGraph(const std::string& nodes_file,
                                    const std::string& edges_file,
                                    const TileHierarchy& tile_hierarchy,
                                    const uint8_t level) {
  LOG_INFO("Sorting graph...");

  // Sort nodes by graphid then by osmid, so its basically a set of tiles
  sequence<Node> nodes(nodes_file, false);
  nodes.sort(
    [&tile_hierarchy, &level](const Node& a, const Node& b) {
      if(a.graph_id == b.graph_id)
        return a.node.osmid < b.node.osmid;
      return a.graph_id < b.graph_id;
    }
  );
  //run through the sorted nodes, going back to the edges they reference and updating each edge
  //to point to the first (out of the duplicates) nodes index. at the end of this there will be
  //tons of nodes that no edges reference, but we need them because they are the means by which
  //we know what edges connect to a given node from the nodes perspective
  sequence<Edge> edges(edges_file, false);
  uint32_t run_index = 0;
  uint32_t node_index = 0;
  size_t node_count = 0;
  Node last_node{};
  std::map<GraphId, size_t> tiles;
  nodes.transform(
    [&nodes, &edges, &run_index, &node_index, &node_count, &last_node, &tiles](Node& node) {
      //remember if this was a new tile
      if(node_index == 0 || node.graph_id != (--tiles.end())->first) {
        tiles.insert({node.graph_id, node_index});
        node.graph_id.fields.id = 0;
        run_index = node_index;
        ++node_count;
      }//but is it a new node
      else if(last_node.node.osmid != node.node.osmid) {
        node.graph_id.fields.id = last_node.graph_id.fields.id + 1;
        run_index = node_index;
        ++node_count;
      }//not new keep the same graphid
      else
        node.graph_id.fields.id = last_node.graph_id.fields.id;

      //if this node marks the start of an edge, go tell the edge where the first node in the series is
      if(node.is_start()) {
        auto element = edges[node.start_of];
        auto edge = *element;
        edge.sourcenode_ = run_index;
        element = edge;
      }
      //if this node marks the end of an edge, go tell the edge where the first node in the series is
      if(node.is_end()) {
        auto element = edges[node.end_of];
        auto edge = *element;
        edge.targetnode_ = run_index;
        element = edge;
      }

      //next node
      last_node = node;
      ++node_index;
    }
  );

  LOG_INFO("Finished with " + std::to_string(node_count) + " graph nodes");
  return tiles;
}

// Construct edges in the graph and assign nodes to tiles.
void ConstructEdges(const OSMData& osmdata, const std::string& ways_file,
          const std::string& way_nodes_file,
          const std::string& nodes_file,
          const std::string& edges_file, const float tilesize,
          const std::function<GraphId (const OSMNode&)>& graph_id_predicate) {
  LOG_INFO("Creating graph edges from ways...")

  //so we can read ways and nodes and write edges
  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, true);
  sequence<Node> nodes(nodes_file, true);

  // Method to get length of an edge (used to find short link edges)
  const auto Length = [&way_nodes](const size_t idx1, const OSMNode& node2) {
    auto node1 = (*way_nodes[idx1]).node;
    PointLL a(node1.lng, node1.lat);
    PointLL b(node2.lng, node2.lat);
    return a.Distance(b);
  };

  // For each way traversed via the nodes
  uint32_t edgeindex = 0;
  GraphId graphid;
  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    // Grab the way and its first node
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index = first_way_node_index + way.node_count() - 1;

    // Remember this edge starts here
    Edge edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);

    // Remember this node as starting this edge
    way_node.node.attributes_.link_edge = way.link();
    way_node.node.attributes_.non_link_edge = !way.link();
    nodes.push_back({way_node.node, static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(-1), graph_id_predicate(way_node.node)});

    // Iterate through the nodes of the way until we find an intersection
    while(current_way_node_index < way_nodes.size()) {
      // Get the next shape point on this edge
      way_node = *way_nodes[++current_way_node_index];
      edge.attributes.llcount++;

      // If its an intersection or the end of the way it's a node of the road network graph
      if (way_node.node.intersection()) {

        // Finish off this edge
        edge.attributes.shortlink = (way.link() &&
                  Length(edge.llindex_, way_node.node) < kMaxInternalLength);
        way_node.node.attributes_.link_edge = way.link();
        way_node.node.attributes_.non_link_edge = !way.link();
        nodes.push_back({way_node.node, static_cast<uint32_t>(-1), static_cast<uint32_t>(edges.size()), graph_id_predicate(way_node.node)});
        edges.push_back(edge);

        // Start a new edge if this is not the last node in the way
        if (current_way_node_index != last_way_node_index) {
          edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);
          sequence<Node>::iterator element = --nodes.end();
          auto node = *element;
          node.start_of = edges.size();
          element = node;
        }// This was the last shape point in the way
        else {
          ++current_way_node_index;
          break;
        }
      }// If this edge has a signal not at a intersection
      else if (way_node.node.traffic_signal()) {
        edge.attributes.traffic_signal = true;
        edge.attributes.forward_signal = way_node.node.forward_signal();
        edge.attributes.backward_signal = way_node.node.backward_signal();
      }
    }
  }
  LOG_INFO("Finished with " + std::to_string(edges.size()) + " graph edges");
}

/*
struct DuplicateEdgeInfo {
  uint32_t edgeindex;
  uint32_t length;

  DuplicateEdgeInfo() : edgeindex(0), length(0) { }
  DuplicateEdgeInfo(const uint32_t idx, const uint32_t l)
      : edgeindex(idx),
        length(l) {
  }
};

void CheckForDuplicates(const GraphId& nodeid, const Node& node,
                const std::vector<uint32_t>& edgelengths,
                const std::unordered_map<GraphId, std::vector<Node>>& nodes,
                const std::vector<Edge>& edges,
                const std::vector<OSMWay>& ways, std::atomic<DataQuality*>& stats) {
  uint32_t edgeindex;
  GraphId endnode;
  std::unordered_map<GraphId, DuplicateEdgeInfo> endnodes;
  uint32_t n = 0;
  for (auto edgeindex : node.edges) {
    const Edge& edge = edges[edgeindex];
    if (edge.sourcenode_ == nodeid) {
      endnode = edge.targetnode_;
    } else {
      endnode = edge.sourcenode_;
    }

    // Check if the end node is already in the set of edges from this node
    const auto en = endnodes.find(endnode);
    if (en != endnodes.end() && en->second.length == edgelengths[n]) {
      uint64_t wayid1 = ways[edges[en->second.edgeindex].wayindex_].way_id();
      uint64_t wayid2 = ways[edges[edgeindex].wayindex_].way_id();
      (*stats).AddIssue(kDuplicateWays, GraphId(), wayid1, wayid2);
    } else {
      endnodes.emplace(std::piecewise_construct,
                       std::forward_as_tuple(endnode),
                       std::forward_as_tuple(edgeindex, edgelengths[n]));
    }
    n++;
  }
}
*/
uint32_t CreateSimpleTurnRestriction(const uint64_t wayid, const size_t endnode,
    sequence<Node>& nodes, sequence<Edge>& edges, const OSMData& osmdata,
    sequence<OSMWay>& ways, DataQuality& stats) {

  auto res = osmdata.restrictions.equal_range(wayid);
  if (res.first == osmdata.restrictions.end()) {
    return 0;
  }

  // Edge is the from edge of a restriction. Find all TRs (if any)
  // through the target (end) node of this directed edge.
  auto node_itr = nodes[endnode];
  auto node = *node_itr;
  std::vector<OSMRestriction> trs;
  for (auto r = res.first; r != res.second; ++r) {
    if (r->second.via() == node.node.osmid) {
      if (r->second.day_on() != DOW::kNone) {
        stats.timedrestrictions++;
      } else {
        trs.push_back(r->second);
      }
    }
  }
  if (trs.empty()) {
    return 0;
  }

  // Get the way Ids of the edges at the endnode
  std::vector<uint64_t> wayids;
  auto bundle = collect_node_edges(node_itr, nodes, edges);
  for (const auto& edge : bundle.node_edges) {
    wayids.push_back((*ways[edge.first.wayindex_]).osmwayid_);
  }

  // There are some cases where both ONLY and NO restriction types are
  // present. Allow this. Iterate through all restrictions and set the
  // restriction mask to include the indexes of restricted turns.
  uint32_t mask = 0;
  for (const auto& tr : trs) {
    switch (tr.type()) {
    case RestrictionType::kNoLeftTurn:
    case RestrictionType::kNoRightTurn:
    case RestrictionType::kNoStraightOn:
    case RestrictionType::kNoUTurn:
      // Iterate through the edge wayIds until the matching to way Id is found
      for (uint32_t idx = 0, n = wayids.size(); idx < n; idx++) {
        if (wayids[idx] == tr.to()) {
          mask |= (1 << idx);
          break;
        }
      }
      break;

    case RestrictionType::kOnlyRightTurn:
    case RestrictionType::kOnlyLeftTurn:
    case RestrictionType::kOnlyStraightOn:
      // Iterate through the edge wayIds - any non-matching edge is added
      // to the turn restriction
      for (uint32_t idx = 0, n = wayids.size(); idx < n; idx++) {
        if (wayids[idx] != tr.to()) {
          mask |= (1 << idx);
        }
      }
      break;
    }
  }

  // Return the restriction mask
  return mask;
}

// Walk the shape and look for any empty tiles that the shape intersects
void CheckForIntersectingTiles(const GraphId& tile1, const GraphId& tile2,
                const Tiles<PointLL>& tiling, std::vector<PointLL>& shape,
                DataQuality& stats) {
  // Walk the shape segments until we are outside
  uint32_t current_tile = tile1.tileid();
  auto shape1 = shape.begin();
  auto shape2 = shape1 + 1;
  while (shape2 < shape.end()) {
    uint32_t next_tile = tiling.TileId(shape2->lat(), shape2->lng());
    if (next_tile != current_tile) {
      // If a neighbor we can just add this tile
      if (tiling.AreNeighbors(current_tile, next_tile)) {
        stats.AddIntersectedTile(GraphId(next_tile, 2, 0));
      } else {
        // Not a neighbor - find any intermediate intersecting tiles

        // Get a bounding box or row, col surrounding the 2 tiles
        auto rc1 = tiling.GetRowColumn(current_tile);
        auto rc2 = tiling.GetRowColumn(next_tile);
        for (int32_t row = std::min(rc1.first, rc2.first);
             row <= std::max(rc1.first, rc2.first); row++) {
          for (int32_t col = std::min(rc1.second, rc2.second);
                       col <= std::max(rc1.second, rc2.second); col++) {
            // Get the tile Id for the row,col. Skip if either of the 2 tiles.
            int32_t tileid = tiling.TileId(row, col);
            GraphId id(tileid, 2, 0);
            if (tileid == current_tile || tileid == next_tile) {
              continue;
            }

            // Check if the shape segment intersects the tile
            if (tiling.TileBounds(tileid).Intersect(*shape1, *shape2)) {
              stats.AddIntersectedTile(id);
            }
          }
        }
      }
    }

    // Increment
    shape1 = shape2;
    shape2++;
  }
}

void BuildTileSet(const std::string& ways_file, const std::string& way_nodes_file,
    const std::string& nodes_file, const std::string& edges_file,
    const TileHierarchy& hierarchy, const OSMData& osmdata,
    std::map<GraphId, size_t>::const_iterator tile_start,
    std::map<GraphId, size_t>::const_iterator tile_end,
    std::promise<DataQuality>& result) {

  std::string thread_id = static_cast<const std::ostringstream&>(std::ostringstream() << std::this_thread::get_id()).str();
  LOG_INFO("Thread " + thread_id + " started");

  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);

  const auto& tl = hierarchy.levels().rbegin();
  Tiles<PointLL> tiling = tl->second.tiles;

  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::vector<PointLL> shape;
    shape.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng, node.lat);
    }
    return shape;
  };

  // For each tile in the task
  bool added = false;
  DataQuality stats;

  ////////////////////////////////////////////////////////////////////////////
  // Iterate over tiles
  for(; tile_start != tile_end; ++tile_start) {
    try {
      // What actually writes the tile
      GraphTileBuilder graphtile;
      GraphId tileid1 = tile_start->first.Tile_Base();

      // Iterate through the nodes
      uint32_t idx = 0;                 // Current directed edge index
      uint32_t directededgecount = 0;

      ////////////////////////////////////////////////////////////////////////
      // Iterate over nodes in the tile
      auto node_itr = nodes[tile_start->second];
      while (node_itr != nodes.end() && (*node_itr).graph_id.Tile_Base() == tileid1) {
        //amalgamate all the node duplicates into one and the edges that connect to it
        //this moves the iterator for you
        auto bundle = collect_node_edges(node_itr, nodes, edges);
        const auto& node = bundle.node;
        PointLL node_ll{node.lng, node.lat};

        // Look for potential duplicates
        //CheckForDuplicates(nodeid, node, edgelengths, nodes, edges, osmdata.ways, stats);

        // it is a fork if more than two edges and more than one driveforward edge and
        //   if all the edges are links
        //   OR the node is a motorway_junction and none of the edges are links
        bool fork = (((bundle.node_edges.size() > 2) && (bundle.driveforward_count > 1))
            && ((bundle.link_count == bundle.node_edges.size())
                || ((node.type() == NodeType::kMotorWayJunction)
                    && (bundle.link_count == 0))));

        //////////////////////////////////////////////////////////////////////
        // Iterate over edges at node
        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        uint32_t n = 0;
        RoadClass bestclass = RoadClass::kServiceOther;
        std::vector<DirectedEdgeBuilder> directededges;
        for (const auto& edge_pair : bundle.node_edges) {
          // Get the edge and way
          const Edge& edge = edge_pair.first;
          const OSMWay w = *ways[edge.wayindex_];

          // Get the shape for the edge and compute its length
          auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
          uint32_t length = static_cast<uint32_t>(PointLL::Length(shape) + .5f);

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          bool forward = edge.sourcenode_ == node_itr.position();
          size_t source = edge.sourcenode_, target = edge.targetnode_;
          if (!forward)
            std::swap(source, target);

          // Validate speed
          uint32_t speed = static_cast<uint32_t>(w.speed());
          if (speed > kMaxSpeedKph) {
            LOG_WARN("Speed = " + std::to_string(speed) + " wayId= " +
                       std::to_string(w.way_id()));
            speed = kMaxSpeedKph;
          }

          // Infer cul-de-sac if a road edge is a loop and is low
          // classification. TODO - do we need length limit?
          Use use = w.use();
          RoadClass rc = static_cast<RoadClass>(edge.attributes.importance);
          if (use == Use::kRoad && source == target &&
              rc > RoadClass::kTertiary) {
            use = Use::kCuldesac;
            stats.culdesaccount++;
          }

          // Handle simple turn restrictions that originate from this
          // directed edge
          uint32_t restrictions = CreateSimpleTurnRestriction(w.way_id(),
            target, nodes, edges, osmdata, ways, stats);
          if (restrictions != 0)
            stats.simplerestrictions++;

          // traffic signal exists at an intersection node
          // OR
          // traffic signal exists at a non-intersection node
          // forward signal must exist if forward direction and vice versa.
          // if forward and backward signal flags are not set then only set for oneways.
          bool has_signal = (!forward && node.traffic_signal()) ||
            ((edge.attributes.traffic_signal) &&
            ((forward && edge.attributes.forward_signal) || (!forward && edge.attributes.backward_signal) ||
            (w.oneway() && !edge.attributes.forward_signal && !edge.attributes.backward_signal)));

          auto bike = osmdata.bike_relations.equal_range(w.way_id());
          uint32_t bike_network = 0;

          //TODO:  Do we want to call out the bike network refs and names for bicycle routing?
          //uint32_t national_ref_index = 0, regional_ref_index = 0, local_ref_index = 0;
          //uint32_t mtb_national_ref_index = 0, mtb_regional_ref_index = 0, mtb_local_ref_index = 0;

          for (auto b = bike.first; b != bike.second; ++b) {
            uint32_t network = b->second.bike_network;

            // a mountain bike network has sub networks (lcn, ncn, or rcn)
            if (network & kMcn) {

              //TODO:  Do we want to call out the bike network refs and names for bicycle routing?
              /*if (network & kNcn) //ncn
                mtb_national_ref_index = b->second.ref_index;
              else if (network & kRcn) //rcn
                mtb_regional_ref_index = b->second.ref_index;
              else if (network & kLcn) //lcn
                mtb_local_ref_index = b->second.ref_index;
              */

              bike_network |= kMcn;

            } else {

              //TODO:  Do we want to call out the bike network refs and names for bicycle routing?
              /*if (network & kNcn) //ncn
                national_ref_index = b->second.ref_index;
              else if (network & kRcn) //rcn
                regional_ref_index = b->second.ref_index;
              else if (network & kLcn) //lcn
                local_ref_index = b->second.ref_index;
              */

              bike_network |= network;
            }
          }

          if ((bike_network & kMcn) || (w.bike_network() & kMcn))
            use = Use::kMountainBike;

          // Add a directed edge and get a reference to it
          directededges.emplace_back(w, (*nodes[target]).graph_id, forward, length,
                        speed, use, rc, n, has_signal, restrictions, bike_network);
          DirectedEdgeBuilder& directededge = directededges.back();

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.classification());

          // Check for updated ref from relations.
          std::string ref;
          auto iter = osmdata.way_ref.find(w.way_id());
          if (iter != osmdata.way_ref.end()) {
            if (w.ref_index() != 0)
              ref = GraphBuilder::GetRef(osmdata.ref_offset_map.name(w.ref_index()),iter->second);
          }

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(
              edge_pair.second, (*nodes[source]).graph_id,
              (*nodes[target]).graph_id, w.way_id(), shape,
              w.GetNames(ref, osmdata.ref_offset_map, osmdata.name_offset_map),
              added);
          directededge.set_edgeinfo_offset(edge_info_offset);

          // TODO - update logic so we limit the CreateExitSignInfoList calls
          // Any exits for this directed edge? is auto and oneway?
          std::vector<SignInfo> exits = GraphBuilder::CreateExitSignInfoList(
              node, w, osmdata, fork);

          // Add signs if signs exist
          // and directed edge if forward access and auto use
          // and directed edge is a link and not (link count=2 and driveforward count=1)
          //    OR node is a fork
          if (!exits.empty() && (directededge.forwardaccess() & kAutoAccess)
              && ((directededge.link()
                  && (!((bundle.link_count == 2)
                      && (bundle.driveforward_count == 1)))) || fork)) {
            graphtile.AddSigns(idx, exits);
            directededge.set_exitsign(true);
          }

          //TODO: If this was a loop edge we need its twin because this node wont be encountered again
          /*if(source == target) {
            idx++;
            n++;
            auto flipped = directededge.flipped();
            flipped.set_localedgeidx(n);
            directededges.emplace_back();
          }*/

          // If the end node is in a different tile and the tile is not
          // a neighboring tile then check for possible shape intersection with
          // empty tiles
          GraphId tileid2 = (*nodes[target]).graph_id.Tile_Base();
          if (tileid1 != tileid2 && !tiling.AreNeighbors(tileid1, tileid2)) {
            CheckForIntersectingTiles(tileid1, tileid2, tiling,
                     shape, stats);
          }

          // Increment the directed edge index within the tile
          idx++;
          n++;
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        NodeInfoBuilder nodebuilder(node_ll, bestclass, node.access_mask(),
                                    node.type(), (n == 1), node.traffic_signal());
        if (fork) {
          nodebuilder.set_intersection(IntersectionType::kFork);
        }

        directededgecount += n;

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);

        // Increment the counts in the histogram
        stats.nodecount++;
        stats.directededge_count += directededges.size();
        stats.node_counts[directededges.size()]++;

        // Next node in the tile
        node_itr += bundle.node_count;
      }

      // Write the actual tile to disk
      graphtile.StoreTileData(hierarchy, tile_start->first);

      // Made a tile
      LOG_INFO((boost::format("Thread %1% wrote tile %2%: %3% bytes") % thread_id % tile_start->first % graphtile.size()).str());
    }// Whatever happens in Vegas..
    catch(std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      LOG_ERROR((boost::format("Thread %1% failed tile %2%: %3%") % thread_id % tile_start->first % e.what()).str());
      return;
    }
  }
  // Let the main thread see how this thread faired
  result.set_value(stats);
}

// Build tiles for the local graph hierarchy
void BuildLocalTiles(const unsigned int thread_count, const OSMData& osmdata,
  const std::string& ways_file, const std::string& way_nodes_file,
  const std::string& nodes_file, const std::string& edges_file,
  const std::map<GraphId, size_t>& tiles, const TileHierarchy& tile_hierarchy, DataQuality& stats) {

  LOG_INFO("Building " + std::to_string(tiles.size()) + " tiles with " + std::to_string(thread_count) + " threads...");

  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);

  // Hold the results (DataQuality/stats) for the threads
  std::vector<std::promise<DataQuality> > results(threads.size());

  // Divvy up the work
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::map<GraphId, size_t>::const_iterator tile_start, tile_end = tiles.begin();

  // Atomically pass around stats info
  LOG_INFO(std::to_string(tiles.size()) + " tiles");
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(
      new std::thread(BuildTileSet,  std::cref(ways_file), std::cref(way_nodes_file),
                      std::cref(nodes_file), std::cref(edges_file), std::cref(tile_hierarchy),
                      std::cref(osmdata), tile_start, tile_end,
                      std::ref(results[i]))
    );
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");

  // Check all of the outcomes and accumulate stats
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      // Add statistics and log issues on this thread
      const auto& stat = result.get_future().get();
      stats.AddStatistics(stat);
      stat.LogIssues();
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  // Add "empty" tiles for any intersected tiles
  for (auto& empty_tile : stats.intersected_tiles) {
    if (!GraphReader::DoesTileExist(tile_hierarchy, empty_tile)) {
      GraphTileBuilder graphtile;
      LOG_INFO("Add empty tile for: " + std::to_string(empty_tile.tileid()));
      graphtile.StoreTileData(tile_hierarchy, empty_tile);
    }
  }
  LOG_INFO("Added " + std::to_string(stats.intersected_tiles.size()) +
           " empty, intersected tiles");
}

}

namespace valhalla {
namespace mjolnir {

// Build the graph from the input
void GraphBuilder::Build(const boost::property_tree::ptree& pt, const OSMData& osmdata,
    const std::string& ways_file, const std::string& way_nodes_file) {
  std::string nodes_file = "nodes.bin";
  std::string edges_file = "edges.bin";
  TileHierarchy tile_hierarchy(pt.get_child("hierarchy"));
  unsigned int threads = std::max(static_cast<unsigned int>(1),
                                  pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));
  const auto& tl = tile_hierarchy.levels().rbegin();
  uint8_t level = tl->second.level;

  // Make the edges and nodes in the graph
  ConstructEdges(osmdata, ways_file, way_nodes_file, nodes_file, edges_file, tl->second.tiles.TileSize(),
    [&tile_hierarchy, &level](const OSMNode& node) {
      return tile_hierarchy.GetGraphId({node.lng, node.lat}, level);
    }
  );

  // Line up the nodes and then re-map the edges that the edges to them
  auto tiles = SortGraph(nodes_file, edges_file, tile_hierarchy, level);

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  DataQuality stats;
  ReclassifyLinks(ways_file, nodes_file, edges_file, stats);

  // Reclassify ferry connection edges - use the highway classification cutoff
  RoadClass rc = RoadClass::kPrimary;
  for (auto& level : tile_hierarchy.levels()) {
    if (level.second.name == "highway") {
      rc = level.second.importance;
    }
  }
  ReclassifyFerryConnections(ways_file, way_nodes_file, nodes_file, edges_file,
                             static_cast<uint32_t>(rc), stats);

  // Build tiles at the local level. Form connected graph from nodes and edges.
  BuildLocalTiles(threads, osmdata, ways_file, way_nodes_file, nodes_file,
                  edges_file, tiles, tile_hierarchy, stats);

  stats.LogStatistics();
}


// Get highway refs from relations
std::string GraphBuilder::GetRef(const std::string& way_ref, const std::string& relation_ref) {
  bool found = false;
  std::string refs;
  std::vector<std::string> way_refs = GetTagTokens(way_ref); // US 51;I 57
  std::vector<std::string> refdirs = GetTagTokens(relation_ref);// US 51|north;I 57|north
  for (auto& ref : way_refs) {
    found = false;
    for (const auto& refdir : refdirs) {
      std::vector<std::string> tmp = GetTagTokens(refdir,'|'); // US 51|north
      if (tmp.size() == 2) {
        if (tmp[0] == ref) { // US 51 == US 51
          if (!refs.empty())
            refs += ";" + ref + " " + tmp[1];// ref order of the way wins.
          else
            refs = ref + " " + tmp[1];
          found = true;
          break;
        }
      }
    }

    if (!found) {   // no direction found in relations for this ref
      if (!refs.empty())
        refs += ";" + ref;
      else
        refs = ref;
    }
  }
  return refs;
}

std::vector<SignInfo> GraphBuilder::CreateExitSignInfoList(
    const OSMNode& node, const OSMWay& way, const OSMData& osmdata, bool fork) {

  std::vector<SignInfo> exit_list;

  ////////////////////////////////////////////////////////////////////////////
  // NUMBER

  // Exit sign number
  if (way.junction_ref_index() != 0) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.ref_offset_map.name(way.junction_ref_index()));
  }  else if (node.ref() && !fork) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.node_ref.find(node.osmid)->second);
  }

  ////////////////////////////////////////////////////////////////////////////
  // BRANCH

  bool has_branch = false;

  // Exit sign branch refs
  if (way.destination_ref_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_index()));
    for (auto& branch_ref : branch_refs) {
      exit_list.emplace_back(Sign::Type::kExitBranch, branch_ref);
    }
  }

  // Exit sign branch road names
  if (way.destination_street_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_streets = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_street_index()));
    for (auto& branch_street : branch_streets) {
      exit_list.emplace_back(Sign::Type::kExitBranch, branch_street);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // TOWARD

  bool has_toward = false;

  // Exit sign toward refs
  if (way.destination_ref_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_to_index()));
    for (auto& toward_ref : toward_refs) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_ref);
    }
  }

  // Exit sign toward streets
  if (way.destination_street_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_streets = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_street_to_index()));
    for (auto& toward_street : toward_streets) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_street);
    }
  }

  // Exit sign toward locations
  if (way.destination_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_names = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_index()));
    for (auto& toward_name : toward_names) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_name);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // Process exit_to only if other branch or toward info does not exist
  if (!has_branch && !has_toward) {
    if (node.exit_to() && !fork) {

      std::string tmp;
      std::size_t pos;
      std::vector<std::string> exit_tos = GetTagTokens(
          osmdata.node_exit_to.find(node.osmid)->second);
      for (auto& exit_to : exit_tos) {

        tmp = exit_to;

        boost::algorithm::to_lower(tmp);

        //remove the "To" For example:  US 11;To I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "to ")) {
            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(3));
            continue;
        }
        //remove the "Toward" For example:  US 11;Toward I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "toward ")) {
            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(7));
            continue;
        }

        std::size_t found = tmp.find(" to ");

        //Default to kToward if found twice or "toward" found as well; otherwise, <branch> to <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
           (tmp.find(" to ",found+4) == std::string::npos && tmp.find(" toward ") == std::string::npos)) {

            exit_list.emplace_back(Sign::Type::kExitBranch, exit_to.substr(0,found));

            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(found+4));
            continue;
        }

        found = tmp.find(" toward ");

        //Default to kToward if found twice or "to" found as well; otherwise, <branch> toward <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
            (tmp.find(" toward ",found+8) == std::string::npos && tmp.find(" to ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, exit_to.substr(0,found));

          exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(found+8));
          continue;
        }

        //default to toward.
        exit_list.emplace_back(Sign::Type::kExitToward, exit_to);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // NAME

  // Exit sign name
  if (node.name() && !fork) {
    std::vector<std::string> names = GetTagTokens(
            osmdata.node_name.find(node.osmid)->second);
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kExitName, name);
    }
  }

  return exit_list;
}

}
}
