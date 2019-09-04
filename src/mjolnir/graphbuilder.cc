#include "mjolnir/graphbuilder.h"
#include "mjolnir/admin.h"
#include "mjolnir/ferry_connections.h"
#include "mjolnir/linkclassification.h"
#include "mjolnir/node_expander.h"
#include "mjolnir/util.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include <future>
#include <set>
#include <thread>
#include <utility>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/signinfo.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/tiles.h"
#include "midgard/util.h"

#include "mjolnir/edgeinfobuilder.h"
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

/**
 * we need the nodes to be sorted by graphid and then by osmid to make a set of tiles
 * we also need to then update the edges that pointed to them
 *
 */
std::map<GraphId, size_t>
SortGraph(const std::string& nodes_file, const std::string& edges_file, const uint8_t level) {
  LOG_INFO("Sorting graph...");

  // Sort nodes by graphid then by osmid, so its basically a set of tiles
  sequence<Node> nodes(nodes_file, false);
  nodes.sort([](const Node& a, const Node& b) {
    if (a.graph_id == b.graph_id) {
      return a.node.osmid_ < b.node.osmid_;
    }
    return a.graph_id < b.graph_id;
  });
  // run through the sorted nodes, going back to the edges they reference and updating each edge
  // to point to the first (out of the duplicates) nodes index. at the end of this there will be
  // tons of nodes that no edges reference, but we need them because they are the means by which
  // we know what edges connect to a given node from the nodes perspective
  sequence<Edge> edges(edges_file, false);
  uint32_t run_index = 0;
  uint32_t node_index = 0;
  size_t node_count = 0;
  Node last_node{};
  std::map<GraphId, size_t> tiles;
  nodes.transform(
      [&nodes, &edges, &run_index, &node_index, &node_count, &last_node, &tiles](Node& node) {
        // remember if this was a new tile
        if (node_index == 0 || node.graph_id != (--tiles.end())->first) {
          tiles.insert({node.graph_id, node_index});
          node.graph_id.set_id(0);
          run_index = node_index;
          ++node_count;
        } // but is it a new node
        else if (last_node.node.osmid_ != node.node.osmid_) {
          node.graph_id.set_id(last_node.graph_id.id() + 1);
          run_index = node_index;
          ++node_count;
        } // not new keep the same graphid
        else {
          node.graph_id.set_id(last_node.graph_id.id());
        }

        // if this node marks the start of an edge, go tell the edge where the first node in the
        // series is
        if (node.is_start()) {
          auto element = edges[node.start_of];
          auto edge = *element;
          edge.sourcenode_ = run_index;
          element = edge;
        }
        // if this node marks the end of an edge, go tell the edge where the first node in the
        // series is
        if (node.is_end()) {
          auto element = edges[node.end_of];
          auto edge = *element;
          edge.targetnode_ = run_index;
          element = edge;
        }

        // next node
        last_node = node;
        ++node_index;
      });

  LOG_INFO("Finished with " + std::to_string(node_count) + " graph nodes");
  return tiles;
}

// Construct edges in the graph and assign nodes to tiles.
void ConstructEdges(const OSMData& osmdata,
                    const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const float tilesize,
                    const std::function<GraphId(const OSMNode&)>& graph_id_predicate) {
  LOG_INFO("Creating graph edges from ways...");

  // so we can read ways and nodes and write edges
  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, true);
  sequence<Node> nodes(nodes_file, true);

  // Method to get length of an edge (used to find short link edges)
  const auto Length = [&way_nodes](const size_t idx1, const OSMNode& node2) {
    auto node1 = (*way_nodes[idx1]).node;
    PointLL a(node1.lng_, node1.lat_);
    PointLL b(node2.lng_, node2.lat_);
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

    // Validate - make sure all nodes for this edge are valid
    bool valid = true;
    for (auto ni = current_way_node_index; ni <= last_way_node_index; ni++) {
      const auto wn = (*way_nodes[ni]).node;
      if (wn.lat_ == 0.0 && wn.lng_ == 0.0) {
        LOG_ERROR("Cannot find node " + std::to_string(wn.osmid_) + " in way " +
                  std::to_string(way.way_id()));
        valid = false;
      }
    }
    if (!valid) {
      LOG_ERROR("Do not add edge!");
      current_way_node_index = last_way_node_index + 1;
      continue;
    }

    // Remember this edge starts here
    Edge prev_edge = Edge{0};
    Edge edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);
    edge.attributes.way_begin = true;

    // Remember this node as starting this edge
    way_node.node.link_edge_ = way.link();
    way_node.node.non_link_edge_ = !way.link() && (way.auto_forward() || way.auto_backward());
    nodes.push_back({way_node.node, static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(-1),
                     graph_id_predicate(way_node.node)});

    // Iterate through the nodes of the way until we find an intersection
    while (current_way_node_index < way_nodes.size()) {
      // Get the next shape point on this edge
      way_node = *way_nodes[++current_way_node_index];
      edge.attributes.llcount++;

      // If its an intersection or the end of the way it's a node of the road network graph
      if (way_node.node.intersection()) {

        // Finish off this edge
        edge.attributes.shortlink =
            (way.link() && Length(edge.llindex_, way_node.node) < kMaxInternalLength);
        way_node.node.link_edge_ = way.link();
        way_node.node.non_link_edge_ = !way.link() && (way.auto_forward() || way.auto_backward());

        uint32_t size = static_cast<uint32_t>(edges.size());
        if (!edge.attributes.way_begin)
          size += 1;
        nodes.push_back(
            {way_node.node, static_cast<uint32_t>(-1), size, graph_id_predicate(way_node.node)});

        // Mark the edge as ending a way if this is the last node in the way
        edge.attributes.way_end = current_way_node_index == last_way_node_index;

        // Mark the previous edge as the prior one since we are processing the last edge
        if (edge.attributes.way_end) {
          prev_edge.attributes.way_prior = true;
        }

        if (!edge.attributes.way_begin)
          edges.push_back(prev_edge);

        // Mark the current edge as the next edge one since we processed the first edge
        if (prev_edge.attributes.way_begin)
          edge.attributes.way_next = true;

        prev_edge = edge;

        // Start a new edge if this is not the last node in the way
        if (current_way_node_index != last_way_node_index) {
          edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);
          sequence<Node>::iterator element = --nodes.end();
          auto node = *element;
          node.start_of = edges.size() + 1; // + 1 because the edge has not been added yet
          element = node;
        } // This was the last shape point in the way
        else {
          edges.push_back(prev_edge); // add the last edge
          ++current_way_node_index;
          break;
        }
      } // If this edge has a signal not at a intersection
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
      uint32_t wayid1 = ways[edges[en->second.edgeindex].wayindex_].way_id();
      uint32_t wayid2 = ways[edges[edgeindex].wayindex_].way_id();
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
uint32_t CreateSimpleTurnRestriction(const uint32_t wayid,
                                     const size_t endnode,
                                     sequence<Node>& nodes,
                                     sequence<Edge>& edges,
                                     const OSMData& osmdata,
                                     sequence<OSMWay>& ways,
                                     DataQuality& stats) {

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
    if (r->second.via() == node.node.osmid_) {
      trs.push_back(r->second);
    }
  }

  if (trs.empty()) {
    return 0;
  }

  // Get the way Ids of the edges at the endnode
  std::vector<uint32_t> wayids;
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
      case RestrictionType::kNoEntry:
      case RestrictionType::kNoExit:
      case RestrictionType::kNoTurn:
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

  // Check if mask exceeds the limit
  if (mask >= (1 << kMaxTurnRestrictionEdges)) {
    LOG_WARN("Restrictions mask exceeds allowable limit on wayid: " + std::to_string(wayid));
  }

  // Return the restriction mask
  return mask;
}

// Add an access restriction. Returns the mode(s) that have access
// restrictions on this edge.
uint32_t AddAccessRestrictions(const uint32_t edgeid,
                               const uint32_t wayid,
                               const OSMData& osmdata,
                               GraphTileBuilder& graphtile) {
  auto res = osmdata.access_restrictions.equal_range(wayid);
  if (res.first == osmdata.access_restrictions.end()) {
    return 0;
  }

  uint32_t modes = 0;
  for (auto r = res.first; r != res.second; ++r) {
    AccessRestriction access_restriction(edgeid, r->second.type(), r->second.modes(),
                                         r->second.value());
    graphtile.AddAccessRestriction(access_restriction);
    modes |= r->second.modes();
  }
  return modes;
}

void BuildTileSet(const std::string& ways_file,
                  const std::string& way_nodes_file,
                  const std::string& nodes_file,
                  const std::string& edges_file,
                  const std::string& complex_restriction_from_file,
                  const std::string& complex_restriction_to_file,
                  const std::string& tile_dir,
                  const OSMData& osmdata,
                  std::map<GraphId, size_t>::const_iterator tile_start,
                  std::map<GraphId, size_t>::const_iterator tile_end,
                  const uint32_t tile_creation_date,
                  const boost::property_tree::ptree& pt,
                  std::promise<DataQuality>& result) {

  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);
  sequence<OSMRestriction> complex_restrictions_from(complex_restriction_from_file, false);
  sequence<OSMRestriction> complex_restrictions_to(complex_restriction_to_file, false);

  auto database = pt.get_optional<std::string>("admin");
  // Initialize the admin DB (if it exists)
  sqlite3* admin_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database) {
    LOG_WARN("Admin db not found.  Not saving admin information.");
  } else if (!admin_db_handle) {
    LOG_WARN("Admin db " + *database + " not found.  Not saving admin information.");
  }

  database = pt.get_optional<std::string>("timezone");
  // Initialize the tz DB (if it exists)
  sqlite3* tz_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database) {
    LOG_WARN("Time zone db not found.  Not saving time zone information.");
  } else if (!tz_db_handle) {
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information.");
  }

  const auto& tl = TileHierarchy::levels().rbegin();
  Tiles<PointLL> tiling = tl->second.tiles;

  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::list<PointLL> shape;
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng_, node.lat_);
    }
    return shape;
  };

  // For each tile in the task
  bool added = false;
  DataQuality stats;

  // Lots of times in a given tile we may end up accessing the same
  // shape/attributes twice we avoid doing this by caching it here
  std::unordered_map<uint32_t, std::pair<float, uint32_t>> geo_attribute_cache;

  ////////////////////////////////////////////////////////////////////////////
  // Iterate over tiles
  for (; tile_start != tile_end; ++tile_start) {
    try {
      // What actually writes the tile
      GraphId tile_id = tile_start->first.Tile_Base();
      GraphTileBuilder graphtile(tile_dir, tile_id, false);

      // Information about tile creation
      graphtile.AddTileCreationDate(tile_creation_date);
      graphtile.header_builder().set_dataset_id(osmdata.max_changeset_id_);

      // Set the base lat,lon of the tile
      uint32_t id = tile_id.tileid();
      PointLL base_ll = tiling.Base(id);
      graphtile.header_builder().set_base_ll(base_ll);

      // Get the admin polygons. If only one exists for the tile check if the
      // tile is entirely inside the polygon
      bool tile_within_one_admin = false;
      std::unordered_multimap<uint32_t, multi_polygon_type> admin_polys;
      std::unordered_map<uint32_t, bool> drive_on_right;
      if (admin_db_handle) {
        admin_polys = GetAdminInfo(admin_db_handle, drive_on_right, tiling.TileBounds(id), graphtile);
        if (admin_polys.size() == 1) {
          // TODO - check if tile bounding box is entirely inside the polygon...
          tile_within_one_admin = true;
        }
      }

      bool tile_within_one_tz = false;
      std::unordered_multimap<uint32_t, multi_polygon_type> tz_polys;
      if (tz_db_handle) {
        tz_polys = GetTimeZones(tz_db_handle, tiling.TileBounds(id));
        if (tz_polys.size() == 1) {
          tile_within_one_tz = true;
        }
      }

      // Iterate through the nodes
      uint32_t idx = 0; // Current directed edge index

      ////////////////////////////////////////////////////////////////////////
      // Iterate over nodes in the tile
      auto node_itr = nodes[tile_start->second];
      // to avoid realloc we guess how many edges there might be in a given tile
      geo_attribute_cache.clear();
      geo_attribute_cache.reserve(5 * (std::next(tile_start) == tile_end
                                           ? nodes.end() - node_itr
                                           : std::next(tile_start)->second - tile_start->second));

      while (node_itr != nodes.end() && (*node_itr).graph_id.Tile_Base() == tile_id) {
        // amalgamate all the node duplicates into one and the edges that connect to it
        // this moves the iterator for you
        auto bundle = collect_node_edges(node_itr, nodes, edges);

        // Make sure node has edges
        if (bundle.node_edges.size() == 0) {
          LOG_ERROR("Node has no edges - skip");
          continue;
        }

        const auto& node = bundle.node;
        PointLL node_ll{node.lng_, node.lat_};

        // Get the admin index
        uint32_t admin_index = (tile_within_one_admin)
                                   ? admin_polys.begin()->first
                                   : GetMultiPolyId(admin_polys, node_ll, graphtile);

        // Look for potential duplicates
        // CheckForDuplicates(nodeid, node, edgelengths, nodes, edges, osmdata.ways, stats);

        // it is a fork if more than two edges and more than one driveforward edge and
        //   if all the edges are links
        //   OR the node is a motorway_junction
        //      AND none of the edges are links
        //      OR all outbound edges are links and there is only one inbound edge
        bool fork =
            (((bundle.node_edges.size() > 2) && (bundle.driveforward_count > 1)) &&
             ((bundle.link_count == bundle.node_edges.size()) ||
              ((node.type() == NodeType::kMotorWayJunction) &&
               ((bundle.link_count == 0) || ((bundle.link_count == bundle.driveforward_count) &&
                                             (bundle.node_edges.size() == bundle.link_count + 1))))));

        //////////////////////////////////////////////////////////////////////
        // Iterate over edges at node
        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        uint32_t n = 0;
        RoadClass bestclass = RoadClass::kServiceOther;
        for (const auto& edge_pair : bundle.node_edges) {
          // Get the edge and way
          const Edge& edge = edge_pair.first;
          const OSMWay w = *ways[edge.wayindex_];

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          bool forward = edge.sourcenode_ == node_itr.position();
          size_t source = edge.sourcenode_, target = edge.targetnode_;
          if (!forward) {
            std::swap(source, target);
          }

          // Validate speed. Set speed limit and truck speed.
          uint32_t speed = w.speed();
          if (forward && w.forward_tagged_speed()) {
            speed = w.forward_speed();
          } else if (!forward && w.backward_tagged_speed()) {
            speed = w.backward_speed();
          }
          if (speed > kMaxSpeedKph) {
            LOG_WARN("Speed = " + std::to_string(speed) + " wayId= " + std::to_string(w.way_id()));
            speed = kMaxSpeedKph;
          }
          uint32_t speed_limit = w.speed_limit();
          if (speed_limit > kMaxSpeedKph) {
            LOG_WARN("Speed limit = " + std::to_string(speed_limit) +
                     " wayId= " + std::to_string(w.way_id()));
            speed_limit = kMaxSpeedKph;
          }
          uint32_t truck_speed = w.truck_speed();
          if (truck_speed > kMaxSpeedKph) {
            LOG_WARN("Truck Speed = " + std::to_string(truck_speed) +
                     " wayId= " + std::to_string(w.way_id()));
            truck_speed = kMaxSpeedKph;
          }

          // Cul du sac
          auto use = w.use();
          if (use == Use::kCuldesac) {
            stats.culdesaccount++;
          }

          // Handle simple turn restrictions that originate from this
          // directed edge
          uint32_t restrictions =
              CreateSimpleTurnRestriction(w.way_id(), target, nodes, edges, osmdata, ways, stats);
          if (restrictions != 0) {
            stats.simplerestrictions++;
          }

          // traffic signal exists at an intersection node
          // OR
          // traffic signal exists at a non-intersection node
          // forward signal must exist if forward direction and vice versa.
          // if forward and backward signal flags are not set then only set for oneways.
          bool has_signal =
              (!forward && node.traffic_signal()) ||
              ((edge.attributes.traffic_signal) &&
               ((forward && edge.attributes.forward_signal) ||
                (!forward && edge.attributes.backward_signal) ||
                (w.oneway() && !edge.attributes.forward_signal && !edge.attributes.backward_signal)));

          auto bike = osmdata.bike_relations.equal_range(w.way_id());
          uint32_t bike_network = 0;

          // TODO:  Do we want to call out the bike network refs and names for bicycle routing?
          // uint32_t national_ref_index = 0, regional_ref_index = 0, local_ref_index = 0;
          // uint32_t mtb_national_ref_index = 0, mtb_regional_ref_index = 0, mtb_local_ref_index =
          // 0;

          for (auto b = bike.first; b != bike.second; ++b) {
            uint32_t network = b->second.bike_network;

            // a mountain bike network has sub networks (lcn, ncn, or rcn)
            if (network & kMcn) {

              // TODO:  Do we want to call out the bike network refs and names for bicycle routing?
              /*if (network & kNcn) //ncn
                mtb_national_ref_index = b->second.ref_index;
              else if (network & kRcn) //rcn
                mtb_regional_ref_index = b->second.ref_index;
              else if (network & kLcn) //lcn
                mtb_local_ref_index = b->second.ref_index;
              */

              bike_network |= kMcn;

            } else {

              // TODO:  Do we want to call out the bike network refs and names for bicycle routing?
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

          // Check if refs occur in both directions for this way. If so, a separate EdgeInfo needs to
          // be stored. This usually indicates a single carriageway with different directional
          // indicators.
          bool dual_refs = false;
          std::string ref;
          if (w.ref_index() != 0) {
            auto iter = osmdata.way_ref.find(w.way_id());
            auto iter_rev = osmdata.way_ref_rev.find(w.way_id());
            dual_refs = iter != osmdata.way_ref.end() && iter_rev != osmdata.way_ref_rev.end();

            // Check for updated ref from relations. If dual refs and reverse direction use the
            // reverse ref, otherwise use the forward ref.
            if (dual_refs && !forward) {
              if (iter_rev != osmdata.way_ref_rev.end()) {
                // Replace the ref with the reverse ref
                ref = GraphBuilder::GetRef(osmdata.name_offset_map.name(w.ref_index()),
                                           osmdata.name_offset_map.name(iter_rev->second));
              }
            } else {
              if (iter != osmdata.way_ref.end()) {
                ref = GraphBuilder::GetRef(osmdata.name_offset_map.name(w.ref_index()),
                                           osmdata.name_offset_map.name(iter->second));
              }
            }
          }

          // Get the shape for the edge and compute its length
          uint32_t edge_info_offset;
          auto found = geo_attribute_cache.cend();
          if (dual_refs || !graphtile.HasEdgeInfo(edge_pair.second, (*nodes[source]).graph_id,
                                                  (*nodes[target]).graph_id, edge_info_offset)) {

            // add the info
            auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);

            uint16_t types = 0;
            auto names = w.GetNames(ref, osmdata.name_offset_map, types);

            // Update bike_network type
            if (bike_network) {
              bike_network |= w.bike_network();
            } else {
              bike_network = w.bike_network();
            }

            // Add edge info. Mean elevation is set to 1234 as a placeholder, set later if we have it.
            edge_info_offset =
                graphtile.AddEdgeInfo(edge_pair.second, (*nodes[source]).graph_id,
                                      (*nodes[target]).graph_id, w.way_id(), 1234, bike_network,
                                      speed_limit, shape, names, types, added, dual_refs);
            if (added) {
              stats.edgeinfocount++;
            }

            // length
            auto length = valhalla::midgard::length(shape);

            // Compute a curvature metric [0-15]. TODO - use resampled polyline?
            uint32_t curvature = compute_curvature(shape);

            // Add the curvature to the cache
            auto inserted = geo_attribute_cache.insert({edge_info_offset, {length, curvature}});
            found = inserted.first;

          } // now we have the edge info offset
          else {
            found = geo_attribute_cache.find(edge_info_offset);
          }

          // this can't happen
          if (found == geo_attribute_cache.cend()) {
            throw std::runtime_error("GeoAttributes cached object should be there!");
          }

          // ferry speed override.  duration is set on the way
          if (w.ferry() && w.duration()) {
            // convert to kph
            uint32_t spd = static_cast<uint32_t>((std::get<0>(found->second) * 3.6f) / w.duration());
            speed = (spd == 0) ? 1 : spd;
          }

          // Add a directed edge and get a reference to it
          DirectedEdgeBuilder de(w, (*nodes[target]).graph_id, forward,
                                 static_cast<uint32_t>(std::get<0>(found->second) + .5), speed,
                                 truck_speed, use, static_cast<RoadClass>(edge.attributes.importance),
                                 n, has_signal, restrictions, bike_network,
                                 edge.attributes.reclass_ferry);
          graphtile.directededges().emplace_back(de);
          DirectedEdge& directededge = graphtile.directededges().back();
          // temporarily set the leaves tile flag to indicate when we need to search the access.bin
          // file. ferries don't have overrides in country access logic, so use this bit to indicate
          // if the speed has been set via the duration and length
          if (!w.ferry()) {
            directededge.set_leaves_tile(w.has_user_tags());
          } else if (w.duration()) {
            directededge.set_leaves_tile(true);
          }

          directededge.set_edgeinfo_offset(found->first);
          directededge.set_curvature(std::get<1>(found->second));

          // Set use to ramp or turn channel
          if (edge.attributes.turn_channel) {
            directededge.set_use(Use::kTurnChannel);
          } else if (edge.attributes.link) {
            directededge.set_use(Use::kRamp);
          }

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.classification());

          // TODO - update logic so we limit the CreateExitSignInfoList calls
          // Any exits for this directed edge? is auto and oneway?
          std::vector<SignInfo> exits =
              GraphBuilder::CreateExitSignInfoList(node, w, osmdata, fork, forward);

          // Add signs if signs exist
          // and directed edge if forward access and auto use
          // and directed edge is a link and not (link count=2 and driveforward count=1)
          //    OR node is a fork
          if (!exits.empty() && (directededge.forwardaccess() & kAutoAccess) &&
              ((directededge.link() &&
                (!((bundle.link_count == 2) && (bundle.driveforward_count == 1)))) ||
               fork) &&
              ((edge.attributes.driveableforward && edge.attributes.way_begin) ||
               (edge.attributes.driveablereverse && edge.attributes.way_end))) {
            graphtile.AddSigns(idx, exits);
            directededge.set_exitsign(true);
          }

          // Add turn lanes if they exist. Store forward index on the last edge for a way
          // and the backward index on the first edge in a way.  The turn lanes are populated
          // later in the enhancer phase.
          std::string turnlane_tags;
          if (forward && w.fwd_turn_lanes_index() > 0 &&
              (edge.attributes.way_end || edge.attributes.way_prior)) {
            turnlane_tags = osmdata.name_offset_map.name(w.fwd_turn_lanes_index());
            if (!turnlane_tags.empty()) {
              std::string str = TurnLanes::GetTurnLaneString(turnlane_tags);
              if (!str.empty()) { // don't add if invalid.
                directededge.set_turnlanes(true);
                graphtile.AddTurnLanes(idx, w.fwd_turn_lanes_index());

                // Temporarily use the internal flag so that in the enhancer we can properly check to
                // see if we have an internal edge
                // Basically, we are setting turn lanes on the prior and last edge because we need
                // to check if the last edge is internal or not.  If it is internal, we remove the
                // turn lanes from the last edge and leave them on the prior.
                if (edge.attributes.way_prior)
                  directededge.set_internal(true);
              }
            }
          } else if (!forward && w.bwd_turn_lanes_index() > 0 &&
                     (edge.attributes.way_begin || edge.attributes.way_next)) {
            turnlane_tags = osmdata.name_offset_map.name(w.bwd_turn_lanes_index());
            if (!turnlane_tags.empty()) {
              std::string str = TurnLanes::GetTurnLaneString(turnlane_tags);
              if (!str.empty()) { // don't add if invalid.
                directededge.set_turnlanes(true);
                graphtile.AddTurnLanes(idx, w.bwd_turn_lanes_index());

                // Temporarily use the internal flag so that in the enhancer we can properly check to
                // see if we have an internal edge
                // Basically, we are setting turn lanes on the next and first edge because we need
                // to check if the fist edge is internal or not.  If it is internal, we remove the
                // turn lanes from the first edge and leave them on the next.
                if (edge.attributes.way_next)
                  directededge.set_internal(true);
              }
            }
          }
          // Add lane connectivity
          try {
            auto ei = osmdata.lane_connectivity_map.equal_range(w.way_id());
            if (ei.first != ei.second) {
              std::vector<LaneConnectivity> v;
              for (; ei.first != ei.second; ++ei.first) {
                const auto& lc = ei.first->second;
                v.emplace_back(idx, lc.from_way_id, osmdata.name_offset_map.name(lc.to_lanes_index),
                               osmdata.name_offset_map.name(lc.from_lanes_index));
              }
              graphtile.AddLaneConnectivity(v);
              directededge.set_laneconnectivity(true);
            }
          } catch (std::exception& e) {
            LOG_WARN("Failed to import lane connectivity for way: " + std::to_string(w.way_id()) +
                     " : " + e.what());
          }

          // Set the number of lanes.
          if (w.forward_tagged_lanes() && w.backward_tagged_lanes()) {
            if (forward) {
              directededge.set_lanecount(w.forward_lanes());
            } else {
              directededge.set_lanecount(w.backward_lanes());
            }
          } else {
            // The lanes tag in OSM means total number of lanes. For ways with
            // 2-way travel divide by 2. This will not be accurate for an odd
            // number of lanes, but in these cases there really should be
            // lanes:forward and lanes:backward tags.
            if (w.oneway() || w.oneway_reverse()) {
              directededge.set_lanecount(w.lanes());
            } else {
              directededge.set_lanecount(std::max(1, static_cast<int>(w.lanes()) / 2));
            }
          }

          // Add restrictions..For now only storing access restrictions for trucks
          // TODO - support more than one mode
          if (directededge.forwardaccess()) {
            uint32_t ar_modes = AddAccessRestrictions(idx, w.way_id(), osmdata, graphtile);
            if (ar_modes) {
              directededge.set_access_restriction(ar_modes);
            }
          }

          if (osmdata.via_set.find(w.way_id()) != osmdata.via_set.end()) {
            directededge.complex_restriction(true);
          }

          // grab all the modes if this way ends at a restriction(s)
          OSMRestriction target_to_res{
              w.way_id()}; // this is our from way id.  to is really our from for to_restrictions.
          OSMRestriction restriction_to{};
          sequence<OSMRestriction>::iterator res_to_it =
              complex_restrictions_to.find(target_to_res,
                                           [](const OSMRestriction& a, const OSMRestriction& b) {
                                             return a.from() < b.from();
                                           });

          while (res_to_it != complex_restrictions_to.end() &&
                 (restriction_to = *res_to_it).from() == w.way_id()) {
            directededge.set_end_restriction(directededge.end_restriction() | restriction_to.modes());
            res_to_it++;
          }

          // grab all the modes if this way starts at a restriction(s)
          OSMRestriction target_from_res{w.way_id()}; // this is our to way id
          OSMRestriction restriction_from{};
          sequence<OSMRestriction>::iterator res_from_it =
              complex_restrictions_from.find(target_from_res,
                                             [](const OSMRestriction& a, const OSMRestriction& b) {
                                               return a.from() < b.from();
                                             });

          while (res_from_it != complex_restrictions_from.end() &&
                 (restriction_from = *res_from_it).from() == w.way_id()) {
            directededge.set_start_restriction(directededge.start_restriction() |
                                               restriction_from.modes());
            res_from_it++;
          }

          // Set shoulder based on current facing direction and which
          // side of the road is meant to be driven on.
          if (forward) {
            directededge.set_shoulder(drive_on_right[admin_index] ? w.shoulder_right()
                                                                  : w.shoulder_left());
          } else {
            directededge.set_shoulder(drive_on_right[admin_index] ? w.shoulder_left()
                                                                  : w.shoulder_right());
          }

          // Figure out cycle lanes
          bool right_cyclelane_forward = true;
          bool left_cyclelane_forward = false;

          // If we know bikes can only go in a specific direction then whatever
          // cyclelanes there are will be in that direction
          if (w.bike_forward() && !w.bike_backward()) {
            right_cyclelane_forward = true;
            left_cyclelane_forward = true;
          } else if (!w.bike_forward() && w.bike_backward()) {
            right_cyclelane_forward = false;
            left_cyclelane_forward = false;
          }
          // If bike can go in both directions but the road is a oneway then we
          // must check for contraflow lanes
          else if (w.oneway()) {
            right_cyclelane_forward = !w.cyclelane_right_opposite();
            left_cyclelane_forward = !w.cyclelane_left_opposite();
            // If the way has a tag of oneway=-1 then bike tags are in the reverse direction
            if (w.oneway_reverse()) {
              right_cyclelane_forward = !right_cyclelane_forward;
              left_cyclelane_forward = !left_cyclelane_forward;
            }
          }
          // If road is not a oneway then we must consider contraflow lanes
          // as well as what side of the road people drive on
          else {
            right_cyclelane_forward = w.cyclelane_right_opposite() ? !drive_on_right[admin_index]
                                                                   : drive_on_right[admin_index];
            left_cyclelane_forward = w.cyclelane_left_opposite() ? drive_on_right[admin_index]
                                                                 : !drive_on_right[admin_index];
          }

          directededge.set_cyclelane(CycleLane::kNone);
          if (forward) {
            if (right_cyclelane_forward) {
              directededge.set_cyclelane(w.cyclelane_right());
            }
            if (left_cyclelane_forward && (static_cast<uint8_t>(w.cyclelane_left()) >
                                           static_cast<uint8_t>(directededge.cyclelane()))) {
              directededge.set_cyclelane(w.cyclelane_left());
            }
          } else {
            if (!right_cyclelane_forward) {
              directededge.set_cyclelane(w.cyclelane_right());
            }
            if (!left_cyclelane_forward && (static_cast<uint8_t>(w.cyclelane_left()) >
                                            static_cast<uint8_t>(directededge.cyclelane()))) {
              directededge.set_cyclelane(w.cyclelane_left());
            }
          }

          // Downgrade classification of any footways that are not kServiceOther
          if ((directededge.use() == Use::kFootway || directededge.use() == Use::kSteps ||
               directededge.use() == Use::kSidewalk || directededge.use() == Use::kPedestrian) &&
              directededge.classification() != RoadClass::kServiceOther) {
            directededge.set_classification(RoadClass::kServiceOther);
          }

          // Increment the directed edge index within the tile
          idx++;
          n++;
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        graphtile.nodes().emplace_back(base_ll, node_ll, bestclass, node.access(), node.type(),
                                       node.traffic_signal());
        graphtile.nodes().back().set_edge_index(graphtile.directededges().size() -
                                                bundle.node_edges.size());
        graphtile.nodes().back().set_edge_count(bundle.node_edges.size());
        if (fork) {
          graphtile.nodes().back().set_intersection(IntersectionType::kFork);
        }

        // Set admin index
        graphtile.nodes().back().set_admin_index(admin_index);

        // Set drive on right flag
        if (admin_index != 0) {
          graphtile.nodes().back().set_drive_on_right(drive_on_right[admin_index]);
        }

        // Set the time zone index
        uint32_t tz_index =
            (tile_within_one_tz) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, node_ll);

        graphtile.nodes().back().set_timezone(tz_index);

        // if you need to look at the attributes for nodes, grab the LL and update the if statement.
        // if (equal(node_ll.lng(), 120.99157f) && equal(node_ll.lat(), 14.584733f)) {
        //  std::cout <<
        //  std::to_string(GraphId(tile_id.id(),tile_id.level(),graphtile.nodes().size()).value) <<
        //  std::endl; std::cout << std::to_string(tile_within_one_admin) << " " <<
        //  std::to_string(tile_id.tileid()) << std::endl;
        // }

        // Increment the counts in the histogram
        stats.nodecount++;
        stats.directededge_count += bundle.node_edges.size();
        stats.node_counts[bundle.node_edges.size()]++;

        // Next node in the tile
        node_itr += bundle.node_count;
      }

      // Write the actual tile to disk
      graphtile.StoreTileData();

      // Made a tile
      LOG_DEBUG((boost::format("Wrote tile %1%: %2% bytes") % tile_start->first %
                 graphtile.header_builder().end_offset())
                    .str());
    } // Whatever happens in Vegas..
    catch (std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      LOG_ERROR((boost::format("Failed tile %1%: %2%") % tile_start->first % e.what()).str());
      return;
    }
  }

  if (admin_db_handle) {
    sqlite3_close(admin_db_handle);
  }

  if (tz_db_handle) {
    sqlite3_close(tz_db_handle);
  }

  // Let the main thread see how this thread faired
  result.set_value(stats);
}

// Build tiles for the local graph hierarchy
void BuildLocalTiles(const unsigned int thread_count,
                     const OSMData& osmdata,
                     const std::string& ways_file,
                     const std::string& way_nodes_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     const std::string& complex_from_restriction_file,
                     const std::string& complex_to_restriction_file,
                     const std::map<GraphId, size_t>& tiles,
                     const std::string& tile_dir,
                     DataQuality& stats,
                     const boost::property_tree::ptree& pt) {

  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  uint32_t tile_creation_date =
      DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));

  LOG_INFO("Building " + std::to_string(tiles.size()) + " tiles with " +
           std::to_string(thread_count) + " threads...");

  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // Hold the results (DataQuality/stats) for the threads
  std::vector<std::promise<DataQuality>> results(threads.size());

  // Divvy up the work
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::map<GraphId, size_t>::const_iterator tile_start, tile_end = tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(new std::thread(BuildTileSet, std::cref(ways_file), std::cref(way_nodes_file),
                                     std::cref(nodes_file), std::cref(edges_file),
                                     std::cref(complex_from_restriction_file),
                                     std::cref(complex_to_restriction_file), std::cref(tile_dir),
                                     std::cref(osmdata), tile_start, tile_end, tile_creation_date,
                                     std::cref(pt.get_child("mjolnir")), std::ref(results[i])));
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
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Build the graph from the input
void GraphBuilder::Build(const boost::property_tree::ptree& pt,
                         const OSMData& osmdata,
                         const std::string& ways_file,
                         const std::string& way_nodes_file,
                         const std::string& nodes_file,
                         const std::string& edges_file,
                         const std::string& complex_from_restriction_file,
                         const std::string& complex_to_restriction_file) {
  std::string tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  unsigned int threads =
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  const auto& tl = TileHierarchy::levels().rbegin();
  uint8_t level = tl->second.level;

  // Make the edges and nodes in the graph
  ConstructEdges(osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                 tl->second.tiles.TileSize(), [&level](const OSMNode& node) {
                   return TileHierarchy::GetGraphId({node.lng_, node.lat_}, level);
                 });

  // Line up the nodes and then re-map the edges that the edges to them
  auto tiles = SortGraph(nodes_file, edges_file, level);

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  DataQuality stats;
  if (pt.get<bool>("mjolnir.reclassify_links", true)) {
    ReclassifyLinks(ways_file, nodes_file, edges_file, way_nodes_file);
  } else {
    LOG_WARN("Not reclassifying link graph edges");
  }

  // Reclassify ferry connection edges - use the highway classification cutoff
  RoadClass rc = RoadClass::kPrimary;
  for (auto& level : TileHierarchy::levels()) {
    if (level.second.name == "highway") {
      rc = level.second.importance;
    }
  }
  ReclassifyFerryConnections(ways_file, way_nodes_file, nodes_file, edges_file,
                             static_cast<uint32_t>(rc), stats);

  // Build tiles at the local level. Form connected graph from nodes and edges.
  BuildLocalTiles(threads, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                  complex_from_restriction_file, complex_to_restriction_file, tiles, tile_dir, stats,
                  pt);

  stats.LogStatistics();
}

// Get highway refs from relations
std::string GraphBuilder::GetRef(const std::string& way_ref, const std::string& relation_ref) {
  bool found = false;
  std::string refs;
  std::vector<std::string> way_refs = GetTagTokens(way_ref);     // US 51;I 57
  std::vector<std::string> refdirs = GetTagTokens(relation_ref); // US 51|north;I 57|north
  for (auto& ref : way_refs) {
    found = false;
    for (const auto& refdir : refdirs) {
      std::vector<std::string> tmp = GetTagTokens(refdir, '|'); // US 51|north
      if (tmp.size() == 2) {
        if (tmp[0] == ref) { // US 51 == US 51
          if (!refs.empty()) {
            refs += ";" + ref + " " + tmp[1]; // ref order of the way wins.
          } else {
            refs = ref + " " + tmp[1];
          }
          found = true;
          break;
        } else if (tmp[0].find(" ") != std::string::npos &&
                   ref.find(" ") != std::string::npos) { // SR 747 vs OH 747
          std::vector<std::string> sign1 = GetTagTokens(tmp[0], ' ');
          std::vector<std::string> sign2 = GetTagTokens(ref, ' ');
          if (sign1.size() == 2 && sign2.size() == 2) {
            if (sign1[1] == sign2[1]) { // 747 == 747
              if (!refs.empty()) {
                refs += ";" + ref + " " + tmp[1]; // ref order of the way wins.
              } else {
                refs = ref + " " + tmp[1];
              }
              found = true;
              break;
            }
          }
        }
      }
    }

    if (!found) { // no direction found in relations for this ref
      if (!refs.empty()) {
        refs += ";" + ref;
      } else {
        refs = ref;
      }
    }
  }
  return refs;
}

std::vector<SignInfo> GraphBuilder::CreateExitSignInfoList(const OSMNode& node,
                                                           const OSMWay& way,
                                                           const OSMData& osmdata,
                                                           bool fork,
                                                           bool forward) {

  std::vector<SignInfo> exit_list;

  ////////////////////////////////////////////////////////////////////////////
  // NUMBER
  // Exit sign number
  if (way.junction_ref_index() != 0) {

    std::vector<std::string> j_refs =
        GetTagTokens(osmdata.name_offset_map.name(way.junction_ref_index()));
    for (auto& j_ref : j_refs) {
      exit_list.emplace_back(Sign::Type::kExitNumber, false, j_ref);
    }
  } else if (node.has_ref() && !fork) {
    std::vector<std::string> n_refs = GetTagTokens(osmdata.node_names.name(node.ref_index()));
    for (auto& n_ref : n_refs) {
      exit_list.emplace_back(Sign::Type::kExitNumber, false, n_ref);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // BRANCH

  bool has_branch = false;

  // Exit sign branch refs
  if (way.destination_ref_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_refs =
        GetTagTokens(osmdata.name_offset_map.name(way.destination_ref_index()));
    for (auto& branch_ref : branch_refs) {
      exit_list.emplace_back(Sign::Type::kExitBranch, true, branch_ref);
    }
  }

  // Exit sign branch road names
  if (way.destination_street_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_streets =
        GetTagTokens(osmdata.name_offset_map.name(way.destination_street_index()));
    for (auto& branch_street : branch_streets) {
      exit_list.emplace_back(Sign::Type::kExitBranch, false, branch_street);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // TOWARD

  bool has_toward = false;

  // Exit sign toward refs
  if (way.destination_ref_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_refs =
        GetTagTokens(osmdata.name_offset_map.name(way.destination_ref_to_index()));
    for (auto& toward_ref : toward_refs) {
      exit_list.emplace_back(Sign::Type::kExitToward, true, toward_ref);
    }
  }

  // Exit sign toward streets
  if (way.destination_street_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_streets =
        GetTagTokens(osmdata.name_offset_map.name(way.destination_street_to_index()));
    for (auto& toward_street : toward_streets) {
      exit_list.emplace_back(Sign::Type::kExitToward, false, toward_street);
    }
  }

  // Exit sign toward locations
  if (way.destination_index() != 0 || (forward && way.destination_forward_index() != 0) ||
      (!forward && way.destination_backward_index() != 0)) {
    has_toward = true;
    uint32_t index = way.destination_index() ? way.destination_index()
                                             : (forward ? way.destination_forward_index()
                                                        : way.destination_backward_index());
    std::vector<std::string> toward_names = GetTagTokens(osmdata.name_offset_map.name(index));
    for (auto& toward_name : toward_names) {
      exit_list.emplace_back(Sign::Type::kExitToward, false, toward_name);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // Process exit_to only if other branch or toward info does not exist
  if (!has_branch && !has_toward) {
    if (node.has_exit_to() && !fork) {
      std::string tmp;
      std::size_t pos;
      std::vector<std::string> exit_tos = GetTagTokens(osmdata.node_names.name(node.exit_to_index()));
      for (auto& exit_to : exit_tos) {

        tmp = exit_to;

        boost::algorithm::to_lower(tmp);

        // remove the "To" For example:  US 11;To I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "to ")) {
          exit_list.emplace_back(Sign::Type::kExitToward, false, exit_to.substr(3));
          continue;
        }
        // remove the "Toward" For example:  US 11;Toward I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "toward ")) {
          exit_list.emplace_back(Sign::Type::kExitToward, false, exit_to.substr(7));
          continue;
        }

        std::size_t found = tmp.find(" to ");

        // Default to kToward if found twice or "toward" found as well; otherwise, <branch> to
        // <toward> For example:  I 95 to I 695
        if (found != std::string::npos && (tmp.find(" to ", found + 4) == std::string::npos &&
                                           tmp.find(" toward ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, false, exit_to.substr(0, found));

          exit_list.emplace_back(Sign::Type::kExitToward, false, exit_to.substr(found + 4));
          continue;
        }

        found = tmp.find(" toward ");

        // Default to kToward if found twice or "to" found as well; otherwise, <branch> toward
        // <toward> For example:  I 95 to I 695
        if (found != std::string::npos && (tmp.find(" toward ", found + 8) == std::string::npos &&
                                           tmp.find(" to ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, false, exit_to.substr(0, found));

          exit_list.emplace_back(Sign::Type::kExitToward, false, exit_to.substr(found + 8));
          continue;
        }

        // default to toward.
        exit_list.emplace_back(Sign::Type::kExitToward, false, exit_to);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // NAME

  // Exit sign name
  if (node.has_name() && !fork) {
    // Get the name from OSMData using the name index
    std::vector<std::string> names = GetTagTokens(osmdata.node_names.name(node.name_index()));
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kExitName, false, name);
    }
  }

  return exit_list;
}

} // namespace mjolnir
} // namespace valhalla
