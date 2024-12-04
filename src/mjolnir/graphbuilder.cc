#include <future>
#include <memory>
#include <thread>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "baldr/conditional_speed_limit.h"
#include "filesystem.h"

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
#include "midgard/sequence.h"
#include "midgard/tiles.h"
#include "midgard/util.h"
#include "mjolnir/admin.h"
#include "mjolnir/edgeinfobuilder.h"
#include "mjolnir/ferry_connections.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/linkclassification.h"
#include "mjolnir/node_expander.h"
#include "mjolnir/util.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

/**
 * we need the nodes to be sorted by graphid and then by osmid to make a set of tiles
 * we also need to then update the edges that pointed to them
 *
 */
std::map<GraphId, size_t> SortGraph(const std::string& nodes_file, const std::string& edges_file) {
  LOG_INFO("Sorting graph...");

  // Sort nodes by graphid then by grid within the tile. This sorts nodes geo-spatially which
  // helps performance by improving memory coherence.
  sequence<Node> nodes(nodes_file, false);
  nodes.sort([](const Node& a, const Node& b) {
    if (a.graph_id == b.graph_id) {
      if (a.grid_id == b.grid_id) {
        return a.node.osmid_ < b.node.osmid_;
      } else {
        return a.grid_id < b.grid_id;
      }
    }
    return a.graph_id < b.graph_id;
  });

  // run through the sorted nodes, going back to the edges they reference and updating each edge
  // to point to the first (out of the duplicates) nodes index. at the end of this there will be
  // tons of nodes that no edges reference, but we need them because they are the means by which
  // we know what edges connect to a given node from the nodes perspective

  auto start_node_edge_file = filesystem::path(edges_file);
  start_node_edge_file.replace_filename(start_node_edge_file.filename().string() + ".starts.tmp");
  auto end_node_edge_file = filesystem::path(edges_file);
  end_node_edge_file.replace_filename(end_node_edge_file.filename().string() + ".ends.tmp");
  using edge_ends_t = sequence<std::pair<uint32_t, uint32_t>>;
  std::unique_ptr<edge_ends_t> starts(new edge_ends_t(start_node_edge_file.string(), true));
  std::unique_ptr<edge_ends_t> ends(new edge_ends_t(end_node_edge_file.string(), true));

  uint32_t run_index = 0;
  uint32_t node_index = 0;
  size_t node_count = 0;
  Node last_node{};
  std::map<GraphId, size_t> tiles;
  nodes.transform(
      [&starts, &ends, &run_index, &node_index, &node_count, &last_node, &tiles](Node& node) {
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

        // if this node marks the start of an edge, keep track of the edge and the node
        // so we can later tell the edge where the first node in the series is
        if (node.is_start()) {
          starts->push_back(std::make_pair(node.start_of, run_index));
        }
        // if this node marks the end of an edge, keep track of the edge and the node
        // so we can later tell the edge where the final node in the series is
        if (node.is_end()) {
          ends->push_back(std::make_pair(node.end_of, run_index));
        }

        // next node
        last_node = node;
        ++node_index;
      });

  // every edge should have a begin and end node
  assert(starts->size() == ends->size());

  LOG_INFO("Nodes processed. Sorting begin and end nodes by edge.");

  // Sort by edge. This enables a sequential update of edges
  auto cmp = [](const std::pair<uint32_t, uint32_t>& a, const std::pair<uint32_t, uint32_t>& b) {
    return a.first < b.first;
  };
  starts->sort(cmp);
  ends->sort(cmp);

  sequence<Edge> edges(edges_file, false);

  LOG_INFO("Sorting begin and end nodes done. Populating edges...");

  auto s_it = starts->begin(), e_it = ends->begin();
  while (s_it != starts->end() && e_it != ends->end()) {
    // there should be exactly one edge per begin and end node sequence and we sorted them the same
    assert((*s_it).first == (*e_it).first);
    auto element = edges[(*s_it).first];
    auto edge = *element;
    edge.sourcenode_ = (*s_it).second;
    edge.targetnode_ = (*e_it).second;
    element = edge;
    ++s_it;
    ++e_it;
  }

  // clean up tmp files
  starts.reset();
  ends.reset();
  filesystem::remove(start_node_edge_file);
  filesystem::remove(end_node_edge_file);

  LOG_INFO("Finished with " + std::to_string(node_count) + " graph nodes");
  return tiles;
}

// Construct edges in the graph and assign nodes to tiles.
void ConstructEdges(const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const std::function<GraphId(const OSMNode&)>& graph_id_predicate,
                    const std::function<uint32_t(const OSMNode&)>& grid_id_predicate,
                    const bool infer_turn_channels) {
  LOG_INFO("Creating graph edges from ways...");

  // so we can read ways and nodes and write edges
  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, true);
  sequence<Node> nodes(nodes_file, true);

  // Method to get length of an edge (used to find short link edges)
  const auto Length = [&way_nodes](const size_t idx1, const OSMNode& node2) {
    auto node1 = (*way_nodes[idx1]).node;
    return node1.latlng().Distance(node2.latlng());
  };

  // For each way traversed via the nodes
  GraphId graphid;
  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    // Grab the way and its first node
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index =
        first_way_node_index + way.node_count() - way_node.way_shape_node_index - 1;

    // Validate - make sure all nodes for this edge are valid
    bool valid = true;
    for (auto ni = current_way_node_index; ni <= last_way_node_index; ni++) {
      const auto wn = (*way_nodes[ni]).node;
      if (!wn.latlng().IsValid()) {
        LOG_WARN("Node " + std::to_string(wn.osmid_) + " in way " + std::to_string(way.way_id()) +
                 " has not had coordinates initialized");
        valid = false;
      }
    }
    if (!valid) {
      LOG_WARN("Do not add edge!");
      current_way_node_index = last_way_node_index + 1;
      continue;
    }

    // Remember this edge starts here
    Edge prev_edge = Edge{0};
    Edge edge = Edge::make_edge(way_node.way_index, current_way_node_index, way, infer_turn_channels);
    edge.attributes.way_begin = way_node.way_shape_node_index == 0;

    // Remember this node as starting this edge
    way_node.node.link_edge_ = way.link();
    way_node.node.non_link_edge_ = !way.link() && (way.auto_forward() || way.auto_backward());
    nodes.push_back({way_node.node, static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(-1),
                     graph_id_predicate(way_node.node), grid_id_predicate(way_node.node)});

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

        // remember what edge this node will end, its complicated by the fact that we delay adding the
        // edge until the next iteration of the loop, ie once the edge becomes prev_edge
        uint32_t end_of = static_cast<uint32_t>(edges.size() + prev_edge.is_valid());
        nodes.push_back({way_node.node, static_cast<uint32_t>(-1), end_of,
                         graph_id_predicate(way_node.node), grid_id_predicate(way_node.node)});

        // Mark the edge as ending a way if this is the last node in the way
        edge.attributes.way_end = current_way_node_index == last_way_node_index;

        // We should add the previous edge now that we know its done
        if (prev_edge.is_valid())
          edges.push_back(prev_edge);

        // Finish this edge
        prev_edge = edge;

        // Figure out if the way doubled back on itself and if so treat it like the way ended
        bool doubled_back = false;
        OSMWayNode next_way_node;
        while (current_way_node_index != last_way_node_index && way_node.node.flat_loop_ &&
               (next_way_node = *way_nodes[current_way_node_index + 1]).node.flat_loop_) {
          way_node = next_way_node;
          ++current_way_node_index;
          doubled_back = current_way_node_index != last_way_node_index;
        }

        // Either we were done making edges from this way or the is an internal part that is doubled
        // backed over itself and we need to skip it
        if (current_way_node_index == last_way_node_index || doubled_back) {
          edges.push_back(prev_edge);
          current_way_node_index += !doubled_back;
          break;
        } // Start a new edge if this is not the last node in the way
        else {
          edge =
              Edge::make_edge(way_node.way_index, current_way_node_index, way, infer_turn_channels);
          sequence<Node>::iterator element = --nodes.end();
          auto node = *element;
          node.start_of = edges.size() + 1; // + 1 because the edge has not been added yet
          element = node;
        }
      } // If this edge has a signal not at a intersection
      else if (way_node.node.traffic_signal()) {
        edge.attributes.traffic_signal = true;
        edge.attributes.forward_signal = way_node.node.forward_signal();
        edge.attributes.backward_signal = way_node.node.backward_signal();
      } else if (way_node.node.stop_sign()) {
        edge.attributes.stop_sign = true;
        edge.attributes.direction = way_node.node.direction();
        edge.attributes.forward_stop = way_node.node.forward_stop();
        edge.attributes.backward_stop = way_node.node.backward_stop();
      } else if (way_node.node.yield_sign()) {
        edge.attributes.yield_sign = true;
        edge.attributes.direction = way_node.node.direction();
        edge.attributes.forward_yield = way_node.node.forward_yield();
        edge.attributes.backward_yield = way_node.node.backward_yield();
      }
    }
  }
  LOG_INFO("Finished with " + std::to_string(edges.size()) + " graph edges");
}

uint32_t CreateSimpleTurnRestriction(const uint64_t wayid,
                                     const size_t endnode,
                                     sequence<Node>& nodes,
                                     sequence<Edge>& edges,
                                     const OSMData& osmdata,
                                     sequence<OSMWay>& ways) {

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
      case RestrictionType::kOnlyProbable:
      case RestrictionType::kNoProbable:
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
                               const uint64_t wayid,
                               const OSMData& osmdata,
                               const bool forward,
                               GraphTileBuilder& graphtile) {
  auto res = osmdata.access_restrictions.equal_range(wayid);
  if (res.first == osmdata.access_restrictions.end()) {
    return 0;
  }

  uint32_t modes = 0;
  for (auto r = res.first; r != res.second; ++r) {
    auto direction = r->second.direction();

    if ((direction == AccessRestrictionDirection::kBoth) ||
        (forward && direction == AccessRestrictionDirection::kForward) ||
        (!forward && direction == AccessRestrictionDirection::kBackward)) {
      AccessRestriction access_restriction(edgeid, r->second.type(), r->second.modes(),
                                           r->second.value());
      graphtile.AddAccessRestriction(access_restriction);
      modes |= r->second.modes();
    }
  }
  return modes;
}

void BuildTileSet(const std::string& ways_file,
                  const std::string& way_nodes_file,
                  const std::string& nodes_file,
                  const std::string& edges_file,
                  const std::string& complex_restriction_from_file,
                  const std::string& complex_restriction_to_file,
                  const std::string& linguistic_node_file,
                  const std::string& tile_dir,
                  const OSMData& osmdata,
                  std::queue<std::pair<GraphId, size_t>>& tiles,
                  std::mutex& tiles_lock,
                  const uint32_t tile_creation_date,
                  const boost::property_tree::ptree& pt,
                  std::promise<DataQuality>& result) {

  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);
  sequence<OSMRestriction> complex_restrictions_from(complex_restriction_from_file, false);
  sequence<OSMRestriction> complex_restrictions_to(complex_restriction_to_file, false);
  sequence<OSMNodeLinguistic> linguistic_node(linguistic_node_file, false);

  auto database = pt.get_optional<std::string>("admin");
  bool infer_internal_intersections =
      pt.get<bool>("data_processing.infer_internal_intersections", true);
  bool use_urban_tag = pt.get<bool>("data_processing.use_urban_tag", false);
  bool use_admin_db = pt.get<bool>("data_processing.use_admin_db", true);

  // Initialize the admin DB (if it exists)
  sqlite3* admin_db_handle = (database && use_admin_db) ? GetDBHandle(*database) : nullptr;
  if (!database && use_admin_db) {
    LOG_WARN("Admin db not found.  Not saving admin information.");
  } else if (!admin_db_handle && use_admin_db) {
    LOG_WARN("Admin db " + *database + " not found.  Not saving admin information.");
  }
  auto admin_conn = make_spatialite_cache(admin_db_handle);

  database = pt.get_optional<std::string>("timezone");
  // Initialize the tz DB (if it exists)
  sqlite3* tz_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database) {
    LOG_WARN("Time zone db not found.  Not saving time zone information.");
  } else if (!tz_db_handle) {
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information.");
  }
  auto tz_conn = make_spatialite_cache(tz_db_handle);

  const auto& tiling = TileHierarchy::levels().back().tiles;

  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::list<PointLL> shape;
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.latlng());
    }
    return shape;
  };

  // For each tile in the task
  bool added = false;
  DataQuality stats;

  // Lots of times in a given tile we may end up accessing the same
  // shape/attributes twice we avoid doing this by caching it here
  std::unordered_map<uint32_t, std::pair<double, uint32_t>> geo_attribute_cache;

  std::map<std::pair<uint8_t, uint8_t>, uint32_t> pronunciationMap;
  std::map<std::pair<uint8_t, uint8_t>, uint32_t> langMap;
  ////////////////////////////////////////////////////////////////////////////
  // Iterate over tiles
  while (true) {
    tiles_lock.lock();
    if (tiles.empty()) {
      // all work done
      tiles_lock.unlock();
      break;
    }

    auto tile = tiles.front();
    tiles.pop();
    tiles_lock.unlock();

    try {
      // What actually writes the tile
      GraphId tile_id = tile.first.Tile_Base();
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
      std::multimap<uint32_t, multi_polygon_type> admin_polys;
      std::unordered_map<uint32_t, bool> drive_on_right;
      std::unordered_map<uint32_t, bool> allow_intersection_names;
      language_poly_index language_polys;

      if (admin_db_handle) {
        admin_polys = GetAdminInfo(admin_db_handle, drive_on_right, allow_intersection_names,
                                   language_polys, tiling.TileBounds(id), graphtile);
        if (admin_polys.size() == 1) {
          // TODO - check if tile bounding box is entirely inside the polygon...
          tile_within_one_admin = true;
        }
      }

      bool tile_within_one_tz = false;
      auto tz_polys = GetTimeZones(tz_db_handle, tiling.TileBounds(id));
      if (tz_polys.size() == 1) {
        tile_within_one_tz = true;
      }

      // Iterate through the nodes
      uint32_t idx = 0; // Current directed edge index

      ////////////////////////////////////////////////////////////////////////
      // Iterate over nodes in the tile
      auto node_itr = nodes[tile.second];
      // to avoid realloc we guess how many edges there might be in a given tile
      geo_attribute_cache.clear();
      // geo_attribute_cache.reserve(5 * (std::next(tile_start) == tile_end
      //                                      ? nodes.end() - node_itr
      //                                      : std::next(tile_start)->second - tile_start->second));

      while (node_itr != nodes.end() && (*node_itr).graph_id.Tile_Base() == tile_id) {
        // amalgamate all the node duplicates into one and the edges that connect to it
        // this moves the iterator for you
        auto bundle = collect_node_edges(node_itr, nodes, edges);

        // Make sure node has edges
        if (bundle.node_edges.empty()) {
          LOG_ERROR("Node has no edges - skip");
          continue;
        }

        const auto& node = bundle.node;
        PointLL node_ll = node.latlng();

        // Get the admin index
        uint32_t admin_index = 0;
        bool dor = false;
        std::vector<std::pair<std::string, bool>> default_languages;

        if (use_admin_db) {
          admin_index = (tile_within_one_admin) ? admin_polys.begin()->first
                                                : GetMultiPolyId(admin_polys, node_ll, graphtile);
          dor = drive_on_right[admin_index];
          default_languages = GetMultiPolyIndexes(language_polys, node_ll);

        } else {
          admin_index = graphtile.AddAdmin("", "", osmdata.node_names.name(node.country_iso_index()),
                                           osmdata.node_names.name(node.state_iso_index()));
        }

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

          if (!use_admin_db)
            dor = w.drive_on_right();

          // Validate speed. Set speed limit and truck speed.
          uint32_t speed = w.speed();
          if (forward && w.forward_tagged_speed()) {
            speed = w.forward_speed();
          } else if (!forward && w.backward_tagged_speed()) {
            speed = w.backward_speed();
          }
          if (speed > kMaxAssumedSpeed) {
            LOG_WARN("Speed = " + std::to_string(speed) + " wayId= " + std::to_string(w.way_id()));
            speed = kMaxAssumedSpeed;
          }
          uint32_t speed_limit = w.speed_limit();
          if (speed_limit > kMaxAssumedSpeed && speed_limit != kUnlimitedSpeedLimit) {
            LOG_WARN("Speed limit = " + std::to_string(speed_limit) +
                     " wayId= " + std::to_string(w.way_id()));
            speed_limit = kMaxAssumedSpeed;
          }

          const uint8_t directed_truck_speed =
              forward ? w.truck_speed_forward() : w.truck_speed_backward();

          // if there's a general truck speed AND a directed one, apply the stricter one
          // otherwise just pick whichever is set
          uint32_t truck_speed = w.truck_speed() && directed_truck_speed
                                     ? std::min(w.truck_speed(), directed_truck_speed)
                                     : std::max(w.truck_speed(), directed_truck_speed);

          if (truck_speed > kMaxAssumedSpeed) {
            LOG_WARN("Truck Speed = " + std::to_string(truck_speed) +
                     " wayId= " + std::to_string(w.way_id()));
            truck_speed = kMaxAssumedSpeed;
          }

          // Cul du sac
          auto use = w.use();
          if (use == Use::kCuldesac) {
            stats.culdesaccount++;
          }

          // Handle simple turn restrictions that originate from this
          // directed edge
          uint32_t restrictions =
              CreateSimpleTurnRestriction(w.way_id(), target, nodes, edges, osmdata, ways);
          if (restrictions != 0) {
            stats.simplerestrictions++;
          }

          // traffic signal exists at a non-intersection node
          // forward signal must exist if forward direction and vice versa.
          // if forward and backward signal flags are not set then only set for oneways.
          bool has_signal =
              ((edge.attributes.traffic_signal) &&
               ((forward && edge.attributes.forward_signal) ||
                (!forward && edge.attributes.backward_signal) ||
                (w.oneway() && !edge.attributes.forward_signal && !edge.attributes.backward_signal)));

          // stop sign exists at a non-intersection node
          // forward stop must exist if forward direction and vice versa.
          // if forward and backward stop flags are not set then only set for oneways.
          bool has_stop =
              ((edge.attributes.stop_sign) &&
               ((forward && (edge.attributes.direction ? edge.attributes.forward_stop : true)) ||
                (!forward && (edge.attributes.direction ? edge.attributes.backward_stop : true)) ||
                (w.oneway() && !edge.attributes.forward_stop && !edge.attributes.backward_stop)));

          // yield sign exists at a non-intersection node
          // forward yield must exist if forward direction and vice versa.
          // if forward and backward yield flags are not set then only set for oneways.
          bool has_yield =
              ((edge.attributes.yield_sign) &&
               ((forward && (edge.attributes.direction ? edge.attributes.forward_yield : true)) ||
                (!forward && (edge.attributes.direction ? edge.attributes.backward_yield : true)) ||
                (w.oneway() && !edge.attributes.forward_yield && !edge.attributes.backward_yield)));

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

          pronunciationMap.clear();
          auto pron = osmdata.pronunciations.equal_range(w.way_id());
          if (pron.first != osmdata.pronunciations.end()) {
            for (auto p = pron.first; p != pron.second; ++p) {
              pronunciationMap[std::make_pair(p->second.key_.type_, p->second.key_.alpha_)] =
                  p->second.name_offset();
            }
          }

          langMap.clear();
          auto lang = osmdata.langs.equal_range(w.way_id());
          if (lang.first != osmdata.langs.end()) {
            for (auto l = lang.first; l != lang.second; ++l) {
              langMap[std::make_pair(l->second.key_.type_, l->second.key_.alpha_)] =
                  l->second.name_offset();
            }
          }

          // Get the shape for the edge and compute its length
          uint32_t edge_info_offset;
          auto found = geo_attribute_cache.cend();
          if (((w.ref_left_index() && w.ref_right_index()) ||
               (w.name_left_index() && w.name_right_index()) ||
               (w.official_name_left_index() && w.official_name_right_index()) ||
               (w.alt_name_left_index() && w.alt_name_right_index()) ||
               (w.tunnel_name_left_index() && w.tunnel_name_right_index())) ||
              dual_refs || (w.name_forward_index() && w.name_backward_index()) ||
              !graphtile.HasEdgeInfo(edge_pair.second, (*nodes[source]).graph_id,
                                     (*nodes[target]).graph_id, edge_info_offset)) {

            // add the info
            auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);

            bool diff_names = false;
            OSMLinguistic::DiffType type = OSMLinguistic::DiffType::kRight;
            uint32_t name_index = w.name_index(), name_lang_index = w.name_lang_index();
            if (w.name_right_index() && forward) {
              name_index = w.name_right_index();
              name_lang_index = w.name_right_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kRight;
            } else if (w.name_left_index() && !forward) {
              name_index = w.name_left_index();
              name_lang_index = w.name_left_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kLeft;
            } else if (w.name_forward_index() && forward) {
              name_index = w.name_forward_index();
              name_lang_index = w.name_forward_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kForward;
            } else if (w.name_backward_index() && !forward) {
              name_index = w.name_backward_index();
              name_lang_index = w.name_backward_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kBackward;
            }

            uint32_t official_name_index = w.official_name_index(),
                     official_name_lang_index = w.official_name_lang_index();
            if (w.official_name_right_index() && forward) {
              official_name_index = w.official_name_right_index();
              official_name_lang_index = w.official_name_right_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kRight;
            } else if (w.official_name_left_index() && !forward) {
              official_name_index = w.official_name_left_index();
              official_name_lang_index = w.official_name_left_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kLeft;
            }

            uint32_t alt_name_index = w.alt_name_index(),
                     alt_name_lang_index = w.alt_name_lang_index();
            if (w.alt_name_right_index() && forward) {
              alt_name_index = w.alt_name_right_index();
              alt_name_lang_index = w.alt_name_right_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kRight;
            } else if (w.alt_name_left_index() && !forward) {
              alt_name_index = w.alt_name_left_index();
              alt_name_lang_index = w.alt_name_left_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kLeft;
            }

            uint32_t ref_index = w.ref_index(), ref_lang_index = w.ref_lang_index();
            if (w.ref_right_index() && forward) {
              ref_index = w.ref_right_index();
              ref_lang_index = w.ref_right_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kRight;
            } else if (w.ref_left_index() && !forward) {
              ref_index = w.ref_left_index();
              ref_lang_index = w.ref_left_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kLeft;
            }

            uint32_t tunnel_index = w.tunnel_name_index(),
                     tunnel_lang_index = w.tunnel_name_lang_index();
            if (w.tunnel_name_right_index() && forward) {
              tunnel_index = w.tunnel_name_right_index();
              tunnel_lang_index = w.tunnel_name_right_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kRight;
            } else if (w.tunnel_name_left_index() && !forward) {
              tunnel_index = w.tunnel_name_left_index();
              tunnel_lang_index = w.tunnel_name_left_lang_index();
              diff_names = true;
              type = OSMLinguistic::DiffType::kLeft;
            }

            uint16_t types = 0;

            std::vector<std::string> names, tagged_values, linguistics;
            w.GetNames(ref, osmdata.name_offset_map, pronunciationMap, langMap, default_languages,
                       ref_index, ref_lang_index, name_index, name_lang_index, official_name_index,
                       official_name_lang_index, alt_name_index, alt_name_lang_index, types, names,
                       linguistics, type, diff_names);
            w.GetTaggedValues(osmdata.name_offset_map, pronunciationMap, langMap, default_languages,
                              tunnel_index, tunnel_lang_index, names.size(), tagged_values,
                              linguistics, type, diff_names);

            // Append conditional limits as tagged values
            const auto cond_limits_range = osmdata.conditional_speeds.equal_range(w.way_id());
            for (auto it = cond_limits_range.first; it != cond_limits_range.second; ++it) {
              std::string value;
              value.reserve(1 + sizeof(ConditionalSpeedLimit));
              value += static_cast<std::string::value_type>(TaggedValue::kConditionalSpeedLimits);
              value.append(reinterpret_cast<const std::string::value_type*>(&it->second),
                           sizeof(ConditionalSpeedLimit));
              tagged_values.push_back(std::move(value));
            }

            // Update bike_network type
            if (bike_network) {
              bike_network |= w.bike_network();
            } else {
              bike_network = w.bike_network();
            }

            edge_info_offset =
                graphtile.AddEdgeInfo(edge_pair.second, (*nodes[source]).graph_id,
                                      (*nodes[target]).graph_id, w.way_id(), kNoElevationData,
                                      bike_network, speed_limit, shape, names, tagged_values,
                                      linguistics, types, added, (diff_names || dual_refs));

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
            uint32_t spd = static_cast<uint32_t>((std::get<0>(found->second) * 3.6) / w.duration());
            speed = (spd == 0) ? 1 : spd;
          }

          // Add a directed edge and get a reference to it
          DirectedEdgeBuilder de(w, (*nodes[target]).graph_id, forward,
                                 static_cast<uint32_t>(std::get<0>(found->second) + .5), speed,
                                 truck_speed, use, static_cast<RoadClass>(edge.attributes.importance),
                                 n, has_signal, has_stop, has_yield,
                                 ((has_stop || has_yield) ? node.minor() : false), restrictions,
                                 bike_network, edge.attributes.reclass_ferry);
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
          if (edge.attributes.turn_channel && use != Use::kConstruction) {
            directededge.set_use(Use::kTurnChannel);
            // Do not overwrite rest area or service area use for ramps
          } else if (edge.attributes.link && (use != Use::kServiceArea && use != Use::kRestArea &&
                                              use != Use::kConstruction)) {
            directededge.set_use(Use::kRamp);
          }

          if (!infer_internal_intersections && w.internal()) {
            if (directededge.use() != Use::kRamp && directededge.use() != Use::kTurnChannel)
              directededge.set_internal(true);
          }

          // TODO - update logic so we limit the CreateSignInfoList calls
          // Any exits for this directed edge? is auto and oneway?
          std::vector<SignInfo> signs;
          std::vector<std::string> linguistics;

          bool has_guide =
              GraphBuilder::CreateSignInfoList(node, w, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, (directededge.use() == Use::kRamp),
                                               (directededge.use() == Use::kTurnChannel));
          // add signs if signs exist
          // and directed edge if forward access and auto use
          // and directed edge is a link and not (link count=2 and driveforward count=1)
          //    OR node is a fork
          //    OR we added guide signs or guidance views
          if (!signs.empty() && (directededge.forwardaccess() & kAutoAccess) &&
              ((directededge.link() &&
                (!((bundle.link_count == 2) && (bundle.driveforward_count == 1)))) ||
               fork || has_guide)) {
            graphtile.AddSigns(idx, signs, linguistics);
            directededge.set_sign(true);
          }

          // Add turn lanes if they exist.
          std::string turnlane_tags;
          if (forward && w.fwd_turn_lanes_index() > 0) {
            turnlane_tags = osmdata.name_offset_map.name(w.fwd_turn_lanes_index());
            if (!turnlane_tags.empty()) {
              std::string str = TurnLanes::GetTurnLaneString(turnlane_tags);
              if (!str.empty()) { // don't add if invalid.
                directededge.set_turnlanes(true);
                graphtile.AddTurnLanes(idx, w.fwd_turn_lanes_index());
              }
            }
          } else if (!forward && w.bwd_turn_lanes_index() > 0) {
            turnlane_tags = osmdata.name_offset_map.name(w.bwd_turn_lanes_index());
            if (!turnlane_tags.empty()) {
              std::string str = TurnLanes::GetTurnLaneString(turnlane_tags);
              if (!str.empty()) { // don't add if invalid.
                directededge.set_turnlanes(true);
                graphtile.AddTurnLanes(idx, w.bwd_turn_lanes_index());
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
            uint32_t ar_modes =
                AddAccessRestrictions(idx, w.way_id(), osmdata, directededge.forward(), graphtile);
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
            directededge.set_shoulder(dor ? w.shoulder_right() : w.shoulder_left());
          } else {
            directededge.set_shoulder(dor ? w.shoulder_left() : w.shoulder_right());
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
            right_cyclelane_forward = w.cyclelane_right_opposite() ? !dor : dor;
            left_cyclelane_forward = w.cyclelane_left_opposite() ? dor : !dor;
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
        graphtile.nodes().emplace_back(base_ll, node_ll, node.access(), node.type(),
                                       node.traffic_signal(), node.tagged_access(),
                                       node.private_access(), node.cash_only_toll());
        graphtile.nodes().back().set_edge_index(graphtile.directededges().size() -
                                                bundle.node_edges.size());
        graphtile.nodes().back().set_edge_count(bundle.node_edges.size());
        if (fork) {
          graphtile.nodes().back().set_intersection(IntersectionType::kFork);
        }

        // Set admin index
        graphtile.nodes().back().set_admin_index(admin_index);

        if ((node.stop_sign() && !node.yield_sign()) || (!node.stop_sign() && node.yield_sign())) {

          // temporarily set the transition index so that we know if we have a stop or yield sign
          // will be processed and removed in the enhancer
          uint8_t stop_yield_info = 0;
          if (node.minor())
            stop_yield_info |= kMinor;

          if (node.stop_sign())
            stop_yield_info |= kStopSign;

          if (node.yield_sign())
            stop_yield_info |= kYieldSign;

          graphtile.nodes().back().set_transition_index(stop_yield_info);
        }

        if (node.named_intersection()) {
          if ((admin_index != 0 && allow_intersection_names[admin_index]) ||
              (node.type() == NodeType::kTollBooth) || (node.type() == NodeType::kTollGantry)) {

            std::vector<std::string> node_names;
            std::vector<valhalla::baldr::Language> node_langs;
            OSMNodeLinguistic ling;
            if (node.linguistic_info_index() != 0) {
              ling = linguistic_node.at(node.linguistic_info_index());
            }
            OSMWay::ProcessNamesPronunciations(osmdata.node_names, default_languages,
                                               node.name_index(), ling.name_lang_index(), node_names,
                                               node_langs, false);

            std::vector<SignInfo> signs;
            signs.reserve(node_names.size());

            std::vector<std::string> ipa_tokens, nt_sampa_tokens, katakana_tokens, jeita_tokens;
            std::vector<valhalla::baldr::Language> ipa_langs, nt_sampa_langs, katakana_langs,
                jeita_langs;

            GraphBuilder::GetPronunciationTokens(osmdata.node_names, default_languages,
                                                 ling.name_pronunciation_ipa_index(),
                                                 ling.name_pronunciation_ipa_lang_index(),
                                                 ling.name_pronunciation_nt_sampa_index(),
                                                 ling.name_pronunciation_nt_sampa_lang_index(),
                                                 ling.name_pronunciation_katakana_index(),
                                                 ling.name_pronunciation_katakana_lang_index(),
                                                 ling.name_pronunciation_jeita_index(),
                                                 ling.name_pronunciation_jeita_lang_index(),
                                                 ipa_tokens, ipa_langs, nt_sampa_tokens,
                                                 nt_sampa_langs, katakana_tokens, katakana_langs,
                                                 jeita_tokens, jeita_langs);

            bool add_ipa = (ipa_tokens.size() != 0);
            bool add_nt_sampa = (nt_sampa_tokens.size() != 0);
            bool add_katakana = (katakana_tokens.size() != 0);
            bool add_jeita = (jeita_tokens.size() != 0);

            std::vector<std::string> linguistics;
            std::map<size_t, valhalla::baldr::Language> lang_map;

            uint32_t phoneme_count = 0;
            size_t phoneme_start_index = 0;

            size_t key = signs.size();
            bool add_phoneme = (add_ipa || add_nt_sampa || add_katakana || add_jeita);
            if (add_phoneme) {

              GraphBuilder::BuildPronunciations(ipa_tokens, ipa_langs, nt_sampa_tokens,
                                                nt_sampa_langs, katakana_tokens, katakana_langs,
                                                jeita_tokens, jeita_langs, node_langs, signs.size(),
                                                key, linguistics, lang_map, add_ipa, add_nt_sampa,
                                                add_katakana, add_jeita);
              phoneme_count = linguistics.size();
              if (phoneme_count == 0)
                add_phoneme = false;
            }

            for (size_t i = 0; i < node_names.size(); ++i) {

              if (lang_map.size() != 0) {
                // only add a language if it has not already been added during pronunciation
                // processing
                auto itr = lang_map.find(i);
                if (itr != lang_map.end()) {
                  GraphBuilder::AddLanguage(i, itr->second, linguistics);
                }
              } else if (i < node_langs.size() && !add_phoneme) {
                phoneme_start_index = linguistics.size();
                GraphBuilder::AddLanguage(key, node_langs.at(i), linguistics);
                phoneme_count = linguistics.size() - phoneme_start_index;
                key++;
              }

              Sign::Type sign_type = Sign::Type::kJunctionName;
              if ((node.type() == NodeType::kTollBooth) || (node.type() == NodeType::kTollGantry))
                sign_type = Sign::Type::kTollName;

              signs.emplace_back(sign_type, false, false, (phoneme_count != 0), phoneme_start_index,
                                 phoneme_count, node_names[i]);
            }

            if (signs.size()) {
              graphtile.nodes().back().set_named_intersection(true);
              graphtile.AddSigns(graphtile.nodes().size() - 1, signs, linguistics);
            }
          }
        }
        // Set drive on right flag
        graphtile.nodes().back().set_drive_on_right(dor);

        // Set the time zone index
        uint32_t tz_index =
            (tile_within_one_tz) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, node_ll);

        graphtile.nodes().back().set_timezone(tz_index);

        // set the density as needed.
        if (use_urban_tag && node.urban())
          graphtile.nodes().back().set_density(kMaxDensity);

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
      LOG_DEBUG((boost::format("Wrote tile %1%: %2% bytes") % tile.first %
                 graphtile.header_builder().end_offset())
                    .str());
    } // Whatever happens in Vegas..
    catch (std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      LOG_ERROR((boost::format("Failed tile %1%: %2%") % tile.first % e.what()).str());
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
                     const std::string& linguistic_node_file,
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
  std::queue<std::pair<GraphId, size_t>> tile_queue;
  std::mutex tile_lock;
  for (auto id : tiles) {
    tile_queue.emplace(id);
  }

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Make the thread
    threads[i] =
        std::make_shared<std::thread>(BuildTileSet, std::cref(ways_file), std::cref(way_nodes_file),
                                      std::cref(nodes_file), std::cref(edges_file),
                                      std::cref(complex_from_restriction_file),
                                      std::cref(complex_to_restriction_file),
                                      std::cref(linguistic_node_file), std::cref(tile_dir),
                                      std::cref(osmdata), std::ref(tile_queue), std::ref(tile_lock),
                                      tile_creation_date, std::cref(pt.get_child("mjolnir")),
                                      std::ref(results[i]));
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
    } // If we couldnt write a tile for whatever reason we fail the whole job
    catch (std::exception& e) {
      throw e;
    }
  }
}

} // namespace

namespace valhalla::mjolnir {

// Returns the grid Id within the tile. A tile is subdivided into a nxn grid.
// The grid Id within the tile is used to sort nodes spatially.
uint32_t GetGridId(const midgard::PointLL& pointll,
                   const midgard::Tiles<midgard::PointLL>& tiling,
                   const uint32_t grid_divisions) {
  // By default grid_divisions is set to 0 to indicate no spatial sorting within a tile
  if (grid_divisions == 0) {
    return 0;
  }

  auto tile_id = tiling.TileId(pointll);
  if (tile_id >= 0) {
    auto base_ll = tiling.Base(tile_id);
    float grid_size = tiling.TileSize() / static_cast<float>(grid_divisions);
    uint32_t row = static_cast<uint32_t>((pointll.lat() - base_ll.lat()) / grid_size);
    uint32_t col = static_cast<uint32_t>((pointll.lng() - base_ll.lng()) / grid_size);
    if (row > grid_divisions || col > grid_divisions) {
      LOG_ERROR("grid row = " + std::to_string(row) + " col = " + std::to_string(col));
      return 0;
    }
    return (row * grid_divisions + col);
  } else {
    LOG_ERROR("GetGridId: Invalid tile id");
    return 0;
  }
}

std::map<GraphId, size_t> GraphBuilder::BuildEdges(const boost::property_tree::ptree& pt,
                                                   const std::string& ways_file,
                                                   const std::string& way_nodes_file,
                                                   const std::string& nodes_file,
                                                   const std::string& edges_file) {
  uint8_t level = TileHierarchy::levels().back().level;
  auto tiling = TileHierarchy::get_tiling(level);
  uint32_t grid_divisions =
      pt.get<unsigned int>("mjolnir.data_processing.grid_divisions_within_tile", 0);
  if (grid_divisions > 0) {
    LOG_INFO("Sort nodes spatially within each tile using nxn grids where n = " +
             std::to_string(grid_divisions));
  } else {
    LOG_INFO("Spatial sorting of nodes within each tile is disabled");
  }

  // Make the edges and nodes in the graph
  ConstructEdges(
      ways_file, way_nodes_file, nodes_file, edges_file,
      [&level](const OSMNode& node) { return TileHierarchy::GetGraphId(node.latlng(), level); },
      [&tiling, &grid_divisions](const OSMNode& node) {
        return GetGridId(node.latlng(), tiling, grid_divisions);
      },
      pt.get<bool>("mjolnir.data_processing.infer_turn_channels", true));

  return SortGraph(nodes_file, edges_file);
}

// Build the graph from the input
void GraphBuilder::Build(const boost::property_tree::ptree& pt,
                         const OSMData& osmdata,
                         const std::string& ways_file,
                         const std::string& way_nodes_file,
                         const std::string& nodes_file,
                         const std::string& edges_file,
                         const std::string& complex_from_restriction_file,
                         const std::string& complex_to_restriction_file,
                         const std::string& linguistic_node_file,
                         const std::map<GraphId, size_t>& tiles) {
  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified. ReclassifyLinks also infers turn channels
  // so we always want to do this unless reclassify_links and infer_turn_channels
  // are both false.
  bool reclassify_links = pt.get<bool>("mjolnir.reclassify_links", true);
  bool infer_turn_channels = pt.get<bool>("mjolnir.data_processing.infer_turn_channels", true);
  if (reclassify_links || infer_turn_channels) {
    ReclassifyLinks(ways_file, nodes_file, edges_file, way_nodes_file, osmdata, reclassify_links,
                    infer_turn_channels);
  } else {
    LOG_WARN("Not reclassifying link graph edges or inferring turn channels");
  }

  // Do not reclassify ferry connection edges if no hierarchies are built. If reclassifying,
  // we use RoadClass::kPrimary (highway classification) as cutoff.
  if (pt.get<bool>("mjolnir.hierarchy", true)) {
    ReclassifyFerryConnections(ways_file, way_nodes_file, nodes_file, edges_file);
  } else {
    LOG_WARN("Not reclassifying ferry connections since no hierarches are being created");
  }

  // Build tiles at the local level. Form connected graph from nodes and edges.
  DataQuality stats;
  unsigned int threads =
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));

  auto tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  // Disable sqlite3 internal memory tracking (results in a high-contention mutex, and we don't care
  // about marginal sqlite memory usage).
  sqlite3_config(SQLITE_CONFIG_MEMSTATUS, false);

  BuildLocalTiles(threads, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                  complex_from_restriction_file, complex_to_restriction_file, linguistic_node_file,
                  tiles, tile_dir, stats, pt);
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
        } else if (tmp[0].find(' ') != std::string::npos &&
                   ref.find(' ') != std::string::npos) { // SR 747 vs OH 747
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

void GraphBuilder::GetPronunciationTokens(
    const UniqueNames& uniquenames,
    const std::vector<std::pair<std::string, bool>>& default_languages,
    const uint32_t ipa_index,
    const uint32_t ipa_lang_index,
    const uint32_t nt_sampa_index,
    const uint32_t nt_sampa_lang_index,
    const uint32_t katakana_index,
    const uint32_t katakana_lang_index,
    const uint32_t jeita_index,
    const uint32_t jeita_lang_index,
    std::vector<std::string>& ipa_tokens,
    std::vector<baldr::Language>& ipa_langs,
    std::vector<std::string>& nt_sampa_tokens,
    std::vector<baldr::Language>& nt_sampa_langs,
    std::vector<std::string>& katakana_tokens,
    std::vector<baldr::Language>& katakana_langs,
    std::vector<std::string>& jeita_tokens,
    std::vector<baldr::Language>& jeita_langs) {

  ipa_tokens.clear();
  nt_sampa_tokens.clear();
  katakana_tokens.clear();
  jeita_tokens.clear();

  ipa_langs.clear();
  nt_sampa_langs.clear();
  katakana_langs.clear();
  jeita_langs.clear();

  if (ipa_index != 0) {
    OSMWay::ProcessNamesPronunciations(uniquenames, default_languages, ipa_index, ipa_lang_index,
                                       ipa_tokens, ipa_langs, false, true);
  }
  if (nt_sampa_index != 0)
    OSMWay::ProcessNamesPronunciations(uniquenames, default_languages, nt_sampa_index,
                                       nt_sampa_lang_index, nt_sampa_tokens, nt_sampa_langs, false,
                                       true);
  if (katakana_index != 0)
    OSMWay::ProcessNamesPronunciations(uniquenames, default_languages, katakana_index,
                                       katakana_lang_index, katakana_tokens, katakana_langs, false,
                                       true);
  if (jeita_index != 0)
    OSMWay::ProcessNamesPronunciations(uniquenames, default_languages, jeita_index, jeita_lang_index,
                                       jeita_tokens, jeita_langs, false, true);
}

void GraphBuilder::AddLanguage(const size_t index,
                               const baldr::Language& lang,
                               std::vector<std::string>& linguistics) {

  if (lang != baldr::Language::kNone) {
    linguistic_text_header_t header{static_cast<uint8_t>(lang),
                                    0,
                                    static_cast<uint8_t>(baldr::PronunciationAlphabet::kNone),
                                    static_cast<uint8_t>(index),
                                    0,
                                    0};
    linguistics.emplace_back(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize);
  }
}

void GraphBuilder::AddPronunciationsWithLang(std::vector<std::string>& pronunciations,
                                             std::map<size_t, baldr::Language>& lang_map,
                                             const baldr::PronunciationAlphabet verbal_type,
                                             const std::vector<std::string>& pronunciation_tokens,
                                             const std::vector<baldr::Language>& pronunciation_langs,
                                             const std::vector<baldr::Language>& token_langs,
                                             const size_t token_size,
                                             const size_t key) {

  auto get_pronunciations = [](const std::vector<std::string>& pronunciation_tokens,
                               const std::vector<baldr::Language>& pronunciation_langs,
                               const std::map<size_t, size_t> indexMap, const size_t key,
                               const baldr::PronunciationAlphabet verbal_type) {
    linguistic_text_header_t header{static_cast<uint8_t>(baldr::Language::kNone),
                                    0,
                                    static_cast<uint8_t>(verbal_type),
                                    static_cast<uint8_t>(key),
                                    0,
                                    0};
    std::vector<std::string> pronunciations;
    for (size_t i = 0; i < pronunciation_tokens.size(); i++) {
      auto& t = pronunciation_tokens[i];

      if (!indexMap.empty()) {
        auto index = indexMap.find(i);
        if (index != indexMap.end())
          header.name_index_ = index->second;
        else
          continue;
      }

      if (t.empty()) { // pronunciation is blank.  just add the lang

        if (pronunciation_langs.empty())
          continue;

        header.language_ = static_cast<uint8_t>(pronunciation_langs.at(i));
        header.length_ = 0;
        header.phonetic_alphabet_ = static_cast<uint8_t>(baldr::PronunciationAlphabet::kNone);

        pronunciations.emplace_back(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize);
      } else {

        header.phonetic_alphabet_ = static_cast<uint8_t>(verbal_type);

        header.length_ = t.size();
        header.language_ =
            (!pronunciation_langs.empty() ? static_cast<uint8_t>(pronunciation_langs.at(i))
                                          : static_cast<uint8_t>(baldr::Language::kNone));

        pronunciations.emplace_back(
            (std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize) + t));
      }
      if (indexMap.empty())
        ++header.name_index_;
    }
    return pronunciations;
  };

  bool process = false, found = false;
  std::map<size_t, size_t> indexMap;
  size_t k = key;

  if ((pronunciation_langs.empty() && token_langs.empty()) ||
      ((!pronunciation_langs.empty() && token_langs.empty()) &&
       (pronunciation_langs.size() <= token_size)))
    process = true;
  else {
    std::pair<std::map<size_t, size_t>::iterator, bool> ret;
    for (auto& token_lang : token_langs) {
      for (size_t j = 0; j < pronunciation_langs.size(); j++) {
        if (token_lang == pronunciation_langs[j]) {
          ret = indexMap.insert(std::make_pair(j, k));
          if (!ret.second) // already used
            continue;
          else {
            k++;
            process = true;
            found = true;
            break;
          }
        }
      }
      if (!found) {
        lang_map.emplace(k, token_lang);
        k++;
      }
      found = false;
    }
  }

  if (process) {
    std::vector<std::string> p =
        get_pronunciations(pronunciation_tokens, pronunciation_langs, indexMap, key, verbal_type);

    pronunciations.insert(pronunciations.end(), p.begin(), p.end());
  }
}

void GraphBuilder::BuildPronunciations(const std::vector<std::string>& ipa_tokens,
                                       const std::vector<baldr::Language>& ipa_langs,
                                       const std::vector<std::string>& nt_sampa_tokens,
                                       const std::vector<baldr::Language>& nt_sampa_langs,
                                       const std::vector<std::string>& katakana_tokens,
                                       const std::vector<baldr::Language>& katakana_langs,
                                       const std::vector<std::string>& jeita_tokens,
                                       const std::vector<baldr::Language>& jeita_langs,
                                       const std::vector<baldr::Language>& token_langs,
                                       const size_t token_size,
                                       const size_t key,
                                       std::vector<std::string>& pronunciations,
                                       std::map<size_t, baldr::Language>& lang_map,
                                       bool add_ipa,
                                       bool add_nt_sampa,
                                       bool add_katakana,
                                       bool add_jeita) {
  if (add_ipa) {
    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kIpa,
                              ipa_tokens, ipa_langs, token_langs, token_size, key);
  }

  if (add_nt_sampa) {

    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kNtSampa,
                              nt_sampa_tokens, nt_sampa_langs, token_langs, token_size, key);
  }

  if (add_katakana) {
    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kKatakana,
                              katakana_tokens, katakana_langs, token_langs, token_size, key);
  }

  if (add_jeita) {
    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kJeita,
                              jeita_tokens, jeita_langs, token_langs, token_size, key);
  }
}

bool GraphBuilder::CreateSignInfoList(
    const OSMNode& node,
    const OSMWay& way,
    const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& pronunciationMap,
    const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& langMap,
    const OSMData& osmdata,
    const std::vector<std::pair<std::string, bool>>& default_languages,
    sequence<OSMNodeLinguistic>& linguistic_node,
    std::vector<SignInfo>& exit_list,
    std::vector<std::string>& linguistics,
    bool fork,
    bool forward,
    bool ramp,
    bool tc) {

  bool has_guide = false;
  std::vector<std::string> ipa_tokens, nt_sampa_tokens, katakana_tokens, jeita_tokens,
      display_ref_tokens;
  std::vector<baldr::Language> ipa_langs, nt_sampa_langs, katakana_langs, jeita_langs;

  bool add_ipa, add_nt_sampa, add_katakana, add_jeita;
  bool is_branch_or_toward = tc || (!ramp && !fork);
  Sign::Type sign_type = Sign::Type::kExitNumber;

  auto get_pronunciation_index = [&pronunciationMap](const uint8_t type, const uint8_t alpha) {
    auto itr = pronunciationMap.find(std::make_pair(type, alpha));
    if (itr != pronunciationMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };

  auto get_lang_index = [&langMap](const uint8_t type, const uint8_t alpha) {
    auto itr = langMap.find(std::make_pair(type, alpha));
    if (itr != langMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };

  auto get_pronunciations =
      [](const UniqueNames& uniquenames,
         const std::vector<std::pair<std::string, bool>>& default_languages, const uint32_t ipa_index,
         const uint32_t ipa_lang_index, const uint32_t nt_sampa_index,
         const uint32_t nt_sampa_lang_index, const uint32_t katakana_index,
         const uint32_t katakana_lang_index, const uint32_t jeita_index,
         const uint32_t jeita_lang_index, std::vector<std::string>& ipa_tokens,
         std::vector<baldr::Language>& ipa_langs, std::vector<std::string>& nt_sampa_tokens,
         std::vector<baldr::Language>& nt_sampa_langs, std::vector<std::string>& katakana_tokens,
         std::vector<baldr::Language>& katakana_langs, std::vector<std::string>& jeita_tokens,
         std::vector<baldr::Language>& jeita_langs, bool& add_ipa, bool& add_nt_sampa,
         bool& add_katakana, bool& add_jeita) {
        GetPronunciationTokens(uniquenames, default_languages, ipa_index, ipa_lang_index,
                               nt_sampa_index, nt_sampa_lang_index, katakana_index,
                               katakana_lang_index, jeita_index, jeita_lang_index, ipa_tokens,
                               ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                               katakana_langs, jeita_tokens, jeita_langs);

        add_ipa = !ipa_tokens.empty();
        add_nt_sampa = !nt_sampa_tokens.empty();
        add_katakana = !katakana_tokens.empty();
        add_jeita = !jeita_tokens.empty();

        return (add_ipa || add_nt_sampa || add_katakana || add_jeita);
      };

  // helper to build SignInfo and update exit_list
  auto add_sign_info = [&](const std::vector<std::string>& refs,
                           const std::vector<baldr::Language>& sign_langs, const Sign::Type& type,
                           const bool is_route_number, const bool phoneme) {
    uint32_t phoneme_count = 0;
    bool add_phoneme = phoneme;
    size_t phoneme_start_index = 0;
    size_t key = exit_list.size();
    std::map<size_t, baldr::Language> lang_map;

    if (add_phoneme) {
      phoneme_start_index = linguistics.size();
      BuildPronunciations(ipa_tokens, ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                          katakana_langs, jeita_tokens, jeita_langs, sign_langs, refs.size(), key,
                          linguistics, lang_map, add_ipa, add_nt_sampa, add_katakana, add_jeita);
      phoneme_count = linguistics.size() - phoneme_start_index;
      if (phoneme_count == 0)
        add_phoneme = false;
    }

    for (size_t i = 0; i < refs.size(); ++i) {
      if (!lang_map.empty()) {
        // only add a language if it has not already been added during pronunciation processing
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(key, itr->second, linguistics);
          phoneme_count = linguistics.size() - phoneme_start_index;
        }
      } else if (i < sign_langs.size() && !add_phoneme) {
        phoneme_start_index = linguistics.size();
        AddLanguage(key, sign_langs.at(i), linguistics);
        phoneme_count = linguistics.size() - phoneme_start_index;
      }
      key++;

      if (phoneme_count == 0) {
        // reset phoneme index if no phonemes were found
        phoneme_start_index = 0;
      }
      exit_list.emplace_back(type, is_route_number, false, (phoneme_count != 0), phoneme_start_index,
                             phoneme_count, refs[i]);
    }
  };

  std::vector<std::string> sign_names;
  std::vector<baldr::Language> sign_langs;

  ////////////////////////////////////////////////////////////////////////////
  // NUMBER
  // Exit sign number
  bool has_phoneme = false;
  const auto ipa = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
  const auto nt_sampa = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
  const auto katakana = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
  const auto jeita = static_cast<uint8_t>(PronunciationAlphabet::kJeita);

  if (way.junction_ref_index() != 0) {
    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.junction_ref_index(), way.junction_ref_lang_index(),
                                   sign_names, sign_langs, false);

    const auto t = static_cast<uint8_t>(OSMLinguistic::Type::kJunctionRef);
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);

  } else if (node.has_ref() && !fork && ramp) {
    OSMNodeLinguistic ling;
    if (node.linguistic_info_index() != 0) {
      ling = linguistic_node.at(node.linguistic_info_index());
    }

    way.ProcessNamesPronunciations(osmdata.node_names, default_languages, node.ref_index(),
                                   ling.ref_lang_index(), sign_names, sign_langs, false);
    has_phoneme =
        get_pronunciations(osmdata.node_names, default_languages, ling.ref_pronunciation_ipa_index(),
                           ling.ref_pronunciation_ipa_lang_index(),
                           ling.ref_pronunciation_nt_sampa_index(),
                           ling.ref_pronunciation_nt_sampa_lang_index(),
                           ling.ref_pronunciation_katakana_index(),
                           ling.ref_pronunciation_katakana_lang_index(),
                           ling.ref_pronunciation_jeita_index(),
                           ling.ref_pronunciation_jeita_lang_index(), ipa_tokens, ipa_langs,
                           nt_sampa_tokens, nt_sampa_langs, katakana_tokens, katakana_langs,
                           jeita_tokens, jeita_langs, add_ipa, add_nt_sampa, add_katakana, add_jeita);
  }

  add_sign_info(sign_names, sign_langs, Sign::Type::kExitNumber, false /* is_route_number */,
                has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  ////////////////////////////////////////////////////////////////////////////
  // BRANCH

  bool has_branch = false;

  // Guide or Exit sign branch refs
  if (way.destination_ref_index() != 0) {
    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.destination_ref_index(), way.destination_ref_lang_index(),
                                   sign_names, sign_langs, false);
    sign_type = is_branch_or_toward ? Sign::Type::kGuideBranch : Sign::Type::kExitBranch;
    has_branch = true;
    has_guide = is_branch_or_toward;

    const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kDestinationRef);
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);
  }
  add_sign_info(sign_names, sign_langs, sign_type, true /* is_route_number */, has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  // Guide or Exit sign branch road names
  if (way.destination_street_index() != 0) {

    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.destination_street_index(),
                                   way.destination_street_lang_index(), sign_names, sign_langs,
                                   false);

    sign_type = is_branch_or_toward ? Sign::Type::kGuideBranch : Sign::Type::kExitBranch;
    has_branch = true;
    has_guide = is_branch_or_toward;

    const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kDestinationStreet);
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);
  }
  add_sign_info(sign_names, sign_langs, sign_type, false /* is_route_number */, has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  ////////////////////////////////////////////////////////////////////////////
  // TOWARD

  bool has_toward = false;

  //-----------------------------
  // Guide or Exit sign toward refs
  if (way.destination_ref_to_index() != 0) {

    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.destination_ref_to_index(),
                                   way.destination_ref_to_lang_index(), sign_names, sign_langs,
                                   false);

    sign_type = is_branch_or_toward ? Sign::Type::kGuideToward : Sign::Type::kExitToward;
    has_toward = true;
    has_guide = is_branch_or_toward;

    const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kDestinationRefTo);
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);
  }
  add_sign_info(sign_names, sign_langs, sign_type, true /* is_route_number */, has_phoneme);

  sign_names.clear();
  sign_langs.clear();

  //-----------------------------
  // Guide or Exit sign toward streets
  if (way.destination_street_to_index() != 0) {

    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.destination_street_to_index(),
                                   way.destination_street_to_lang_index(), sign_names, sign_langs,
                                   false);

    sign_type = is_branch_or_toward ? Sign::Type::kGuideToward : Sign::Type::kExitToward;
    has_toward = true;
    has_guide = is_branch_or_toward;

    const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kDestinationStreetTo);
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);
  }
  add_sign_info(sign_names, sign_langs, sign_type, false /* is_route_number */, has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  //-----------------------------
  // Exit sign toward locations
  if (way.destination_index() != 0 || (forward && way.destination_forward_index() != 0) ||
      (!forward && way.destination_backward_index() != 0)) {

    uint32_t index = way.destination_index() ? way.destination_index()
                                             : (forward ? way.destination_forward_index()
                                                        : way.destination_backward_index());

    // make sure we grab the correct lang based on the index we used above.
    uint32_t lang_index = way.destination_lang_index()
                              ? way.destination_lang_index()
                              : (forward ? way.destination_forward_lang_index()
                                         : way.destination_backward_lang_index());

    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages, index, lang_index,
                                   sign_names, sign_langs, false);

    sign_type = is_branch_or_toward ? Sign::Type::kGuideToward : Sign::Type::kExitToward;
    has_toward = true;
    has_guide = is_branch_or_toward;

    const uint8_t t_destination = static_cast<uint8_t>(OSMLinguistic::Type::kDestination);
    const uint8_t t_destination_forward =
        static_cast<uint8_t>(OSMLinguistic::Type::kDestinationForward);
    const uint8_t t_destination_backward =
        static_cast<uint8_t>(OSMLinguistic::Type::kDestinationBackward);

    index = get_pronunciation_index(t_destination, ipa);
    uint32_t ipa_index = index ? index
                               : (forward ? get_pronunciation_index(t_destination_forward, ipa)
                                          : get_pronunciation_index(t_destination_backward, ipa));

    index = get_pronunciation_index(t_destination, nt_sampa);
    uint32_t nt_sampa_index =
        index ? index
              : (forward ? get_pronunciation_index(t_destination_forward, nt_sampa)
                         : get_pronunciation_index(t_destination_backward, nt_sampa));

    index = get_pronunciation_index(t_destination, katakana);
    uint32_t katakana_index =
        index ? index
              : (forward ? get_pronunciation_index(t_destination_forward, katakana)
                         : get_pronunciation_index(t_destination_backward, katakana));

    index = get_pronunciation_index(t_destination, jeita);
    uint32_t jeita_index = index ? index
                                 : (forward ? get_pronunciation_index(t_destination_forward, jeita)
                                            : get_pronunciation_index(t_destination_backward, jeita));

    index = get_lang_index(t_destination, ipa);
    uint32_t ipa_lang_index = index ? index
                                    : (forward ? get_lang_index(t_destination_forward, ipa)
                                               : get_lang_index(t_destination_backward, ipa));

    index = get_lang_index(t_destination, nt_sampa);
    uint32_t nt_sampa_lang_index = index
                                       ? index
                                       : (forward ? get_lang_index(t_destination_forward, nt_sampa)
                                                  : get_lang_index(t_destination_backward, nt_sampa));

    index = get_lang_index(t_destination, katakana);
    uint32_t katakana_lang_index = index
                                       ? index
                                       : (forward ? get_lang_index(t_destination_forward, katakana)
                                                  : get_lang_index(t_destination_backward, katakana));

    index = get_lang_index(t_destination, jeita);
    uint32_t jeita_lang_index = index ? index
                                      : (forward ? get_lang_index(t_destination_forward, jeita)
                                                 : get_lang_index(t_destination_backward, jeita));

    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages, ipa_index, ipa_lang_index,
                           nt_sampa_index, nt_sampa_lang_index, katakana_index, katakana_lang_index,
                           jeita_index, jeita_lang_index, ipa_tokens, ipa_langs, nt_sampa_tokens,
                           nt_sampa_langs, katakana_tokens, katakana_langs, jeita_tokens, jeita_langs,
                           add_ipa, add_nt_sampa, add_katakana, add_jeita);
  }

  add_sign_info(sign_names, sign_langs, sign_type, false /* is_route_number */, has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  ////////////////////////////////////////////////////////////////////////////
  // Process exit_to only if other branch or toward info does not exist  No pronunciations.
  if (!has_branch && !has_toward) {
    if (node.has_exit_to() && !fork) {
      std::string tmp;
      std::vector<std::string> exit_tos = GetTagTokens(osmdata.node_names.name(node.exit_to_index()));
      for (auto& exit_to : exit_tos) {

        tmp = exit_to;

        boost::algorithm::to_lower(tmp);

        // remove the "To" For example:  US 11;To I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "to ")) {
          exit_list.emplace_back(Sign::Type::kExitToward, false, false, false, 0, 0,
                                 exit_to.substr(3));
          continue;
        }
        // remove the "Toward" For example:  US 11;Toward I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "toward ")) {
          exit_list.emplace_back(Sign::Type::kExitToward, false, false, false, 0, 0,
                                 exit_to.substr(7));
          continue;
        }

        std::size_t found = tmp.find(" to ");

        // Default to kToward if found twice or "toward" found as well; otherwise, <branch> to
        // <toward> For example:  I 95 to I 695
        if (found != std::string::npos && (tmp.find(" to ", found + 4) == std::string::npos &&
                                           tmp.find(" toward ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, false, false, false, 0, 0,
                                 exit_to.substr(0, found));

          exit_list.emplace_back(Sign::Type::kExitToward, false, false, false, 0, 0,
                                 exit_to.substr(found + 4));
          continue;
        }

        found = tmp.find(" toward ");

        // Default to kToward if found twice or "to" found as well; otherwise, <branch> toward
        // <toward> For example:  I 95 to I 695
        if (found != std::string::npos && (tmp.find(" toward ", found + 8) == std::string::npos &&
                                           tmp.find(" to ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, false, false, false, 0, 0,
                                 exit_to.substr(0, found));

          exit_list.emplace_back(Sign::Type::kExitToward, false, false, false, 0, 0,
                                 exit_to.substr(found + 8));
          continue;
        }

        // default to toward.
        exit_list.emplace_back(Sign::Type::kExitToward, false, false, false, 0, 0, exit_to);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // NAME

  // Exit sign name
  if (node.has_name() && !node.named_intersection() && !fork && ramp) {
    sign_type = Sign::Type::kExitName;
    // Get the name from OSMData using the name index

    OSMNodeLinguistic ling;
    if (node.linguistic_info_index() != 0) {
      ling = linguistic_node.at(node.linguistic_info_index());
    }

    way.ProcessNamesPronunciations(osmdata.node_names, default_languages, node.name_index(),
                                   ling.name_lang_index(), sign_names, sign_langs, false);
    has_phoneme =
        get_pronunciations(osmdata.node_names, default_languages, ling.name_pronunciation_ipa_index(),
                           ling.name_pronunciation_ipa_lang_index(),
                           ling.name_pronunciation_nt_sampa_index(),
                           ling.name_pronunciation_nt_sampa_lang_index(),
                           ling.name_pronunciation_katakana_index(),
                           ling.name_pronunciation_katakana_lang_index(),
                           ling.name_pronunciation_jeita_index(),
                           ling.name_pronunciation_jeita_lang_index(), ipa_tokens, ipa_langs,
                           nt_sampa_tokens, nt_sampa_langs, katakana_tokens, katakana_langs,
                           jeita_tokens, jeita_langs, add_ipa, add_nt_sampa, add_katakana, add_jeita);
  }

  add_sign_info(sign_names, sign_langs, sign_type, false /* is_route_number */, has_phoneme);
  sign_names.clear();
  sign_langs.clear();

  //-----------------------------
  // junction:name:*
  //-----------------------------
  if (way.junction_name_index() != 0) {

    way.ProcessNamesPronunciations(osmdata.name_offset_map, default_languages,
                                   way.junction_name_index(), way.junction_name_lang_index(),
                                   sign_names, sign_langs, false);

    const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kJunctionName);
    sign_type = Sign::Type::kExitName;
    has_phoneme =
        get_pronunciations(osmdata.name_offset_map, default_languages,
                           get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                           get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                           get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                           get_pronunciation_index(t, jeita), get_lang_index(t, jeita), ipa_tokens,
                           ipa_langs, nt_sampa_tokens, nt_sampa_langs, katakana_tokens,
                           katakana_langs, jeita_tokens, jeita_langs, add_ipa, add_nt_sampa,
                           add_katakana, add_jeita);
  }

  add_sign_info(sign_names, sign_langs, sign_type, false /* is_route_number */, has_phoneme);

  sign_names.clear();
  sign_langs.clear();

  ////////////////////////////////////////////////////////////////////////////
  // GUIDANCE VIEWS

  // junction
  bool has_guidance_view_jct = false;
  if (forward && way.fwd_jct_base_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.fwd_jct_base_index()), '|');
    // route number set to true for kGuidanceViewJct type means base type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewJunction, true, false, false, 0, 0, name);
    }
    has_guidance_view_jct = true;
  } else if (!forward && way.bwd_jct_base_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.bwd_jct_base_index()), '|');
    // route number set to true for kGuidanceViewJct type means base type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewJunction, true, false, false, 0, 0, name);
    }
    has_guidance_view_jct = true;
  }

  if (forward && way.fwd_jct_overlay_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.fwd_jct_overlay_index()), '|');
    // route number set to false for kGuidanceViewJct type means overlay type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewJunction, false, false, false, 0, 0, name);
    }
    has_guidance_view_jct = true;
  } else if (!forward && way.bwd_jct_overlay_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.bwd_jct_overlay_index()), '|');
    // route number set to false for kGuidanceViewJct type means overlay type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewJunction, false, false, false, 0, 0, name);
    }
    has_guidance_view_jct = true;
  }

  // signboard
  bool has_guidance_view_signboard = false;
  if (forward && way.fwd_signboard_base_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.fwd_signboard_base_index()), '|');
    // route number set to true for kGuidanceViewSignboard type means base type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewSignboard, true, false, false, 0, 0, name);
    }
    has_guidance_view_signboard = true;
  } else if (!forward && way.bwd_signboard_base_index() > 0) {
    std::vector<std::string> names =
        GetTagTokens(osmdata.name_offset_map.name(way.bwd_signboard_base_index()), '|');
    // route number set to true for kGuidanceViewSignboard type means base type
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kGuidanceViewSignboard, true, false, false, 0, 0, name);
    }
    has_guidance_view_signboard = true;
  }

  // we have to sort because we need the key/indexes for phonemes
  std::stable_sort(exit_list.begin(), exit_list.end());

  return (has_guide || has_guidance_view_jct || has_guidance_view_signboard);
}

} // namespace valhalla::mjolnir
