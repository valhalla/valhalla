#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "graphbuilder.h"

#include <valhalla/baldr/graphid.h>
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

#include <utility>
#include <algorithm>
#include <string>
#include <valhalla/midgard/aabbll.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt,
                           const std::string& input_file)
    : skippednodes_(0),
      relation_count_(0),
      node_count_(0),
      input_file_(input_file),
      tile_hierarchy_(pt) {

  // Initialize Lua based on config
  LuaInit(pt.get<std::string>("tagtransform.node_script"),
          pt.get<std::string>("tagtransform.node_function"),
          pt.get<std::string>("tagtransform.way_script"),
          pt.get<std::string>("tagtransform.way_function"));
}

void GraphBuilder::Build() {
  // Parse the pbf ways. Find all node Ids needed.
  std::cout << "Parse PBF ways to find all OSM Node Ids needed" << std::endl;
  preprocess_ = true;
  CanalTP::read_osm_pbf(input_file_, *this);
  std::cout << "Way count = " << ways_.size() << std::endl;
// TODO - add later
//  std::cout << "Total node count = " << node_count_ << " Ways use "
//            << osmnodeids_.nonempty() << " nodes" << std::endl;
//  std::cout << "NodeId memory use " << osmnodeids_.memory_use() << std::endl;

  // Step 2 - parse nodes and relations
  std::cout << "Parse PBF nodes and relations" << std::endl;
  preprocess_ = false;
  CanalTP::read_osm_pbf(input_file_, *this);
  std::cout << relation_count_ << " relations" << std::endl;
  std::cout << "Skipped " << skippednodes_ << " nodes" << std::endl;

  // Compute node use counts
  SetNodeUses();

  // Construct edges
  ConstructEdges();

  // Remove unused node (maybe this can recover memory?)
  RemoveUnusedNodes();

  // Tile the nodes
  //TODO: generate more than just the most detailed level
  const auto& tl = tile_hierarchy_.levels().rbegin();
  TileNodes(tl->second.tiles.TileSize(), tl->second.level);

  // Iterate through edges - tile the end nodes to create connected graph
  BuildLocalTiles(tile_hierarchy_.tile_dir(), tl->second.level);
}

// Initialize Lua tag transformations
void GraphBuilder::LuaInit(const std::string& nodetagtransformscript,
                           const std::string& nodetagtransformfunction,
                           const std::string& waytagtransformscript,
                           const std::string& waytagtransformfunction) {
  lua_.SetLuaNodeScript(nodetagtransformscript);
  lua_.SetLuaNodeFunc(nodetagtransformfunction);
  lua_.SetLuaWayScript(waytagtransformscript);
  lua_.SetLuaWayFunc(waytagtransformfunction);
  lua_.OpenLib();
}

void GraphBuilder::node_callback(uint64_t osmid, double lng, double lat,
                                 const Tags &tags) {
  // Skip on preprocess step
  if (preprocess_) {
    node_count_++;
    return;
  }

  // Check if it is in the list of nodes used by ways
/** TODO - add later
  if (!osmnodeids_[osmid]) {
    skippednodes_++;
    return;
  }
  */

  // Get tags
  Tags results = lua_.TransformInLua(false, tags);
  if (results.size() == 0)
    return;

  // Create a new node and set its attributes
  OSMNode n(lat, lng);
  for (const auto& tag : results) {
    if (tag.first == "exit_to") {
      bool hasTag = (tag.second.length() ? true : false);
      n.set_exit_to(hasTag);
      if (hasTag)
        map_exit_to_[osmid] = tag.second;
    }
    else if (tag.first == "ref") {
      bool hasTag = (tag.second.length() ? true : false);
      n.set_ref(hasTag);
      if (hasTag)
        map_ref_[osmid] = tag.second;
    }
    else if (tag.first == "gate")
      n.set_gate((tag.second == "true" ? true : false));
    else if (tag.first == "bollard")
      n.set_bollard((tag.second == "true" ? true : false));
    else if (tag.first == "modes_mask")
      n.set_modes_mask(std::stoi(tag.second));

    //  if (osmid == 2385249)
    //    std::cout << "key: " << tag.first << " value: " << tag.second << std::endl;
  }

  // Add to the node map;
  nodes_.emplace(osmid, n);

  size_t nodecount = nodes_.size();
  if (nodecount % 1000000 == 0) {
    std::cout << nodecount << " nodes on ways and " << skippednodes_
              << " skipped" << std::endl;
  }
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {
  // Ways are processed in the first iteration. Skip on 2nd iteration.
  if (!preprocess_)
    return;

  // Do not add ways with < 2 nodes. Log error or add to a problem list
  // TODO - find out if we do need these, why they exist...
  if (refs.size() < 2) {
    //    std::cout << "ERROR - way " << osmid << " with < 2 nodes" << std::endl;
    return;
  }

  // Transform tags. If no results that means the way does not have tags
  // suitable for use in routing.
  Tags results = lua_.TransformInLua(true, tags);
  if (results.size() == 0) {
    return;
  }

  // Add the node reference list to the way
  OSMWay w(osmid);
  w.set_nodes(refs);

  // Process tags
  for (const auto& tag : results) {

    if (tag.first == "road_class") {

      RoadClass roadclass = (RoadClass) std::stoi(tag.second);

      switch (roadclass) {

        case RoadClass::kMotorway:
          w.set_road_class(RoadClass::kMotorway);
          break;
        case RoadClass::kTrunk:
          w.set_road_class(RoadClass::kTrunk);
          break;
        case RoadClass::kPrimary:
          w.set_road_class(RoadClass::kPrimary);
          break;
        case RoadClass::kTertiaryUnclassified:
          w.set_road_class(RoadClass::kTertiaryUnclassified);
          break;
        case RoadClass::kResidential:
          w.set_road_class(RoadClass::kResidential);
          break;
        case RoadClass::kService:
          w.set_road_class(RoadClass::kService);
          break;
        case RoadClass::kTrack:
          w.set_road_class(RoadClass::kTrack);
          break;
        default:
          w.set_road_class(RoadClass::kOther);
          break;
      }
    }

    else if (tag.first == "auto_forward")
      w.set_auto_forward(tag.second == "true" ? true : false);
    else if (tag.first == "bike_forward")
      w.set_bike_forward(tag.second == "true" ? true : false);
    else if (tag.first == "auto_backward")
      w.set_auto_backward(tag.second == "true" ? true : false);
    else if (tag.first == "bike_backward")
      w.set_bike_backward(tag.second == "true" ? true : false);
    else if (tag.first == "pedestrian")
      w.set_pedestrian(tag.second == "true" ? true : false);

    else if (tag.first == "private")
      w.set_destination_only(tag.second == "true" ? true : false);

    else if (tag.first == "use") {

      Use use = (Use) std::stoi(tag.second);

      switch (use) {

        case Use::kNone:
          w.set_use(Use::kNone);
          break;
        case Use::kCycleway:
          w.set_use(Use::kCycleway);
          break;
        case Use::kParkingAisle:
          w.set_use(Use::kParkingAisle);
          break;
        case Use::kDriveway:
          w.set_use(Use::kDriveway);
          break;
        case Use::kAlley:
          w.set_use(Use::kAlley);
          break;
        case Use::kEmergencyAccess:
          w.set_use(Use::kEmergencyAccess);
          break;
        case Use::kDriveThru:
          w.set_use(Use::kDriveThru);
          break;
        case Use::kSteps:
          w.set_use(Use::kSteps);
          break;
        case Use::kOther:
          w.set_use(Use::kOther);
          break;
        default:
          w.set_use(Use::kNone);
          break;
      }
    }

    else if (tag.first == "no_thru_traffic_")
      w.set_no_thru_traffic(tag.second == "true" ? true : false);
    else if (tag.first == "oneway")
      w.set_oneway(tag.second == "true" ? true : false);
    else if (tag.first == "roundabout")
      w.set_roundabout(tag.second == "true" ? true : false);
    else if (tag.first == "link")
      w.set_link(tag.second == "true" ? true : false);
    else if (tag.first == "ferry")
      w.set_ferry(tag.second == "true" ? true : false);
    else if (tag.first == "rail")
      w.set_rail(tag.second == "true" ? true : false);

    else if (tag.first == "name")
      w.set_name(tag.second);
    else if (tag.first == "name:en")
      w.set_name_en(tag.second);
    else if (tag.first == "alt_name")
      w.set_alt_name(tag.second);
    else if (tag.first == "official_name")
      w.set_official_name(tag.second);

    else if (tag.first == "speed")
      w.set_speed(std::stof(tag.second));

    else if (tag.first == "ref")
      w.set_ref(tag.second);
    else if (tag.first == "int_ref")
      w.set_int_ref(tag.second);

    else if (tag.first == "surface")
      w.set_surface(tag.second == "true" ? true : false);

    else if (tag.first == "lanes")
      w.set_lanes(std::stoi(tag.second));

    else if (tag.first == "tunnel")
      w.set_tunnel(tag.second == "true" ? true : false);
    else if (tag.first == "toll")
      w.set_toll(tag.second == "true" ? true : false);
    else if (tag.first == "bridge")
      w.set_bridge(tag.second == "true" ? true : false);

    else if (tag.first == "bike_network_mask")
      w.set_bike_network(std::stoi(tag.second));
    else if (tag.first == "bike_national_ref")
      w.set_bike_national_ref(tag.second);
    else if (tag.first == "bike_regional_ref")
      w.set_bike_regional_ref(tag.second);
    else if (tag.first == "bike_local_ref")
      w.set_bike_local_ref(tag.second);

    else if (tag.first == "destination")
      w.set_destination(tag.second);
    else if (tag.first == "destination:ref")
      w.set_destination_ref(tag.second);
    else if (tag.first == "destination:ref:to")
      w.set_destination_ref_to(tag.second);
    else if (tag.first == "junction_ref")
      w.set_junction_ref(tag.second);

    // if (osmid == 368034)   //http://www.openstreetmap.org/way/368034#map=18/39.82859/-75.38610
    //   std::cout << "key: " << tag.first << " value: " << tag.second << std::endl;
  }

  // Add the way to the list
  ways_.emplace_back(w);
//  if (ways_.size() % 1000000 == 0) std::cout << ways_.size() <<
//      " ways parsed" << std::endl;

  /** TODO - add later
  // Add list of OSM Node Ids we need
  for (const auto nodeid : refs) {
    osmnodeids_.set(nodeid);
  }
  */
}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                                     const CanalTP::References & /*refs*/) {
  // Relations are processed in the seconds iteration. Skip on 1st iteration.
  if (preprocess_)
    return;
  relation_count_++;
}

// Once all the ways and nodes are read, we compute how many times a node
// is used. This allows us to detect intersections (nodes in a graph).
void GraphBuilder::SetNodeUses() {
  // Iterate through the ways
  for (const auto& way : ways_) {
    // Iterate through the nodes that make up the way
    for (const auto& node : way.nodes()) {
      nodes_[node].IncrementUses();
    }

    // Make sure that the first and last node of the way have use count of
    // at least 2 (so they will not get removed later).
    nodes_[way.nodes().front()].IncrementUses();
    nodes_[way.nodes().back()].IncrementUses();
  }
}

// Construct edges in the graph.
// TODO - memory use? Delete temporary edges?
// TODO - compare logic to example_routing app. to see why the edge
// count differs.
void GraphBuilder::ConstructEdges() {
  // Iterate through the OSM ways
  uint32_t edgeindex = 0;
  uint32_t wayindex = 0;
  uint64_t currentid;
  for (const auto& way : ways_) {
    // Start an edge at the first node of the way and add the
    // edge index to the node
    currentid = way.nodes()[0];
    OSMNode& node = nodes_[currentid];
    Edge* edge = new Edge(currentid, wayindex, node.latlng());
    node.AddEdge(edgeindex);

    // Iterate through the nodes of the way and add lat,lng to the current
    // way until a node with > 1 uses is found.
    for (size_t i = 1, n = way.node_count(); i < n; i++) {
      // Add the node lat,lng to the edge shape.
      // TODO - not sure why I had to use a different OSMNode declaration here?
      // But if I use node from above I get errors!
      currentid = way.nodes()[i];
      OSMNode& nd = nodes_[currentid];
      edge->AddLL(nd.latlng());

      // If a node is used more than once, it is an intersection or the end
      // of the way, hence it's a node of the road network graph
      if (nd.uses() > 1) {
        // End the current edge and add its edge index to the node
        edge->targetnode_ = currentid;
        nd.AddEdge(edgeindex);

        // Add the edge to the list of edges
        edges_.emplace_back(*edge);
        edgeindex++;

        // Start a new edge if this is not the last node in the way
        if (i < n-1) {
          edge = new Edge(currentid, wayindex, node.latlng());
          nd.AddEdge(edgeindex);
        }
      }
    }
    wayindex++;
  }
  std::cout << "Constructed " << edges_.size() << " edges" << std::endl;
}

// Iterate through the node map and remove any nodes that are not part of
// the graph (they have been converted to part of the edge shape)
void GraphBuilder::RemoveUnusedNodes() {
  std::cout << "Remove unused nodes" << std::endl;
  node_map_type::iterator itr = nodes_.begin();
  while (itr != nodes_.end()) {
    if (itr->second.uses() < 2) {
      node_map_type::iterator to_erase = itr;
      ++itr;
      nodes_.erase(to_erase);
    } else {
      ++itr;
    }
  }
  std::cout << "Removed unused nodes; remaining node count = "
            << nodes_.size() << std::endl;
}

void GraphBuilder::TileNodes(const float tilesize, const unsigned int level) {
  std::cout << "Tile nodes..." << std::endl;
  int32_t tileid;
  size_t n;
  Tiles tiles(AABBLL(-90.0f, -180.0f, 90.0f, 180.0f), tilesize);

  // Get number of tiles and resize the tilednodes vector
  unsigned int tilecount = tiles.TileCount();
  tilednodes_.resize(tilecount);

  // Iterate through all OSM nodes and assign GraphIds
  for (auto& node : nodes_) {
    // Skip any nodes that have no edges
    if (node.second.edge_count() == 0) {
      //std::cout << "Node with no edges" << std::endl;
      continue;
    }

    // Get tile Id
    tileid = tiles.TileId(node.second.latlng());
    if (tileid < 0) {
      // TODO: error!
      std::cout << "Tile error" << std::endl;
    }

    // Get the number of nodes currently in the tile.
    // Set the GraphId for this OSM node.
    n = tilednodes_[tileid].size();
    node.second.set_graphid(GraphId(static_cast<uint32_t>(tileid), level,
                                    static_cast<uint64_t>(n)));

    // Add this OSM node to the list of nodes for this tile
    tilednodes_[tileid].push_back(node.first);
  }
  std::cout << "Done TileNodes" << std::endl;
}

namespace {
struct NodePairHasher {
  std::size_t operator()(const node_pair& k) const {
    std::size_t seed = 13;
    boost::hash_combine(seed, k.first.HashCode());
    boost::hash_combine(seed, k.second.HashCode());
    return seed;
  }
};
}

// Build tiles for the local graph hierarchy (basically
void GraphBuilder::BuildLocalTiles(const std::string& outputdir,
                                   const unsigned int level) {
  // Iterate through the tiles
  uint32_t tileid = 0;
  for (const auto& tile : tilednodes_) {
    // Skip empty tiles
    if (tile.size() == 0) {
      tileid++;
      continue;
    }

    GraphTileBuilder graphtile;

    // Edge info offset and map
    size_t edge_info_offset = 0;
    std::unordered_map<node_pair, size_t, NodePairHasher> edge_offset_map;

    // The edgeinfo list
    std::vector<EdgeInfoBuilder> edgeinfo_list;

    // Text list offset and map
    size_t text_list_offset = 0;
    std::unordered_map<std::string, size_t> text_offset_map;

    // Text list
    std::vector<std::string> text_list;

    // Iterate through the nodes
    uint32_t id = 0;
    uint32_t directededgecount = 0;
    for (auto osmnodeid : tile) {
      OSMNode& node = nodes_[osmnodeid];
      NodeInfoBuilder nodebuilder;
      nodebuilder.set_latlng(node.latlng());

      // Set the index of the first outbound edge within the tile.
      nodebuilder.set_edge_index(directededgecount);
      nodebuilder.set_edge_count(node.edge_count());

      // Set up directed edges
      std::vector<DirectedEdgeBuilder> directededges;
      for (auto edgeindex : node.edges()) {
        DirectedEdgeBuilder directededge;
        const Edge& edge = edges_[edgeindex];

        // Compute length from the latlngs.
        float length = node.latlng().Length(edge.latlngs_);
        directededge.set_length(length);

        // Get the way information and set attributes
        OSMWay &w = ways_[edge.wayindex_];

        directededge.set_importance(w.road_class());
        directededge.set_use(w.use());
        directededge.set_link(w.link());
        directededge.set_speed(w.speed());    // KPH
        directededge.set_ferry(w.ferry());
        directededge.set_railferry(w.rail());
        directededge.set_toll(w.toll());
        directededge.set_dest_only(w.destination_only());
        directededge.set_unpaved(w.surface());
        directededge.set_tunnel(w.tunnel());
        directededge.set_roundabout(w.roundabout());
        directededge.set_bridge(w.bridge());
        directededge.set_bikenetwork(w.bike_network());

        //http://www.openstreetmap.org/way/368034#map=18/39.82859/-75.38610
        /*  if (w.osmwayid_ == 368034)
         {
         std::cout << edge.sourcenode_ << " "
         << edge.targetnode_ << " "
         << w.auto_forward_ << " "
         << w.pedestrian_ << " "
         << w.bike_forward_ << " "
         << w.auto_backward_ << " "
         << w.pedestrian_ << " "
         << w.bike_backward_ << std::endl;

         }*/

        // Assign nodes and determine orientation along the edge (forward
        // or reverse between the 2 nodes)
        const GraphId& nodea = nodes_[edge.sourcenode_].graphid();
        if (!nodea.Is_Valid()) {
          std::cout << "ERROR: Node A: OSMID = " << edge.sourcenode_ <<
              " GraphID is not valid" << std::endl;
        }
        const GraphId& nodeb = nodes_[edge.targetnode_].graphid();
        if (!nodeb.Is_Valid()) {
          std::cout << "Node B: OSMID = " << edge.targetnode_ <<
            " GraphID is not valid" << std::endl;
        }
        if (edge.sourcenode_ == osmnodeid) {
          // Edge traversed in forward direction
          directededge.set_caraccess(true, false, w.auto_forward());
          directededge.set_pedestrianaccess(true, false, w.pedestrian());
          directededge.set_bicycleaccess(true, false, w.bike_forward());

          directededge.set_caraccess(false, true, w.auto_backward());
          directededge.set_pedestrianaccess(false, true, w.pedestrian());
          directededge.set_bicycleaccess(false, true, w.bike_backward());

          // Set end node to the target (end) node
          directededge.set_endnode(nodeb);
        } else if (edge.targetnode_ == osmnodeid) {
          // Reverse direction.  Reverse the access logic and end node
          directededge.set_caraccess(true, false, w.auto_backward());
          directededge.set_pedestrianaccess(true, false, w.pedestrian());
          directededge.set_bicycleaccess(true, false, w.bike_backward());

          directededge.set_caraccess(false, true, w.auto_forward());
          directededge.set_pedestrianaccess(false, true, w.pedestrian());
          directededge.set_bicycleaccess(false, true, w.bike_forward());

          // Set end node to the source (start) node
          directededge.set_endnode(nodea);
        } else {
          // ERROR!!!
          std::cout << "ERROR: WayID = " << w.way_id() << " Edge Index = " <<
              edgeindex << " Edge nodes " <<
              edge.sourcenode_ << " and " << edge.targetnode_ <<
              " do not match the OSM node Id" <<
              osmnodeid << std::endl;
        }

        // Check if we need to add edge info
        auto node_pair_item = ComputeNodePair(nodea, nodeb);
        auto existing_edge_offset_item = edge_offset_map.find(node_pair_item);

        // Add new edge info
        if (existing_edge_offset_item == edge_offset_map.end()) {
          EdgeInfoBuilder edgeinfo;
          edgeinfo.set_nodea(nodea);
          edgeinfo.set_nodeb(nodeb);
          // TODO - shape encode
          edgeinfo.set_shape(edge.latlngs_);
          // TODO - names
          std::vector<std::string> names = w.GetNames();
          std::vector<size_t> street_name_offset_list;

          for (const auto& name : names) {
            if (name.empty()) {
              continue;
            }

            auto existing_text_offset = text_offset_map.find(name);
            // Add if not found
            if (existing_text_offset == text_offset_map.end()) {
              // Add name to text list
              text_list.push_back(name);

              // Add name offset to list
              street_name_offset_list.push_back(text_list_offset);

              // Add name/offset pair to map
              text_offset_map.insert(std::make_pair(name, text_list_offset));

              // Update text offset value to length of string plus null terminator
              text_list_offset += (name.length() + 1);
            } else {
              // Add existing offset to list
              street_name_offset_list.push_back(existing_text_offset->second);
            }
          }
          edgeinfo.set_street_name_offset_list(street_name_offset_list);

          // TODO - other attributes

          // Add to the map
          edge_offset_map.insert(
              std::make_pair(node_pair_item, edge_info_offset));

          // Add to the list
          edgeinfo_list.push_back(edgeinfo);

          // Set edge offset within the corresponding directed edge
          directededge.set_edgedataoffset(edge_info_offset);

          // Update edge offset for next item
          edge_info_offset += edgeinfo.SizeOf();

        }
        // Update directed edge with existing edge offset
        else {
          directededge.set_edgedataoffset(existing_edge_offset_item->second);
        }

        // Add to the list
        directededges.push_back(directededge);
        directededgecount++;
      }

      // Add information to the tile
      graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);
      id++;
    }

    graphtile.SetEdgeInfoAndSize(edgeinfo_list, edge_info_offset);
    graphtile.SetTextListAndSize(text_list, text_list_offset);

    // File name for tile
    GraphId graphid(tileid, level, 0);
    graphtile.StoreTileData(outputdir, graphid);

    tileid++;
  }
}

node_pair GraphBuilder::ComputeNodePair(const baldr::GraphId& nodea,
                                        const baldr::GraphId& nodeb) const {
  if (nodea < nodeb)
    return std::make_pair(nodea, nodeb);
  return std::make_pair(nodeb, nodea);
}

}
}
