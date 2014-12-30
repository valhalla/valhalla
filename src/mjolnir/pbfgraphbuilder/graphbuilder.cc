#include <boost/functional/hash.hpp>
#include <future>

#include "graphbuilder.h"

#include <valhalla/baldr/graphid.h>
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

#include <utility>
#include <algorithm>
#include <string>
#include <thread>
#include <memory>
#include <valhalla/midgard/aabbll.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

  const size_t kAsyncBatchSize = 64;

}

namespace valhalla {
namespace mjolnir {

// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;

GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt,
                           const std::string& input_file)
    : maxosmid_(0),
      relation_count_(0),
      node_count_(0),
      input_file_(input_file),
      tile_hierarchy_(pt),
      osmnodeids_(kMaxOSMNodeId) {

  // Initialize Lua based on config
  LuaInit(pt.get<std::string>("tagtransform.node_script"),
          pt.get<std::string>("tagtransform.node_function"),
          pt.get<std::string>("tagtransform.way_script"),
          pt.get<std::string>("tagtransform.way_function"));
}

void GraphBuilder::Build() {
  // Parse the pbf ways. Find all node Ids needed.
  std::cout << "Parse PBF ways to find all OSM Node Ids needed" << std::endl;
  CanalTP::read_osm_pbf(input_file_, *this, CanalTP::Interest::WAYS);
  std::cout << "Way count = " << ways_.size() << std::endl;

  // Step 2 - parse nodes and relations
  std::cout << "Parse PBF nodes and relations" << std::endl;
  CanalTP::read_osm_pbf(input_file_, *this, static_cast<CanalTP::Interest>(CanalTP::Interest::NODES | CanalTP::Interest::RELATIONS));
  std::cout << "Max OSM Node ID = " << maxosmid_ << std::endl;
  std::cout << relation_count_ << " relations" << std::endl;
  std::cout << "Ways use " << nodes_.size() << " nodes out of  " << node_count_ << " total nodes" << std::endl;

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
  BuildLocalTiles(tl->second.level);
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
  if (osmid > maxosmid_)
    maxosmid_ = osmid;

  node_count_++;

  // Check if it is in the list of nodes used by ways
  if (!osmnodeids_.IsUsed(osmid)) {
    return;
  }

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
    std::cout << "Processed " << nodecount << " nodes on ways" << std::endl;
  }
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {
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

  // Add list of OSM Node Ids we need
  for (const auto nodeid : refs) {
    osmnodeids_.set(nodeid);
  }
}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                                     const CanalTP::References & /*refs*/) {
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

void GraphBuilder::TileNodes(const float tilesize, const uint8_t level) {
  std::cout << "Tile nodes..." << std::endl;

  // Get number of tiles and reserve space for them
  // < 30% of the earth is land and most roads are on land, even less than that even has roads
  Tiles tiles(AABBLL(-90.0f, -180.0f, 90.0f, 180.0f), tilesize);
  tilednodes_.reserve(tiles.TileCount() * .3f);

  // Iterate through all OSM nodes and assign GraphIds
  for (auto& node : nodes_) {
    // Skip any nodes that have no edges
    if (node.second.edge_count() == 0) {
      //std::cout << "Node with no edges" << std::endl;
      continue;
    }

    // Put the node into the tile
    GraphId id = tile_hierarchy_.GetGraphId(node.second.latlng(), level);
    std::vector<uint64_t>& tile = tilednodes_[id];
    tile.emplace_back(node.first);

    // Set the GraphId for this OSM node.
    node.second.set_graphid(GraphId(id.tileid(), id.level(), tile.size() - 1));
  }
  std::cout << "Done TileNodes" << std::endl;
}

namespace {
struct NodePairHasher {
  std::size_t operator()(const node_pair& k) const {
    std::size_t seed = 13;
    boost::hash_combine(seed, id_hasher(k.first));
    boost::hash_combine(seed, id_hasher(k.second));
    return seed;
  }
  //function to hash each id
  std::hash<valhalla::baldr::GraphId> id_hasher;
};

node_pair ComputeNodePair(const baldr::GraphId& nodea,
                          const baldr::GraphId& nodeb) {
  if (nodea < nodeb)
    return std::make_pair(nodea, nodeb);
  return std::make_pair(nodeb, nodea);
}

uint32_t GetOpposingIndex(const uint64_t endnode, const uint64_t startnode, const std::unordered_map<uint64_t, OSMNode>& nodes, const std::vector<Edge>& edges) {
  uint32_t n = 0;
  auto node = nodes.find(endnode);
  if(node != nodes.end()) {
    for (const auto& edgeindex : node->second.edges()) {
      if ((edges[edgeindex].sourcenode_ == endnode &&
           edges[edgeindex].targetnode_ == startnode) ||
          (edges[edgeindex].targetnode_ == endnode &&
           edges[edgeindex].sourcenode_ == startnode)) {
        return n;
      }
      n++;
    }
  }
  std::cout << "ERROR Opposing directed edge not found!" << std::endl;
  return 31;
}

void BuildTileRange(std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start,
                    std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_end,
                    const std::unordered_map<uint64_t, OSMNode>& nodes, const std::vector<OSMWay>& ways,
                    const std::vector<Edge>& edges, const std::string& outdir,  std::promise<size_t>& result) {

  std::cout << "Thread " << std::this_thread::get_id() << " started" << std::endl;

  // A place to keep information about what was done
  size_t written = 0;

  // For each tile
  for(; tile_start != tile_end; ++tile_start) {
    try {
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
      uint32_t directededgecount = 0;
      for (const auto& osmnodeid : tile_start->second) {
        const OSMNode& node = nodes.find(osmnodeid)->second; //TODO: check validity?
        NodeInfoBuilder nodebuilder;
        nodebuilder.set_latlng(node.latlng());

        // Set the index of the first outbound edge within the tile.
        nodebuilder.set_edge_index(directededgecount);
        nodebuilder.set_edge_count(node.edge_count());

        // Set up directed edges
        std::vector<DirectedEdgeBuilder> directededges;
        directededgecount += node.edge_count();
        for (auto edgeindex : node.edges()) {
          DirectedEdgeBuilder directededge;
          const Edge& edge = edges[edgeindex];

          // Compute length from the latlngs.
          float length = node.latlng().Length(edge.latlngs_);
          directededge.set_length(length);

          // Get the way information and set attributes
          const OSMWay &w = ways[edge.wayindex_];

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

          // TODO - if sorting of directed edges occurs after this will need to
          // remove this code and do elsewhere.

          // Assign nodes and determine orientation along the edge (forward
          // or reverse between the 2 nodes)
          const GraphId& nodea = nodes.find(edge.sourcenode_)->second.graphid(); //TODO: check validity?
          if (!nodea.Is_Valid()) {
            std::cout << "ERROR: Node A: OSMID = " << edge.sourcenode_ <<
                " GraphID is not valid" << std::endl;
          }
          const GraphId& nodeb = nodes.find(edge.targetnode_)->second.graphid(); //TODO: check validity?
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

            // Set the opposing edge offset at the end node of this directed edge.
            directededge.set_opp_index(GetOpposingIndex(edge.targetnode_, edge.sourcenode_, nodes, edges));
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

            // Set opposing edge index at the end node of this directed edge.
            directededge.set_opp_index(GetOpposingIndex(edge.sourcenode_, edge.targetnode_, nodes, edges));
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
                text_list.emplace_back(name);

                // Add name offset to list
                street_name_offset_list.emplace_back(text_list_offset);

                // Add name/offset pair to map
                text_offset_map.insert(std::make_pair(name, text_list_offset));

                // Update text offset value to length of string plus null terminator
                text_list_offset += (name.length() + 1);
              } else {
                // Add existing offset to list
                street_name_offset_list.emplace_back(existing_text_offset->second);
              }
            }
            edgeinfo.set_street_name_offset_list(street_name_offset_list);

            // TODO - other attributes

            // Add to the map
            edge_offset_map.insert(
                std::make_pair(node_pair_item, edge_info_offset));

            // Add to the list
            edgeinfo_list.emplace_back(edgeinfo);

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
          directededges.emplace_back(directededge);
        }

        // Add information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);
      }

      graphtile.SetEdgeInfoAndSize(edgeinfo_list, edge_info_offset);
      graphtile.SetTextListAndSize(text_list, text_list_offset);

      // Write the actual tile to disk
      graphtile.StoreTileData(outdir, tile_start->first);

      // Made a tile
      std::cout << "Thread " << std::this_thread::get_id() << " wrote tile " << tile_start->first << ": " << graphtile.size() << " bytes" << std::endl;
      written += graphtile.size();
    }// Whatever happens in Vagas..
    catch(std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      std::cout << "Thread " << std::this_thread::get_id() << " failed tile " << tile_start->first << ": " << e.what() << std::endl;
      return;
    }
  }
  // Let the main thread how this thread faired
  result.set_value(written);
}

//TODO: make this an option
constexpr size_t kThreadCount = 4;

}

// Build tiles for the local graph hierarchy (basically
void GraphBuilder::BuildLocalTiles(const uint8_t level) const {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(kThreadCount);
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::vector<std::promise<size_t> > results(kThreadCount);
  // Divvy up the work
  size_t floor = tilednodes_.size() / kThreadCount;
  size_t at_ceiling = tilednodes_.size() - (kThreadCount * floor);
  std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start, tile_end = tilednodes_.begin();
  std::cout << tilednodes_.size() << " tiles\n";
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(
      new std::thread(BuildTileRange, tile_start, tile_end, nodes_, ways_,
                      edges_, tile_hierarchy_.tile_dir(), std::ref(results[i]))
    );
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto pass_fail = result.get_future().get();
      //TODO: print out stats about how many tiles or bytes were written by the thread?
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }


  /*// Wait for results to come back from the threads, logging what happened and removing the result
  auto process_result = [] (std::unordered_map<GraphId, std::future<size_t> >& results) {
    // For each task
    for (std::unordered_map<GraphId, std::future<size_t> >::iterator result = results.begin(); result != results.end(); ++result) {
      try {
        // Block until the result is ready
        auto status = result->second.wait_for(std::chrono::microseconds(1));
        // If it was ready
        if (status == std::future_status::ready) {
          size_t bytes = result->second.get();
          std::cout << "Wrote tile " << result->first << ": " << bytes << " bytes" << std::endl;
          results.erase(result);
          break;
        }
      }
      catch (const std::exception& e) {
        //TODO: log and rethrow
        std::cout << "Filed tile " << result->first << ": " << e.what() << std::endl;
        results.erase(result);
        break;
      }
    }
  };

  // Process each tile asynchronously
  std::unordered_map<GraphId, std::future<size_t> > results(8);
  for (const auto& tile : tilednodes_) {
    // If we can squeeze this task in
    if(results.size() < kMaxInFlightTasks) {
      // Build the tile
      results.emplace(tile.first, std::async(std::launch::async, BuildTile, tile, nodes_, ways_, edges_, tile_hierarchy_.tile_dir()));
      continue;
    }

    // If We already have too many in flight wait for some to finish
    while(results.size() >= kMaxInFlightTasks) {
      // Try to get some results
      process_result(results);
    }
  }

  // We're done adding tasks but there may be more results
  while(results.size())
    process_result(results);*/
}


}
}
