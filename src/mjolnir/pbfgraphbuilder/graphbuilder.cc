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
    : relation_count_(0),
      node_count_(0),
      input_file_(input_file),
      tile_hierarchy_(pt) {

  //grab out the lua config
  LuaInit(pt.get<std::string>("tagtransform.node_script"),
          pt.get<std::string>("tagtransform.node_function"),
          pt.get<std::string>("tagtransform.way_script"),
          pt.get<std::string>("tagtransform.way_function"));
}

void GraphBuilder::Build() {
  //parse the pbf
  CanalTP::read_osm_pbf(input_file_, *this);
  PrintCounts();

  // Compute node use counts
  SetNodeUses();

  // Construct edges
  ConstructEdges();

  // Remove unused node (maybe this can recover memory?)
  RemoveUnusedNodes();

  // Tile the nodes
  //TODO: generate more than just the most detailed level
  const auto& tl = *tile_hierarchy_.levels().rbegin();
  TileNodes(tl.tiles.TileSize(), tl.level);

  // Iterate through edges - tile the end nodes to create connected graph
  BuildLocalTiles(tile_hierarchy_.tile_dir(), tl.level);
}

void GraphBuilder::LuaInit(std::string nodetagtransformscript,
                           std::string nodetagtransformfunction,
                           std::string waytagtransformscript,
                           std::string waytagtransformfunction) {
  lua_.SetLuaNodeScript(nodetagtransformscript);
  lua_.SetLuaNodeFunc(nodetagtransformfunction);

  lua_.SetLuaWayScript(waytagtransformscript);
  lua_.SetLuaWayFunc(waytagtransformfunction);

  lua_.OpenLib();

}

void GraphBuilder::node_callback(uint64_t osmid, double lng, double lat,
                                 const Tags &tags) {
  // Get tags
  Tags results = lua_.TransformInLua(false, tags);
  if (results.size() == 0)
    return;

  // Create a new node and set its attributes
  OSMNode n(lat, lng);
  for (const auto& tag : results) {
    if (tag.first == "exit_to")
      n.set_exit_to(tag.second);
    else if (tag.first == "ref")
      n.set_ref(tag.second);
    else if (tag.first == "gate")
      n.set_gate((tag.second == "true" ? true : false));
    else if (tag.first == "bollard")
      n.set_bollard((tag.second == "true" ? true : false));
    else if (tag.first == "modes_mask")
      n.set_modes_mask(std::stoi(tag.second));

    //  if (osmid == 2385249)
    //    std::cout << "key: " << tag.first << " value: " << tag.second << std::endl;
  }

  // Add to the node map
  nodes_.emplace(osmid, n);
  node_count_++;
  //if (nodecount % 10000 == 0) std::cout << nodecount << " nodes" << std::endl;
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {
  // Do not add ways with < 2 nodes. Log error or add to a problem list
  // TODO - find out if we do need these, why they exist...
  if (refs.size() < 2) {
    //    std::cout << "ERROR - way " << osmid << " with < 2 nodes" << std::endl;
    return;
  }

  Tags results = lua_.TransformInLua(true, tags);

  if (results.size() == 0)
    return;

  OSMWay w(osmid);
  w.nodelist_ = refs;
  //  std::copy(refs.begin(), refs.end(), std::back_inserter(w.nodelist_));

  for (const auto& tag : results) {

    if (tag.first == "road_class") {

      RoadClass roadclass = (RoadClass) std::stoi(tag.second);

      switch (roadclass) {

        case RoadClass::kMotorway:
          w.road_class_ = RoadClass::kMotorway;
          break;
        case RoadClass::kTrunk:
          w.road_class_ = RoadClass::kTrunk;
          break;
        case RoadClass::kPrimary:
          w.road_class_ = RoadClass::kPrimary;
          break;
        case RoadClass::kTertiaryUnclassified:
          w.road_class_ = RoadClass::kTertiaryUnclassified;
          break;
        case RoadClass::kResidential:
          w.road_class_ = RoadClass::kResidential;
          break;
        case RoadClass::kService:
          w.road_class_ = RoadClass::kService;
          break;
        case RoadClass::kTrack:
          w.road_class_ = RoadClass::kTrack;
          break;
        default:
          w.road_class_ = RoadClass::kOther;
          break;
      }
    }

    else if (tag.first == "auto_forward")
      w.auto_forward_ = (tag.second == "true" ? true : false);
    else if (tag.first == "bike_forward")
      w.bike_forward_ = (tag.second == "true" ? true : false);
    else if (tag.first == "auto_backward")
      w.auto_backward_ = (tag.second == "true" ? true : false);
    else if (tag.first == "bike_backward")
      w.bike_backward_ = (tag.second == "true" ? true : false);
    else if (tag.first == "pedestrian")
      w.pedestrian_ = (tag.second == "true" ? true : false);

    else if (tag.first == "private")
      w.private_ = (tag.second == "true" ? true : false);

    else if (tag.first == "use") {

      Use use = (Use) std::stoi(tag.second);

      switch (use) {

        case Use::kNone:
          w.use_ = Use::kNone;
          break;
        case Use::kCycleway:
          w.use_ = Use::kCycleway;
          break;
        case Use::kParkingAisle:
          w.use_ = Use::kParkingAisle;
          break;
        case Use::kDriveway:
          w.use_ = Use::kDriveway;
          break;
        case Use::kAlley:
          w.use_ = Use::kAlley;
          break;
        case Use::kEmergencyAccess:
          w.use_ = Use::kEmergencyAccess;
          break;
        case Use::kDriveThru:
          w.use_ = Use::kDriveThru;
          break;
        case Use::kSteps:
          w.use_ = Use::kSteps;
          break;
        case Use::kOther:
          w.use_ = Use::kOther;
          break;
        default:
          w.use_ = Use::kNone;
          break;
      }
    }

    else if (tag.first == "no_thru_traffic_")
      w.no_thru_traffic_ = (tag.second == "true" ? true : false);
    else if (tag.first == "oneway")
      w.oneway_ = (tag.second == "true" ? true : false);
    else if (tag.first == "roundabout")
      w.roundabout_ = (tag.second == "true" ? true : false);
    else if (tag.first == "link")
      w.link_ = (tag.second == "true" ? true : false);
    else if (tag.first == "ferry")
      w.ferry_ = (tag.second == "true" ? true : false);
    else if (tag.first == "rail")
      w.rail_ = (tag.second == "true" ? true : false);

    else if (tag.first == "name")
      w.name_ = tag.second;
    else if (tag.first == "name:en")
      w.name_en_ = tag.second;
    else if (tag.first == "alt_name")
      w.alt_name_ = tag.second;
    else if (tag.first == "official_name")
      w.official_name_ = tag.second;

    else if (tag.first == "speed")
      w.speed = std::stof(tag.second);

    else if (tag.first == "ref")
      w.ref_ = tag.second;
    else if (tag.first == "int_ref")
      w.int_ref_ = tag.second;

    else if (tag.first == "surface")
      w.surface_ = (tag.second == "true" ? true : false);

    else if (tag.first == "lanes")
      w.lanes_ = std::stoi(tag.second);

    else if (tag.first == "tunnel")
      w.tunnel_ = (tag.second == "true" ? true : false);
    else if (tag.first == "toll")
      w.toll_ = (tag.second == "true" ? true : false);
    else if (tag.first == "bridge")
      w.bridge_ = (tag.second == "true" ? true : false);

    else if (tag.first == "bike_network_mask")
      w.bike_network_mask_ = std::stoi(tag.second);
    else if (tag.first == "bike_national_ref")
      w.bike_national_ref_ = tag.second;
    else if (tag.first == "bike_regional_ref")
      w.bike_regional_ref_ = tag.second;
    else if (tag.first == "bike_local_ref")
      w.bike_local_ref_ = tag.second;

    else if (tag.first == "destination")
      w.destination_ = tag.second;
    else if (tag.first == "destination:ref")
      w.destination_ref_ = tag.second;
    else if (tag.first == "destination:ref:to")
      w.destination_ref_to_ = tag.second;
    else if (tag.first == "junction_ref")
      w.junction_ref_ = tag.second;

    // if (osmid == 368034)   //http://www.openstreetmap.org/way/368034#map=18/39.82859/-75.38610
    //   std::cout << "key: " << tag.first << " value: " << tag.second << std::endl;

  }

  ways_.push_back(w);

}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                                     const CanalTP::References & /*refs*/) {
  relation_count_++;

}

void GraphBuilder::PrintCounts() {
  std::cout << "Read and parsed " << node_count_ << " nodes, " << ways_.size()
            << " ways and " << relation_count_ << " relations" << std::endl;
}

// Once all the ways and nodes are read, we compute how many times a node
// is used. This allows us to detect intersections (nodes in a graph).
void GraphBuilder::SetNodeUses() {
  // Iterate through the ways
  for (auto way : ways_) {
    // Iterate through the nodes that make up the way
    for (auto node : way.nodelist_) {
      nodes_[node].IncrementUses();
    }
    // make sure that the last node is considered as an extremity
    // TODO - could avoid this by altering ConstructEdges
    nodes_[way.nodelist_.back()].IncrementUses();
  }
}

// Construct edges in the graph.
// TODO - compare logic to example_routing app. to see why the edge
// count differs.
void GraphBuilder::ConstructEdges() {
  // Iterate through the OSM ways
  unsigned int edgeindex = 0;
  unsigned int wayindex = 0;
  uint64_t target, current;
  for (auto way : ways_) {
    // TODO - memory use? Delete temporary edges?
    Edge* edge = new Edge;
    current = way.nodelist_[0];
    OSMNode& edgestartnode = nodes_[current];
    edge->sourcenode_ = current;
    edge->AddLL(edgestartnode.latlng());
    edge->wayindex_ = wayindex;
    for (size_t i = 1, n = way.nodelist_.size(); i < n; i++) {
      // Add the node lat,lng to the edge shape
      current = way.nodelist_[i];
      OSMNode& node = nodes_[current];
      edge->AddLL(node.latlng());

      // If a node is used more than once, it is an intersection, hence it's
      // a node of the road network graph
      if (node.uses() > 1) {
        edge->targetnode_ = current;
        edges_.push_back(*edge);

        // Add the edge index to the start and target node
        edgestartnode.AddEdge(edgeindex);
        node.AddEdge(edgeindex);

        // Start a new edge
        edge = new Edge;
        edge->sourcenode_ = current;
        edgestartnode = node;
        edge->wayindex_ = wayindex;

        edgeindex++;
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
      nodes_.erase(itr++);
    } else {
      ++itr;
    }
  }
  std::cout << "Removed unused nodes; size = " << nodes_.size() << std::endl;
}

void GraphBuilder::TileNodes(const float tilesize, const unsigned int level) {
  std::cout << "Tile nodes..." << std::endl;
  int tileid;
  unsigned int n;
  Tiles tiles(AABBLL(-90.0f, -180.0f, 90.0f, 180.0f), tilesize);

  // Get number of tiles and resize the tilednodes vector
  unsigned int tilecount = tiles.TileCount();
  tilednodes_.resize(tilecount);

  for (auto node : nodes_) {
    if (node.second.edge_count() == 0) {
      std::cout << "Node with no edges" << std::endl;
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
    node_graphids_.insert(
        std::make_pair(node.first, GraphId((unsigned int) tileid, level, n)));

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
  unsigned int tileid = 0;
  for (auto tile : tilednodes_) {
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
    unsigned int directededgecount = 0;
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
        // Assign nodes
        auto nodea = node_graphids_[edge.sourcenode_];
        auto nodeb = node_graphids_[edge.targetnode_];

        directededge.set_endnode(nodeb);

        // Compute length from the latlngs.
        float length = node.latlng().Length(*edge.latlngs_);
        directededge.set_length(length);

        OSMWay &w = ways_[edge.wayindex_];

        directededge.set_importance(w.road_class_);
        directededge.set_use(w.use_);
        directededge.set_link(w.link_);
        directededge.set_speed(w.speed);    // KPH

        directededge.set_ferry(w.ferry_);
        directededge.set_railferry(w.rail_);
        directededge.set_toll(w.toll_);
        directededge.set_dest_only(w.private_);
        directededge.set_unpaved(w.surface_);
        directededge.set_tunnel(w.tunnel_);
        directededge.set_roundabout(w.roundabout_);
        directededge.set_bridge(w.bridge_);
        directededge.set_bikenetwork(w.bike_network_mask_);

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

        if (edge.sourcenode_ == osmnodeid) {  //forward direction

          directededge.set_caraccess(true, false, w.auto_forward_);
          directededge.set_pedestrianaccess(true, false, w.pedestrian_);
          directededge.set_bicycleaccess(true, false, w.bike_forward_);

          directededge.set_caraccess(false, true, w.auto_backward_);
          directededge.set_pedestrianaccess(false, true, w.pedestrian_);
          directededge.set_bicycleaccess(false, true, w.bike_backward_);

        } else {  //reverse direction.  Must flip in relation to the drawing of the way.

          directededge.set_caraccess(true, false, w.auto_backward_);
          directededge.set_pedestrianaccess(true, false, w.pedestrian_);
          directededge.set_bicycleaccess(true, false, w.bike_backward_);

          directededge.set_caraccess(false, true, w.auto_forward_);
          directededge.set_pedestrianaccess(false, true, w.pedestrian_);
          directededge.set_bicycleaccess(false, true, w.bike_forward_);

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
          edgeinfo.set_shape(*edge.latlngs_);
          // TODO - names
          std::vector<std::string> names = w.GetNames();
          std::vector<size_t> street_name_offset_list;

          for (auto name : names) {
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
