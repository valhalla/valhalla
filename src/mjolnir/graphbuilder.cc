#include <boost/functional/hash.hpp>
#include <future>

#include "mjolnir/graphbuilder.h"

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"

#include <utility>
#include <algorithm>
#include <string>
#include <thread>
#include <memory>
#include <list>
#include <unordered_set>
#include <valhalla/midgard/aabbll.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>


using namespace valhalla::midgard;
using namespace valhalla::baldr;

using node_pair = std::pair<const valhalla::baldr::GraphId&, const valhalla::baldr::GraphId&>;

namespace valhalla {
namespace mjolnir {

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;


NodeIdTable::NodeIdTable(const uint64_t maxosmid): maxosmid_(maxosmid) {
  // Create a vector to mark bits. Initialize to 0.
  bitmarkers_.resize((maxosmid / 64) + 1, 0);
}

NodeIdTable::~NodeIdTable() {

}

void NodeIdTable::set(const uint64_t id) {
  // Test if the max is exceeded
  if (id > maxosmid_) {
    throw std::runtime_error("NodeIDTable - OSM Id exceeds max specified");
  }
  bitmarkers_[id / 64] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
}

const bool NodeIdTable::IsUsed(const uint64_t id) const {
  return bitmarkers_[id / 64] & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
}

GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt,
                           const std::string& input_file)
    : node_count_(0), edge_count_(0), speed_assignment_count_(0), input_file_(input_file),
      tile_hierarchy_(pt),
      shape_(kMaxOSMNodeId),
      intersection_(kMaxOSMNodeId){

  // Initialize Lua based on config
  LuaInit(pt.get<std::string>("tagtransform.node_script"),
          pt.get<std::string>("tagtransform.node_function"),
          pt.get<std::string>("tagtransform.way_script"),
          pt.get<std::string>("tagtransform.way_function"));
}

void GraphBuilder::Build() {
  // Parse the ways and relations. Find all node Ids needed.
  std::cout << "Parsing ways and relations to mark nodes needed" << std::endl;
  CanalTP::read_osm_pbf(input_file_, *this, CanalTP::Interest::WAYS);
  CanalTP::read_osm_pbf(input_file_, *this, CanalTP::Interest::RELATIONS);
  std::cout << "Routable ways " << ways_.size() << std::endl;

  std::cout << "Percentage of ways using speed assignment: " << std::fixed <<
      std::setprecision(2) <<
      (static_cast<float>(speed_assignment_count_) / ways_.size()) * 100  << std::endl;

  // Run through the nodes
  std::cout << "Parsing nodes but only keeping " << node_count_ << std::endl;
  nodes_.reserve(node_count_);
  //TODO: we know how many knows we expect, stop early once we have that many
  CanalTP::read_osm_pbf(input_file_, *this, CanalTP::Interest::NODES);
  std::cout << "Routable nodes " << nodes_.size() << std::endl;

  // Construct edges
  ConstructEdges();

  // Tile the nodes
  //TODO: generate more than just the most detailed level?
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
  // Check if it is in the list of nodes used by ways
  if (!shape_.IsUsed(osmid)) {
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
  nodes_.emplace(osmid, std::move(n));

  if (nodes_.size() % 1000000 == 0) {
    std::cout << "Processed " << nodes_.size() << " nodes on ways" << std::endl;
  }
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {
  // Do not add ways with < 2 nodes. Log error or add to a problem list
  // TODO - find out if we do need these, why they exist...
  if (refs.size() < 2) {
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

  // Mark the nodes that we will care about when processing nodes
  for (const auto ref : refs) {
    if(shape_.IsUsed(ref)) {
      intersection_.set(ref);
      ++edge_count_;
    }
    else {
      ++node_count_;
    }
    shape_.set(ref);
  }
  intersection_.set(refs.front());
  intersection_.set(refs.back());
  edge_count_ += 2;

  float default_speed;
  bool has_speed = false;

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
        case Use::kFootway:
          w.set_use(Use::kFootway);
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

    else if (tag.first == "no_thru_traffic")
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

    else if (tag.first == "speed") {
      w.set_speed(std::stof(tag.second));
      has_speed = true;
    }

    else if (tag.first == "default_speed")
      default_speed = std::stof(tag.second);

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

//If no speed has been set by a user, assign a speed based on highway tag.
  if (!has_speed) {
    w.set_speed(default_speed);
    speed_assignment_count_++;
  }


  // Add the way to the list
  ways_.emplace_back(std::move(w));
//  if (ways_.size() % 1000000 == 0) std::cout << ways_.size() <<
//      " ways parsed" << std::endl;
}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                                     const CanalTP::References & /*refs*/) {
  //TODO:
}

// Construct edges in the graph.
// TODO - compare logic to example_routing app. to see why the edge
// count differs.
void GraphBuilder::ConstructEdges() {
  // Iterate through the OSM ways
  uint32_t edgeindex = 0;
  uint32_t wayindex = 0;
  uint64_t currentid;
  edges_.reserve(edge_count_);
  for (const auto& way : ways_) {
    // Start an edge at the first node of the way and add the
    // edge index to the node
    currentid = way.nodes()[0];
    OSMNode& node = nodes_[currentid];
    Edge edge(currentid, wayindex, node.latlng());
    node.AddEdge(edgeindex);

    // Iterate through the nodes of the way and add lat,lng to the current
    // way until a node with > 1 uses is found.
    for (size_t i = 1, n = way.node_count(); i < n; i++) {
      // Add the node lat,lng to the edge shape.
      // TODO - not sure why I had to use a different OSMNode declaration here?
      // But if I use node from above I get errors!
      currentid = way.nodes()[i];
      OSMNode& nd = nodes_[currentid];
      edge.AddLL(nd.latlng());

      // If a is an intersection or the end of the way
      // it's a node of the road network graph
      if (intersection_.IsUsed(currentid)) {
        // End the current edge and add its edge index to the node
        edge.targetnode_ = currentid;
        nd.AddEdge(edgeindex);

        // Add the edge to the list of edges
        edges_.push_back(edge);
        edgeindex++;

        // Start a new edge if this is not the last node in the way
        if (i < n-1) {
          edge = Edge(currentid, wayindex, nd.latlng());
          nd.AddEdge(edgeindex);
        }
      }
    }
    wayindex++;
  }
  std::cout << "Constructed " << edges_.size() << " edges" << std::endl;
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

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNoThroughEdge(const uint64_t startnode, const uint64_t endnode,
                     const uint32_t startedgeindex,
                     const std::unordered_map<uint64_t, OSMNode>& nodes,
                     const std::vector<Edge>& edges) {
  std::unordered_set<uint64_t> visitedset;
  std::unordered_set<uint64_t> expandset;

  // Add the end node to the expand and visited sets.
  expandset.emplace(endnode);

  // Expand edges until exhausted, the maximum number of expansions occur,
  // or end up back at the starting node. No node can be visited twice.
  uint32_t n = 0;
  uint64_t node;
  uint64_t osmendnode;
  while (n < kMaxNoThruTries) {
    // If expand list is exhausted this is "not thru"
    if (expandset.empty()) {
      return true;
    }
    n++;

    // Get the node off of the expand list and add it to the visited list
    node = *expandset.begin();
    expandset.erase(expandset.begin());
    visitedset.emplace(node);

    // Expand all edges from this node
    auto nd = nodes.find(endnode);
    if (nd != nodes.end()) {
      for (const auto& edgeindex : nd->second.edges()) {
        if (edgeindex == startedgeindex) {
          // Do not allow use of the start edge
          continue;
        }
        const Edge& edge = edges[edgeindex];
        osmendnode = (edge.sourcenode_ == node) ?
            edge.targetnode_ : edge.sourcenode_;

        // Return false if we have returned back to the start node
        if (osmendnode == startnode) {
          return false;
        }

        // TODO - if edges had the road class we could end search here if a
        // major rado is encountered

        // Add to the expand set
        expandset.emplace(osmendnode);
      }
    }
  }
  return false;
}

void BuildTileSet(std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start,
                  std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_end,
                  const std::unordered_map<uint64_t, OSMNode>& nodes, const std::vector<OSMWay>& ways,
                  const std::vector<Edge>& edges, const std::string& outdir,  std::promise<size_t>& result) {

  std::cout << "Thread " << std::this_thread::get_id() << " started" << std::endl;

  // A place to keep information about what was done
  size_t written = 0;

  // For each tile in the task
  for(; tile_start != tile_end; ++tile_start) {
    try {
     // What actually writes the tile
      GraphTileBuilder graphtile;

      // Edge info offset and map
      size_t edge_info_offset = 0;
      std::unordered_map<node_pair, size_t, NodePairHasher> edge_offset_map;

      // The edgeinfo list
      std::list<EdgeInfoBuilder> edgeinfo_list;

      // Text list offset and map
      uint32_t text_list_offset = 0;
      std::unordered_map<std::string, uint32_t> text_offset_map;

      // Text list
      std::list<std::string> text_list;

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
            directededge.set_forward(true);
            directededge.set_caraccess(true, false, w.auto_forward());
            directededge.set_pedestrianaccess(true, false, w.pedestrian());
            directededge.set_bicycleaccess(true, false, w.bike_forward());

            directededge.set_caraccess(false, true, w.auto_backward());
            directededge.set_pedestrianaccess(false, true, w.pedestrian());
            directededge.set_bicycleaccess(false, true, w.bike_backward());

            // Set end node to the target (end) node
            directededge.set_endnode(nodeb);

            // Set the opposing edge offset at the end node of this directed edge.
            directededge.set_opp_index(GetOpposingIndex(edge.targetnode_,
                             edge.sourcenode_, nodes, edges));

            // Set the not_thru flag. TODO - only do this for
             if (directededge.importance() <= RoadClass::kTertiaryUnclassified) {
               directededge.set_not_thru(false);
             } else {
               directededge.set_not_thru(IsNoThroughEdge(edge.sourcenode_,
                            edge.targetnode_, edgeindex, nodes, edges));
             }
          } else if (edge.targetnode_ == osmnodeid) {
            // Reverse direction.  Reverse the access logic and end node
            directededge.set_forward(false);
            directededge.set_caraccess(true, false, w.auto_backward());
            directededge.set_pedestrianaccess(true, false, w.pedestrian());
            directededge.set_bicycleaccess(true, false, w.bike_backward());

            directededge.set_caraccess(false, true, w.auto_forward());
            directededge.set_pedestrianaccess(false, true, w.pedestrian());
            directededge.set_bicycleaccess(false, true, w.bike_forward());

            // Set end node to the source (start) node
            directededge.set_endnode(nodea);

            // Set opposing edge index at the end node of this directed edge.
            directededge.set_opp_index(GetOpposingIndex(edge.sourcenode_,
                                edge.targetnode_, nodes, edges));

            // Set the not_thru flag
              if (directededge.importance() <= RoadClass::kTertiaryUnclassified) {
                directededge.set_not_thru(false);
              } else {
                directededge.set_not_thru(IsNoThroughEdge(edge.targetnode_,
                               edge.sourcenode_, edgeindex, nodes, edges));
              }
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
            edgeinfo_list.emplace_back();
            EdgeInfoBuilder& edgeinfo = edgeinfo_list.back();
            // TODO - shape encode
            edgeinfo.set_shape(edge.latlngs_);

            std::vector<std::string> names = w.GetNames();
            std::vector<uint32_t> street_name_offset_list;

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

}

void GraphBuilder::TileNodes(const float tilesize, const uint8_t level) {
  std::cout << "Tiling nodes" << std::endl;

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

  std::cout << "Tiled nodes created" << std::endl;
}

// Build tiles for the local graph hierarchy (basically
void GraphBuilder::BuildLocalTiles(const uint8_t level) const {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(std::max(static_cast<size_t>(1), static_cast<size_t>(std::thread::hardware_concurrency())));
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::vector<std::promise<size_t> > results(threads.size());
  // Divvy up the work
  size_t floor = tilednodes_.size() / threads.size();
  size_t at_ceiling = tilednodes_.size() - (threads.size() * floor);
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
        new std::thread(BuildTileSet, tile_start, tile_end, nodes_, ways_,
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
