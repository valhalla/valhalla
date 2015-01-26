#include <future>

#include "mjolnir/graphbuilder.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"

#include <utility>
#include <algorithm>
#include <string>
#include <thread>
#include <memory>

#include <boost/format.hpp>
#include <valhalla/midgard/logging.h>
#include <boost/algorithm/string.hpp>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

// Not thru count and Duplicate way Ids (TODO - put in a stats/issues class)
uint32_t not_thru_count = 0;
std::map<std::pair<uint64_t, uint64_t>, uint32_t> DuplicateWays;

// Constructor to create table of OSM Node IDs being used
NodeIdTable::NodeIdTable(const uint64_t maxosmid): maxosmid_(maxosmid) {
  // Create a vector to mark bits. Initialize to 0.
  bitmarkers_.resize((maxosmid / 64) + 1, 0);
}

// Destructor for NodeId table
NodeIdTable::~NodeIdTable() {
}

// Set an OSM Id within the node table
void NodeIdTable::set(const uint64_t id) {
  // Test if the max is exceeded
  if (id > maxosmid_) {
    throw std::runtime_error("NodeIDTable - OSM Id exceeds max specified");
  }
  bitmarkers_[id / 64] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
}

// Check if an OSM Id is used (in the Node table)
const bool NodeIdTable::IsUsed(const uint64_t id) const {
  return bitmarkers_[id / 64] & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
}

// Construct GraphBuilder based on properties file and input PBF extract
GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt)
    : node_count_(0),
      edge_count_(0),
      speed_assignment_count_(0),
      tile_hierarchy_(pt),
      shape_(kMaxOSMNodeId),
      intersection_(kMaxOSMNodeId) {

  // Initialize Lua based on config
  LuaInit(pt.get<std::string>("tagtransform.node_script"),
          pt.get<std::string>("tagtransform.node_function"),
          pt.get<std::string>("tagtransform.way_script"),
          pt.get<std::string>("tagtransform.way_function"));
}

void GraphBuilder::Load(const std::vector<std::string>& input_files) {
  for(const auto& input_file : input_files) {
    // Parse the ways and relations. Find all node Ids needed.
    std::clock_t start = std::clock();
    LOG_INFO("Parsing ways and relations to mark nodes needed");
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::WAYS);
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::RELATIONS);
    LOG_INFO("Routable ways " + std::to_string(ways_.size()));
    uint32_t msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    LOG_INFO("Parsing ways and relations took " + std::to_string(msecs) + " ms");
  }

  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << (static_cast<float>(speed_assignment_count_) / ways_.size()) * 100;
  LOG_INFO("Percentage of ways using speed assignment: " + s.str());

  for(const auto& input_file : input_files) {
    // Run through the nodes
    std::clock_t start = std::clock();
    LOG_INFO("Parsing nodes but only keeping " + std::to_string(node_count_));
    nodes_.reserve(node_count_);
    //TODO: we know how many knows we expect, stop early once we have that many
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::NODES);
    LOG_INFO("Routable nodes " + std::to_string(nodes_.size()));
    uint32_t msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    LOG_INFO("Parsing nodes took " + std::to_string(msecs) + " ms");
  }
}

// Build the graph from the input
void GraphBuilder::Build() {
  // Construct edges
  std::clock_t start = std::clock();
  ConstructEdges();
  uint32_t msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("ConstructEdges took " + std::to_string(msecs) + " ms");

  // Sort the edge indexes at the nodes (by driveability and importance)
  start = std::clock();
  SortEdgesFromNodes();
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("SortEdges took " + std::to_string(msecs) + " ms");

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  start = std::clock();
  ReclassifyLinks();
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("ReclassifyLinks took " + std::to_string(msecs) + " ms");

  // Tile the nodes at the base (local) level
  start = std::clock();
  const auto& tl = tile_hierarchy_.levels().rbegin();
  TileNodes(tl->second.tiles.TileSize(), tl->second.level);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("TileNodes took " + std::to_string(msecs) + " ms");

  // Iterate through edges - tile the end nodes to create connected graph
  start = std::clock();
  BuildLocalTiles(tl->second.level);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("BuildLocalTiles took " + std::to_string(msecs) + " ms");

  // TODO - log statistics and issues

  // Log the duplicates
  uint32_t duplicates = 0;
  LOG_WARN("Duplicate Way count = " + std::to_string(DuplicateWays.size()));
  for (const auto& dup : DuplicateWays) {
    LOG_WARN("Duplicate: wayids = " + std::to_string(dup.first.first) +
        " and " + std::to_string(dup.first.second) + " Created " +
        std::to_string(dup.second) + " duplicate edges");
    duplicates += dup.second;
  }
  LOG_INFO("Duplicate edgecount = " + std::to_string(duplicates));
  LOG_INFO("Not thru edgecount = " + std::to_string(not_thru_count));
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
  OSMNode n(lng, lat);
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
  }

  // Add to the node map;
  nodes_.emplace(osmid, std::move(n));

  if (nodes_.size() % 1000000 == 0) {
    LOG_INFO("Processed " + std::to_string(nodes_.size()) + " nodes on ways");
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
  bool has_surface = true;

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
        case RoadClass::kSecondary:
          w.set_road_class(RoadClass::kSecondary);
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
        case Use::kTrack:
          w.set_use(Use::kTrack);
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

    else if (tag.first == "surface") {
      std::string value = tag.second;
      boost::algorithm::to_lower(value);

      if (value.find("paved") != std::string::npos
          || value.find("pavement") != std::string::npos
          || value.find("asphalt") != std::string::npos
          || value.find("concrete") != std::string::npos
          || value.find("cement") != std::string::npos)
        w.set_surface(Surface::kPavedSmooth);

      else if (value.find("tartan") != std::string::npos
          || value.find("pavingstone") != std::string::npos
          || value.find("paving_stones") != std::string::npos
          || value.find("sett") != std::string::npos)
        w.set_surface(Surface::kPaved);

      else if (value.find("cobblestone") != std::string::npos
          || value.find("brick") != std::string::npos)
        w.set_surface(Surface::kPavedRough);

      else if (value.find("compacted") != std::string::npos)
        w.set_surface(Surface::kCompacted);

      else if (value.find("dirt") != std::string::npos
          || value.find("natural") != std::string::npos
          || value.find("earth") != std::string::npos
          || value.find("ground") != std::string::npos
          || value.find("mud") != std::string::npos)
        w.set_surface(Surface::kDirt);

      else if (value.find("gravel") != std::string::npos
          || value.find("pebblestone") != std::string::npos
          || value.find("sand") != std::string::npos
          || value.find("wood") != std::string::npos
          || value.find("boardwalk") != std::string::npos
          || value.find("unpaved") != std::string::npos)
        w.set_surface(Surface::kGravel);
      else if (value.find("grass") != std::string::npos)
        w.set_surface(Surface::kPath);
      //We have to set a flag as surface may come before Road classes and Uses
      else has_surface = false;
    }

    else if (tag.first == "cycle_lane") {
      CycleLane cyclelane = (CycleLane) std::stoi(tag.second);
      switch (cyclelane) {
        case CycleLane::kNone:
          w.set_cyclelane(CycleLane::kNone);
          break;
        case CycleLane::kDedicated:
          w.set_cyclelane(CycleLane::kDedicated);
          break;
        case CycleLane::kSeparated:
          w.set_cyclelane(CycleLane::kSeparated);
          break;
        case CycleLane::kShared:
          w.set_cyclelane(CycleLane::kShared);
          break;
        default:
          w.set_use(Use::kNone);
          break;
      }
    }

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
  }

  //If no surface has been set by a user, assign a surface based on Road Class and Use
  if (!has_surface) {

    switch (w.road_class()) {

      case RoadClass::kMotorway:
      case RoadClass::kTrunk:
      case RoadClass::kPrimary:
      case RoadClass::kTertiaryUnclassified:
      case RoadClass::kResidential:
      case RoadClass::kService:
        w.set_surface(Surface::kPavedSmooth);
        break;
      default:
        switch (w.use()) {

        case Use::kFootway:
        case Use::kTrack:
          w.set_surface(Surface::kPath);
          break;
        case Use::kParkingAisle:
        case Use::kDriveway:
        case Use::kAlley:
        case Use::kEmergencyAccess:
        case Use::kDriveThru:
          w.set_surface(Surface::kPavedSmooth);
          break;
        case Use::kCycleway:
        case Use::kSteps:
          w.set_surface(Surface::kPaved);
          break;
        default:
          w.set_surface(Surface::kImpassable);  //Not sure about this one.
          break;
        }
        break;
    }
  }

  //If no speed has been set by a user, assign a speed based on highway tag.
  if (!has_speed) {
    w.set_speed(default_speed);
    speed_assignment_count_++;
  }

  // Add the way to the list
  ways_.emplace_back(std::move(w));
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
  uint64_t startnodeid, nodeid;
  edges_.reserve(edge_count_);
  for (size_t wayindex = 0; wayindex < ways_.size(); wayindex++) {
    // Get some way attributes needed for the edge
    const auto& way = ways_[wayindex];

    // Start an edge at the first node of the way and add the
    // edge index to the node
    startnodeid = nodeid = way.nodes()[0];
    OSMNode& node = nodes_[nodeid];
    Edge edge(nodeid, wayindex, node.latlng(), way);
    node.AddEdge(edgeindex, way.link());

    // Iterate through the nodes of the way and add lat,lng to the current
    // way until a node with > 1 uses is found.
    for (size_t i = 1; i < way.node_count(); i++) {
      // Add the node lat,lng to the edge shape.
      nodeid = way.nodes()[i];
      OSMNode& nd = nodes_[nodeid];
      edge.AddLL(nd.latlng());

      // If a is an intersection or the end of the way
      // it's a node of the road network graph
      if (intersection_.IsUsed(nodeid)) {
        // End the current edge and add its edge index to the node
        edge.targetnode_ = nodeid;

        // Add the edgeindex to the node (unless this is a loop with same
        // start and end node Ids)
        if (startnodeid != nodeid) {
          nd.AddEdge(edgeindex, way.link());
        }

        // Add the edge to the list of edges
        edges_.emplace_back(std::move(edge));
        edgeindex++;

        // Start a new edge if this is not the last node in the way
        if (i < way.node_count() - 1) {
          startnodeid = nodeid;
          edge = Edge(nodeid, wayindex, nd.latlng(), way);
          nd.AddEdge(edgeindex, way.link());
        }
      }
    }
  }
  LOG_INFO("Constructed " + std::to_string(edges_.size()) + " edges");
}

class EdgeSorter {
 public:
  EdgeSorter(const std::vector<Edge>& edges)
     : osmnodeid_(0),
       edges_(edges) {
  }
  void SetNodeId(const uint64_t nodeid) {
    osmnodeid_ = nodeid;
  }
  bool operator() (const uint32_t e1index, const uint32_t e2index) const {
    const Edge& e1 = edges_[e1index];
    const Edge& e2 = edges_[e2index];

    // Check if edges are forward or reverse
    bool e1forward = (e1.sourcenode_ == osmnodeid_);
    bool e2forward = (e2.sourcenode_ == osmnodeid_);

    bool e1drive = ((e1forward && e1.attributes_.fields.driveableforward) ||
                       (!e1forward && e1.attributes_.fields.driveablereverse));
    bool e2drive = ((e2forward && e2.attributes_.fields.driveableforward) ||
                       (!e2forward && e2.attributes_.fields.driveablereverse));

    // If both driveable or both not driveable, compare importance
    if (e1drive == e2drive) {
      return e1.attributes_.fields.importance < e2.attributes_.fields.importance;
    } else if (e1drive && !e2drive) {
      return true;
    } else {
      return false;
    }
  }
 private:
  uint64_t osmnodeid_;
  const std::vector<Edge>& edges_;
};

// Sort edge indexes from each node
void GraphBuilder::SortEdgesFromNodes() {
  // Sort the directed edges from the node
  EdgeSorter sorter(edges_);
  for (auto& node : nodes_) {
    sorter.SetNodeId(node.first);
    std::sort(node.second.mutable_edges().begin(),
              node.second.mutable_edges().end(), sorter);
  }
}

uint32_t GraphBuilder::GetBestNonLinkClass(const OSMNode& node) const {
  uint32_t bestrc = kAbsurdRoadClass;
  for (auto idx : node.edges()) {
    const Edge& edge = edges_[idx];
    if (!edge.attributes_.fields.link) {
      if (edge.attributes_.fields.importance < bestrc)
        bestrc = edge.attributes_.fields.importance;
    }
  }
  return bestrc;
}

// Reclassify links (ramps and turn channels).
void GraphBuilder::ReclassifyLinks() {
  uint32_t count = 0;
  std::unordered_set<uint64_t> visitedset;  // Set of visited nodes
  std::unordered_set<uint64_t> expandset;   // Set of nodes to expand
  std::vector<uint32_t> linkedgeindexes;    // Edge indexes to reclassify
  for (const auto& node : nodes_) {
    if (node.second.link_edge() && node.second.non_link_edge()) {
      // Get the highest classification of non-link edges at this node
      uint32_t beststartrc = GetBestNonLinkClass(node.second);

      // Expand from all link edges
      for (auto startedgeindex : node.second.edges()) {
        // Get the edge information. Skip non-link edges
        const Edge& startedge = edges_[startedgeindex];
        if (!startedge.attributes_.fields.link) {
          continue;
        }

        // Clear the sets and edge list
        visitedset.clear();
        expandset.clear();
        linkedgeindexes.clear();

        // Add the start edge to list of links edges to potentially reclassify
        linkedgeindexes.push_back(startedgeindex);

        // If the first end node contains a non-link edge we compute the best
        // classification. If not we add the end node to the expand set.
        uint32_t bestendrc = kAbsurdRoadClass;
        uint64_t endnode = (startedge.sourcenode_ == node.first) ?
            startedge.targetnode_ : startedge.sourcenode_;
        auto firstendnode = nodes_.find(endnode);
        if (firstendnode != nodes_.end()) {
          if (firstendnode->second.non_link_edge()) {
            bestendrc = GetBestNonLinkClass(firstendnode->second);
          } else {
            expandset.insert(endnode);
          }
        }

        // Expand edges until all paths reach a node that has a non-link
        for (uint32_t n = 0; n < 512; n++) {
          // Once expand list is empty - mark all link edges encountered
          // with the specified classification / importance and break out
          // of this loop (can still expand other ramp edges
          // from the starting node
          if (expandset.empty()) {
            // Make sure this connects...
            if (bestendrc == kAbsurdRoadClass)  {
              LOG_WARN("Link edge may not connect: OSM WayID = " +
                       std::to_string(ways_[startedge.wayindex_].way_id()) +
                       " n = " + std::to_string(n));
            } else {
              // Set to the lower of the 2 best road classes (start and end).
              // Apply this to each of the link edges
              uint32_t rc = std::max(beststartrc, bestendrc);
              for (auto idx : linkedgeindexes) {
                if (rc > edges_[idx].attributes_.fields.importance) {
                  edges_[idx].attributes_.fields.importance = rc;
                  count++;
                }
              }
            }
            break;
          }

          // Get the node off of the expand list and add it to the visited list
          uint64_t expandnode = *expandset.begin();
          expandset.erase(expandset.begin());
          visitedset.insert(expandnode);

          // Expand all edges from this node
          const auto nd = nodes_.find(expandnode);
          if (nd == nodes_.end()) {
            continue;
          }
          for (const auto& edgeindex : nd->second.edges()) {
            // Do not allow use of the start edge
            if (edgeindex == startedgeindex) {
              continue;
            }

            // Add this edge (it should be a link edge) and get its end node
            const Edge& nextedge = edges_[edgeindex];
            if (!nextedge.attributes_.fields.link) {
              LOG_ERROR("Expanding onto non-link edge!");
              continue;
            }
            uint32_t osmendnode = (nextedge.sourcenode_ == expandnode) ?
                    nextedge.targetnode_ : nextedge.sourcenode_;
            linkedgeindexes.push_back(edgeindex);
            const auto endnd = nodes_.find(osmendnode);
            if (endnd == nodes_.end()) {
              continue;
            }

            // If the end node contains any non-links find the best
            // classification - do not expand from this node
            if (endnd->second.non_link_edge()) {
              uint32_t rc = GetBestNonLinkClass(endnd->second);
              if (rc < bestendrc) {
                bestendrc = rc;
              }
            } else if (visitedset.find(osmendnode) == visitedset.end()) {
              // Add to the expand set if not in the visited set
              expandset.insert(osmendnode);
            }
          }
        }
      }
    }
  }
  LOG_INFO((boost::format("Reclassify Links: Count %1%") % count).str());
}

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNoThroughEdge(const uint64_t startnode, const uint64_t endnode,
                     const uint32_t startedgeindex,
                     const std::unordered_map<uint64_t, OSMNode>& nodes,
                     const std::vector<Edge>& edges) {
  // Add the end node Id to the set of nodes to expand
  std::unordered_set<uint64_t> visitedset;
  std::unordered_set<uint64_t> expandset;
  expandset.insert(endnode);

  // Expand edges until exhausted, the maximum number of expansions occur,
  // or end up back at the starting node. No node can be visited twice.
  for (uint32_t n = 0; n < kMaxNoThruTries; n++) {
    // If expand list is exhausted this is "not thru"
    if (expandset.empty()) {
      return true;
    }

    // Get the node off of the expand list and add it to the visited list.
    // Expand edges from this node.
    uint64_t node = *expandset.begin();
    expandset.erase(expandset.begin());
    visitedset.emplace(node);
    const auto nd = nodes.find(node);
    if (nd != nodes.end()) {
      for (const auto& edgeindex : nd->second.edges()) {
        if (edgeindex == startedgeindex) {
          // Do not allow use of the start edge
          continue;
        }

        // Return false if we have returned back to the start node or we
        // encounter a tertiary road (or better)
        const Edge& edge = edges[edgeindex];
        uint64_t osmendnode = (edge.sourcenode_ == node) ?
                  edge.targetnode_ : edge.sourcenode_;
        if (osmendnode == startnode ||
            edge.attributes_.fields.importance <=
            static_cast<uint32_t>(RoadClass::kTertiaryUnclassified)) {
          return false;
        }

        // Add to the expand set if not in the visited set
        if (visitedset.find(osmendnode) == visitedset.end()) {
          expandset.insert(osmendnode);
        }
      }
    }
  }
  return false;
}

/**
 * Get the use for a link (either a kRamp or kTurnChannel)
 * TODO - validate logic with some real world cases.
 */
Use GetLinkUse(const RoadClass rc, const float length,
               const uint64_t startnode,  const uint64_t endnode,
               const std::unordered_map<uint64_t, OSMNode>& nodes) {
  // Assume link that has highway = motorway or trunk is a ramp.
  // Also, if length is > kMaxTurnChannelLength we assume this is a ramp
  if (rc == RoadClass::kMotorway || rc == RoadClass::kTrunk ||
      length > kMaxTurnChannelLength) {
    return Use::kRamp;
  }

  // TODO - if there is a exit sign or exit number present this is
  // considered kRamp

  // Both end nodes have to connect to a non-link edge. If either end node
  // connects only to "links" this likely indicates a split or fork,
  // which are not so prevalent in turn channels.
  auto startnd = nodes.find(startnode);
  if (startnd != nodes.end()) {
    auto endnd = nodes.find(endnode);
    if (endnd != nodes.end()) {
      if (startnd->second.non_link_edge() && endnd->second.non_link_edge())
        return Use::kTurnChannel;
    }
  }
  return Use::kRamp;
}

struct DuplicateEdgeInfo {
  uint32_t edgeindex;
  uint32_t length;

  DuplicateEdgeInfo() : edgeindex(0), length(0) { }
  DuplicateEdgeInfo(const uint32_t idx, const uint32_t l)
      : edgeindex(idx),
        length(l) {
  }
};

void CheckForDuplicates(const uint64_t osmnodeid, const OSMNode& node,
                        const std::vector<uint32_t>& edgelengths,
                        const std::unordered_map<uint64_t, OSMNode>& nodes,
                        const std::vector<Edge>& edges,
                        const std::vector<OSMWay>& ways) {
  uint32_t edgeindex;
  uint64_t endnode;
  std::unordered_map<uint64_t, DuplicateEdgeInfo> endnodes;
  for (size_t n = 0; n < node.edge_count(); n++) {
    edgeindex = node.edges().at(n);
    const Edge& edge = edges[edgeindex];
    if (edge.sourcenode_ == osmnodeid) {
      endnode = nodes.find(edge.targetnode_)->second.graphid().value();
    } else {
      endnode = nodes.find(edge.sourcenode_)->second.graphid().value();
    }

    // Check if the end node is already in the set of edges from this node
    const auto en = endnodes.find(endnode);
    if (en != endnodes.end() && en->second.length == edgelengths[n]) {
      uint64_t wayid1 = ways[edges[en->second.edgeindex].wayindex_].way_id();
      uint64_t wayid2 = ways[edges[edgeindex].wayindex_].way_id();
      std::pair<uint64_t, uint64_t> wayids = std::make_pair(wayid1, wayid2);
      auto it = DuplicateWays.find(wayids);
      if (it == DuplicateWays.end()) {
        DuplicateWays.emplace(wayids, 1);
      } else {
        it->second++;
      }
    } else {
      endnodes.emplace(endnode, DuplicateEdgeInfo(edgeindex, edgelengths[n]));
    }
  }
}

void BuildTileSet(std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start,
                  std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_end,
                  const std::unordered_map<uint64_t, OSMNode>& nodes, const std::vector<OSMWay>& ways,
                  const std::vector<Edge>& edges, const baldr::TileHierarchy& hierarchy,  std::promise<size_t>& result) {

  std::string thread_id = static_cast<std::ostringstream&>(std::ostringstream() << std::this_thread::get_id()).str();
  LOG_INFO("Thread " + thread_id + " started");

  // A place to keep information about what was done
  size_t written = 0;

  // For each tile in the task
  bool added = false;
  for(; tile_start != tile_end; ++tile_start) {
    try {
     // What actually writes the tile
      GraphTileBuilder graphtile;

      // Iterate through the nodes
      uint32_t directededgecount = 0;
      for (const auto& osmnodeid : tile_start->second) {
        const OSMNode& node = nodes.find(osmnodeid)->second; //TODO: check validity?
        NodeInfoBuilder nodebuilder;
        nodebuilder.set_latlng(node.latlng());

        // Set the index of the first outbound edge within the tile.
        nodebuilder.set_edge_index(directededgecount);
        nodebuilder.set_edge_count(node.edge_count());

        // Track the best classification/importance or edge
        RoadClass bestrc = RoadClass::kOther;

        // Compute edge lengths from the edge lat,lngs (round to nearest meter)
        std::vector<uint32_t> edgelengths;
        for (auto edgeindex : node.edges()) {
          float length = node.latlng().Length(edges[edgeindex].latlngs_);
          edgelengths.push_back(static_cast<uint32_t>(length + 0.5f));
        }

        // Look for potential duplicates
        CheckForDuplicates(osmnodeid, node, edgelengths, nodes, edges, ways);

        // Build directed edges
        uint32_t edgeindex;
        std::vector<DirectedEdgeBuilder> directededges;
        directededgecount += node.edge_count();
        for (size_t n = 0; n < node.edge_count(); n++) {
          directededges.emplace_back();
          DirectedEdgeBuilder& directededge = directededges.back();
          edgeindex = node.edges().at(n);
          const Edge& edge = edges[edgeindex];

          // Set length
          directededge.set_length(edgelengths[n]);

          // Get the way information and set attributes
          const OSMWay &w = ways[edge.wayindex_];

          // Set the importance (from the edge rather than the way since
          // ReclassifyLinks may change the importance). Update the node's
          // best road class
          directededge.set_importance(
              static_cast<RoadClass>(edge.attributes_.fields.importance));
          if (directededge.importance() < bestrc) {
            bestrc = directededge.importance();
          }

          directededge.set_use(w.use());
          directededge.set_speed(w.speed());    // KPH
          directededge.set_ferry(w.ferry());
          directededge.set_railferry(w.rail());
          directededge.set_toll(w.toll());
          directededge.set_dest_only(w.destination_only());
          directededge.set_surface(w.surface());
          directededge.set_cyclelane(w.cyclelane());
          directededge.set_tunnel(w.tunnel());
          directededge.set_roundabout(w.roundabout());
          directededge.set_bridge(w.bridge());
          directededge.set_bikenetwork(w.bike_network());

          // Link - this indicates a ramp or a turn channel. If link is set
          // test to see if we can infer that the edge is a turn channel
          directededge.set_link(w.link());
          if (directededge.link()) {
            if (directededge.use() != Use::kNone) {
              LOG_WARN("Edge is a link but has use = " +
                       std::to_string(static_cast<int>(directededge.use())));
            }
            directededge.set_use(GetLinkUse(directededge.importance(),
                 edgelengths[n], edge.sourcenode_, edge.targetnode_, nodes));

            // TEMP - update ramp speeds based on class. Should have some way
            // of doing this with lua?
            if (directededge.use() == Use::kTurnChannel) {
              directededge.set_speed(w.speed() * 1.25f);
            } else if (directededge.use() == Use::kRamp) {
              if (directededge.importance() == RoadClass::kMotorway) {
                directededge.set_speed(95.0f);
              } else if (directededge.importance() == RoadClass::kTrunk) {
                directededge.set_speed(80.0f);
              } else if (directededge.importance() == RoadClass::kPrimary) {
                directededge.set_speed(65.0f);
//              } else if (directededge.importance() == RoadClass::kSecondary) {
//                directededge.set_speed(50.0f);
              } else if (directededge.importance() == RoadClass::kTertiaryUnclassified) {
                directededge.set_speed(40.0f);
              } else {
                directededge.set_speed(25.0f);
              }
            }
          }

          // Assign nodes and determine orientation along the edge (forward
          // or reverse between the 2 nodes)
          const GraphId& nodea = nodes.find(edge.sourcenode_)->second.graphid(); //TODO: check validity?
          if (!nodea.Is_Valid()) {
            LOG_ERROR("Node A: OSMID = " + std::to_string(edge.sourcenode_) + " GraphID is not valid");
          }
          const GraphId& nodeb = nodes.find(edge.targetnode_)->second.graphid(); //TODO: check validity?
          if (!nodeb.Is_Valid()) {
            LOG_ERROR("Node B: OSMID = " + std::to_string(edge.targetnode_) + " GraphID is not valid");
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

            // Set the not_thru flag.
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

            // Set the not_thru flag
            if (directededge.importance() <= RoadClass::kTertiaryUnclassified) {
              directededge.set_not_thru(false);
            } else {
              directededge.set_not_thru(IsNoThroughEdge(edge.targetnode_,
                       edge.sourcenode_, edgeindex, nodes, edges));
            }
          } else {
            // ERROR!!!
            LOG_ERROR((boost::format("WayID =  %1% Edge Index = %2% Edge nodes %3% and %4% did not match the OSM node Id %5%")
              % w.way_id() % edgeindex %  edge.sourcenode_  % edge.targetnode_ % osmnodeid).str());
          }

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(edgeindex,
               nodea, nodeb, edge.latlngs_, w.GetNames(), added);
          directededge.set_edgedataoffset(edge_info_offset);

          // TODO - add general statistics!
          if (directededge.not_thru()) {
            not_thru_count++;
          }
        }

        // Update the best classification used by directed edges
        nodebuilder.set_bestrc(bestrc);

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);
      }

      // Write the actual tile to disk
      graphtile.StoreTileData(hierarchy, tile_start->first);

      // Made a tile
      LOG_INFO((boost::format("Thread %1% wrote tile %2%: %3% bytes") % thread_id % tile_start->first % graphtile.size()).str());
      written += graphtile.size();
    }// Whatever happens in Vagas..
    catch(std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      LOG_ERROR((boost::format("Thread %1% failed tile %2%: %3%") % thread_id % tile_start->first % e.what()).str());
      return;
    }
  }
  // Let the main thread how this thread faired
  result.set_value(written);
}

void GraphBuilder::TileNodes(const float tilesize, const uint8_t level) {
  LOG_INFO("Tiling nodes");

  // Get number of tiles and reserve space for them
  // < 30% of the earth is land and most roads are on land, even less than that even has roads
  Tiles tiles(AABB2({-180.0f, -90.0f}, {180.0f, 90.0f}), tilesize);
  tilednodes_.reserve(tiles.TileCount() * .3f);
  // Iterate through all OSM nodes and assign GraphIds
  for (auto& node : nodes_) {
    // Skip any nodes that have no edges
    if (node.second.edge_count() == 0) {
      continue;
    }
    // Put the node into the tile
    GraphId id = tile_hierarchy_.GetGraphId(node.second.latlng(), level);
    std::vector<uint64_t>& tile = tilednodes_[id];
    tile.emplace_back(node.first);
    // Set the GraphId for this OSM node.
    node.second.set_graphid(GraphId(id.tileid(), id.level(), tile.size() - 1));
  }

  LOG_INFO("Tiled nodes created");
}

// Build tiles for the local graph hierarchy
void GraphBuilder::BuildLocalTiles(const uint8_t level) const {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(std::max(static_cast<size_t>(1), static_cast<size_t>(std::thread::hardware_concurrency())));
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::vector<std::promise<size_t> > results(threads.size());
  // Divvy up the work
  size_t floor = tilednodes_.size() / threads.size();
  size_t at_ceiling = tilednodes_.size() - (threads.size() * floor);
  std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start, tile_end = tilednodes_.begin();
  LOG_INFO(std::to_string(tilednodes_.size()) + " tiles");
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(
      new std::thread(BuildTileSet, tile_start, tile_end, nodes_, ways_, edges_, tile_hierarchy_, std::ref(results[i]))
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
          LOG_INFO((boost::format("Wrote tile %1%: %2% bytes") % result->first % bytes).str());
          results.erase(result);
          break;
        }
      }
      catch (const std::exception& e) {
        //TODO: log and rethrow
        LOG_ERROR((boost::format("Failed tile %1%: %2%") % result->first % << e.what()).str());
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
