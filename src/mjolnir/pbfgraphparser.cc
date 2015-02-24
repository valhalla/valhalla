#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/util.h"

#include "mjolnir/osmpbfparser.h"

#include <future>
#include <utility>
#include <thread>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/mjolnir/luatagtransform.h>
#include <valhalla/mjolnir/idtable.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

// Construct PBFGraphParser based on properties file and input PBF extract
struct graph_callback : public OSMPBF::Callback {
 public:
  graph_callback() = delete;
  graph_callback(const graph_callback&) = delete;
  virtual ~graph_callback() {}

  graph_callback(const boost::property_tree::ptree& pt, OSMData& osmdata) :
    shape_(kMaxOSMNodeId), intersection_(kMaxOSMNodeId), tile_hierarchy_(pt.get_child("hierarchy")), osmdata_(osmdata) {

    // Initialize Lua based on config
    lua_.SetLuaNodeScript(pt.get<std::string>("tagtransform.node_script"));
    lua_.SetLuaNodeFunc(pt.get<std::string>("tagtransform.node_function"));
    lua_.SetLuaWayScript(pt.get<std::string>("tagtransform.way_script"));
    lua_.SetLuaWayFunc(pt.get<std::string>("tagtransform.way_function"));
    lua_.SetLuaRelationScript(pt.get<std::string>("tagtransform.relation_script"));
    lua_.SetLuaRelationFunc(pt.get<std::string>("tagtransform.relation_function"));
    lua_.OpenLib();
  }

  void node_callback(uint64_t osmid, double lng, double lat, const OSMPBF::Tags &tags) {
    // Check if it is in the list of nodes used by ways
    if (!shape_.IsUsed(osmid)) {
      return;
    }

    // Get tags
    Tags results = lua_.Transform(OSMType::kNode, tags);
    if (results.size() == 0)
      return;

    const auto& highway_junction = results.find("highway");
    bool is_highway_junction = ((highway_junction != results.end())
        && (highway_junction->second == "motorway_junction"));

    // Create a new node and set its attributes
    OSMNode n(osmid, lng, lat);
    for (const auto& tag : results) {

      if (tag.first == "highway") {
        n.set_traffic_signal(tag.second == "traffic_signals" ? true : false); // TODO: add logic for traffic_signals:direction
      }
      else if (is_highway_junction && (tag.first == "exit_to")) {
        bool hasTag = (tag.second.length() ? true : false);
        n.set_exit_to(hasTag);
        if (hasTag)
          osmdata_.node_exit_to[osmid] = tag.second;
      }
      else if (is_highway_junction && (tag.first == "ref")) {
        bool hasTag = (tag.second.length() ? true : false);
        n.set_ref(hasTag);
        if (hasTag)
          osmdata_.node_ref[osmid] = tag.second;
      }
      else if (is_highway_junction && (tag.first == "name")) {
        bool hasTag = (tag.second.length() ? true : false);
        n.set_name(hasTag);
        if (hasTag)
          osmdata_.node_name[osmid] = tag.second;
      }
      else if (tag.first == "gate") {
        if (tag.second == "true") {
          if (!intersection_.IsUsed(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kGate);
        }
      }
      else if (tag.first == "bollard") {
        if (tag.second == "true") {
          if (!intersection_.IsUsed(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kBollard);
        }
      }
      else if (tag.first == "access_mask")
        n.set_access_mask(std::stoi(tag.second));
    }

    // Set the intersection flag (relies on ways being processed first to set
    // the intersection Id markers).
    if (intersection_.IsUsed(osmid)) {
      n.set_intersection(true);
      osmdata_.intersection_count++;
    }

    osmdata_.nodes.emplace_back(std::move(n));

    if (osmdata_.nodes.size() % 5000000 == 0) {
      LOG_INFO("Processed " + std::to_string(osmdata_.nodes.size()) + " nodes on ways");
    }
  }

  void way_callback(uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<uint64_t> &nodes) {

    // Do not add ways with < 2 nodes. Log error or add to a problem list
    // TODO - find out if we do need these, why they exist...
    if (nodes.size() < 2) {
      return;
    }

    // Transform tags. If no results that means the way does not have tags
    // suitable for use in routing.
    Tags results = lua_.Transform(OSMType::kWay, tags);
    if (results.size() == 0) {
      return;
    }

    // Add the refs to the node reference list
    uint32_t idx = osmdata_.noderefs.size();
    for (const auto node : nodes) {
      osmdata_.noderefs.push_back(node);
    }

    // Construct OSMWay and set the node ref index and count
    OSMWay w(osmid);
    w.set_noderef_index(idx);
    w.set_node_count(nodes.size());

    // Mark the nodes that we will care about when processing nodes
    for (const auto node : nodes) {
      if(shape_.IsUsed(node)) {
        intersection_.set(node);
        ++osmdata_.edge_count;
      }
      else {
        ++osmdata_.node_count;
      }
      shape_.set(node);
    }
    intersection_.set(nodes.front());
    intersection_.set(nodes.back());
    osmdata_.edge_count += 2;

    float default_speed;
    bool has_speed = false;
    bool has_surface = true;
    std::string name;

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
          case Use::kRoad:
          default:
            w.set_use(Use::kRoad);
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

      else if (tag.first == "name" && !tag.second.empty())
        name = tag.second;
      else if (tag.first == "name:en" && !tag.second.empty())
        w.set_name_en_index(osmdata_.name_offset_map.index(tag.second));
      else if (tag.first == "alt_name" && !tag.second.empty())
        w.set_alt_name_index(osmdata_.name_offset_map.index(tag.second));
      else if (tag.first == "official_name" && !tag.second.empty())
        w.set_official_name_index(osmdata_.name_offset_map.index(tag.second));

      else if (tag.first == "speed") {
        w.set_speed(std::stof(tag.second));
        has_speed = true;
      }

      else if (tag.first == "default_speed")
        default_speed = std::stof(tag.second);

      else if (tag.first == "ref" && !tag.second.empty())
        w.set_ref_index(osmdata_.ref_offset_map.index(tag.second));
      else if (tag.first == "int_ref" && !tag.second.empty())
        w.set_int_ref_index(osmdata_.ref_offset_map.index(tag.second));

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
          case CycleLane::kDedicated:
            w.set_cyclelane(CycleLane::kDedicated);
            break;
          case CycleLane::kSeparated:
            w.set_cyclelane(CycleLane::kSeparated);
            break;
          case CycleLane::kShared:
            w.set_cyclelane(CycleLane::kShared);
            break;
          case CycleLane::kNone:
          default:
            w.set_cyclelane(CycleLane::kNone);
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
      else if (tag.first == "bike_national_ref" && !tag.second.empty())
        w.set_bike_national_ref_index(osmdata_.ref_offset_map.index(tag.second));
      else if (tag.first == "bike_regional_ref" && !tag.second.empty())
        w.set_bike_regional_ref_index(osmdata_.ref_offset_map.index(tag.second));
      else if (tag.first == "bike_local_ref" && !tag.second.empty())
        w.set_bike_local_ref_index(osmdata_.ref_offset_map.index(tag.second));

      else if (tag.first == "destination" && !tag.second.empty()) {
        w.set_destination_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      }
      else if (tag.first == "destination:ref" && !tag.second.empty()) {
        w.set_destination_ref_index(osmdata_.ref_offset_map.index(tag.second));
        w.set_exit(true);
      }
      else if (tag.first == "destination:ref:to" && !tag.second.empty()) {
        w.set_destination_ref_to_index(osmdata_.ref_offset_map.index(tag.second));
        w.set_exit(true);
      }
      else if (tag.first == "destination:street" && !tag.second.empty()) {
        w.set_destination_street_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      }
      else if (tag.first == "destination:street:to" && !tag.second.empty()) {
        w.set_destination_street_to_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      }
      else if (tag.first == "junction:ref" && !tag.second.empty()) {
        w.set_junction_ref_index(osmdata_.ref_offset_map.index(tag.second));
        w.set_exit(true);
      }
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
    if (!has_speed)
      w.set_speed(default_speed);

    // Delete the name from from name field if it exists in the ref.
    if (!name.empty() && w.ref_index()) {
      std::vector<std::string> names = GetTagTokens(name);
      std::vector<std::string> refs = GetTagTokens(osmdata_.ref_offset_map.name(w.ref_index()));
      bool bFound = false;

      std::string tmp;

      for (auto& name : names) {
        for (auto& ref : refs) {
          if (name == ref) {
            bFound = true;
            break;
          }
        }
        if (!bFound) {
          if (!tmp.empty())
            tmp += ";";
          tmp += name;
        }
        bFound = false;
      }
      if (!tmp.empty())
        w.set_name_index(osmdata_.name_offset_map.index(tmp));
    } else
      w.set_name_index(osmdata_.name_offset_map.index(name));

    // Add the way to the list
    osmdata_.ways.push_back(std::move(w));
  }

  void relation_callback(const uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<OSMPBF::Member> &members) {
    // Get tags
    Tags results = lua_.Transform(OSMType::kRelation, tags);
    if (results.size() == 0)
      return;

    OSMRestriction restriction;
    uint64_t from_way_id = 0;
    bool isRestriction = false;
    bool hasRestriction = false;
    bool isRoad = false;
    bool isRoute = false;

    std::string network, ref;

    for (const auto& tag : results) {

      if (tag.first == "type") {
        if (tag.second == "restriction")
          isRestriction = true;
        else if (tag.second == "route")
          isRoute = true;
      }
      else if (tag.first == "route") {
        if (tag.second == "road")
          isRoad = true;
      }
      else if (tag.first == "network") {
        network = tag.second;//US:US
      }
      else if (tag.first == "ref") {
        ref = tag.second;
      }
      else if (tag.first == "restriction") {
        RestrictionType type = (RestrictionType) std::stoi(tag.second);

        switch (type) {

          case RestrictionType::kNoLeftTurn:
          case RestrictionType::kNoRightTurn:
          case RestrictionType::kNoStraightOn:
          case RestrictionType::kNoUTurn:
          case RestrictionType::kOnlyRightTurn:
          case RestrictionType::kOnlyLeftTurn:
          case RestrictionType::kOnlyStraightOn:
            hasRestriction = true;
            restriction.set_type(type);
            break;
          default:
            // kNoEntry and kNoExit not supported.
            return;
        }
      }
      //sample with date time.  1168738
      else if (tag.first == "hour_on") {

        std::size_t found = tag.second.find(":");
        if (found == std::string::npos)
          return;

        std::stringstream stream(tag.second);
        uint32_t hour, min;

        stream >> hour;
        stream.ignore();
        stream >> min;

        restriction.set_hour_on(hour);
        restriction.set_minute_on(min);
      }
      else if  (tag.first == "hour_off") {

        std::size_t found = tag.second.find(":");
        if (found == std::string::npos)
          return;

        std::stringstream stream(tag.second);
        uint32_t hour, min;

        stream >> hour;
        stream.ignore();
        stream >> min;

        restriction.set_hour_off(hour);
        restriction.set_minute_off(min);

      }
      else if  (tag.first == "day_on") {
        restriction.set_day_on((DOW) std::stoi(tag.second));
      }
      else if  (tag.first == "day_off") {
        restriction.set_day_off((DOW) std::stoi(tag.second));
      }
    }

    if (isRoad && isRoute && !ref.empty() && !network.empty()) {

      std::vector<std::string> net = GetTagTokens(network,':');

      if (net.size() != 2)
        return;

      std::string reference = net.at(1) + " " + ref;// US 51 or I 95

      std::string direction;

      for (const auto& member : members) {

        if (member.role.empty() || member.role == "forward" || member.role == "backward")
          continue;

        direction = member.role;

        boost::algorithm::to_lower(direction);
        direction[0] = std::toupper(direction[0]);

        // TODO:  network=e-road with int_ref=E #
        if ((boost::starts_with(direction, "North (")
            || boost::starts_with(direction, "South (")
            || boost::starts_with(direction, "East (")
            || boost::starts_with(direction, "West (")) || direction == "North"
            || direction == "South" || direction == "East"
            || direction == "West") {
          auto iter = osmdata_.way_ref.find(member.member_id);
          if (iter != osmdata_.way_ref.end())
            osmdata_.way_ref[member.member_id] = iter->second + ";" + reference + "|"
                + direction;
          else
            osmdata_.way_ref[member.member_id] = reference + "|" + direction;
        }
      }
    }
    else if (isRestriction && hasRestriction) {

      for (const auto& member : members) {

        // from and to must be of type 1(way).  via must be of type 0(node)
        if (member.role == "from" && member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY)
          from_way_id = member.member_id;
        else if (member.role == "to" && member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY)
          restriction.set_to(member.member_id);
        else if (member.role == "via" && member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_NODE)
          restriction.set_via(member.member_id);
      }

      // Add the restriction to the list.  For now only support simple restrictions.
      if (from_way_id != 0 && restriction.via() && restriction.to())
        osmdata_.restrictions.insert(RestrictionsMap::value_type(from_way_id, restriction));
    }
  }

 protected:
  // List of the tile levels to be created
  TileHierarchy tile_hierarchy_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData& osmdata_;

  // Mark the OSM Node Ids used by ways
  IdTable shape_, intersection_;
};

}

namespace valhalla {
namespace mjolnir {

OSMData PBFGraphParser::Parse(const boost::property_tree::ptree& pt, const std::vector<std::string>& input_files) {
  //TODO: option 1: each one threads makes an osmdata and we splice them together at the end
  //option 2: synchronize around adding things to a single osmdata. will have to test to see
  //which is the least expensive (memory and speed). leaning towards option 2
  unsigned int threads = std::max(static_cast<unsigned int>(1), pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));

  // Create OSM data. Set the member pointer so that the parsing callback methods can use it.
  OSMData osmdata{};
  graph_callback callback(pt, osmdata);
  OSMPBF::Parser parser(callback);

  // Parse the ways and find all node Ids needed (those that are part of a
  // way's node list. Iterate through each pbf input file.
  auto t1 = std::chrono::high_resolution_clock::now();
  for (const auto& input_file : input_files) {
    parser.parse(input_file, OSMPBF::Interest::WAYS);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("Parsing ways took " + std::to_string(msecs) + " ms");
  LOG_INFO("Routable ways count = " + std::to_string(osmdata.ways.size()));
  LOG_INFO("Number of noderefs = " + std::to_string(osmdata.noderefs.size()));

  // Shrink the OSM ways vector and the OSM node reference vector (list of
  // nodes that the ways include).
  osmdata.ways.shrink_to_fit();
  osmdata.noderefs.shrink_to_fit();

  // Parse relations.
  t1 = std::chrono::high_resolution_clock::now();
  for (const auto& input_file : input_files) {
    parser.parse(input_file, OSMPBF::Interest::RELATIONS);
  }
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("Parsing relations took " + std::to_string(msecs) + " ms");
  LOG_INFO("Simple restrictions count = " + std::to_string(osmdata.restrictions.size()));
  LOG_INFO("Sizeof OSMRestriction = " + std::to_string(sizeof(OSMRestriction)));

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  // TODO: we know how many knows we expect, stop early once we have that many
  t1 = std::chrono::high_resolution_clock::now();
  LOG_INFO("Parsing nodes but only keeping " + std::to_string(osmdata.node_count));
  osmdata.nodes.reserve(osmdata.node_count);
  for (const auto& input_file : input_files) {
    parser.parse(input_file, OSMPBF::Interest::NODES);
  }
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("Parsing nodes took " + std::to_string(msecs) + " ms");
  LOG_INFO("Nodes included on routable ways, count = " + std::to_string(osmdata.nodes.size()));

  // Sort the OSM nodes vector by OSM Id
  std::sort(osmdata.nodes.begin(), osmdata.nodes.end());

  // Log some information about extra node information and names
  LOG_INFO("Number of node refs (exits) = " + std::to_string(osmdata.node_ref.size()));
  LOG_INFO("Number of node exit_to = " + std::to_string(osmdata.node_exit_to.size()));
  LOG_INFO("Number of node names = " + std::to_string(osmdata.node_name.size()));
  LOG_INFO("Number of way refs = " + std::to_string(osmdata.node_ref.size()));
  LOG_INFO("Ref Names:");
  osmdata.ref_offset_map.Log();
  LOG_INFO("Names");
  osmdata.name_offset_map.Log();

  // Return OSM data
  return osmdata;
}


}
}
